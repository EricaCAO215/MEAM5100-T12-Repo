#include "vive510.h"
#include <math.h>   // 为了 atan2 和 PI

// Vive1 / Vive2 所在的 GPIO 引脚
#define SIGNALPIN1 17   // Vive1: A0 / GPIO17
#define SIGNALPIN2 18   // Vive2: A1 / GPIO18（如用其他脚改这里）

Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);

#define FREQ 1 // in Hz

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  vive1.begin();
  vive2.begin();
  Serial.println("Two Vive trackers started");
}

uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t middle;
  if ((a <= b) && (a <= c))
    middle = (b <= c) ? b : c;  
  else if ((b <= a) && (b <= c))
    middle = (a <= c) ? a : c;
  else
    middle = (a <= b) ? a : b;
  return middle;
}

void loop() {  
  static uint16_t x1, y1, x2, y2;
  bool valid1 = false, valid2 = false;  // 记录各自数据是否有效

  // -------- Vive1 逻辑（完全照你原来的，只是变量换成 1）--------
  if (vive1.status() == VIVE_RECEIVING) {
    static uint16_t x10, y10, oldx11, oldx12, oldy11, oldy12;
    oldx12 = oldx11; oldy12 = oldy11;
    oldx11 = x10;    oldy11 = y10;
    
    x10 = vive1.xCoord();
    y10 = vive1.yCoord();
    x1 = med3filt(x10, oldx11, oldx12);
    y1 = med3filt(y10, oldy11, oldy12);

    if (x1 > 8000 || y1 > 8000 || x1 < 1000 || y1 < 1000) {
      x1 = 0; 
      y1 = 0;
    } else {
      valid1 = true;
    }
  } else {
    x1 = 0;
    y1 = 0; 
    vive1.sync(5); 
  }
  delay (10);

  // -------- Vive2 逻辑（同样复制一份）--------
  if (vive2.status() == VIVE_RECEIVING) {
    static uint16_t x20, y20, oldx21, oldx22, oldy21, oldy22;
    oldx22 = oldx21; oldy22 = oldy21;
    oldx21 = x20;    oldy21 = y20;
    
    x20 = vive2.xCoord();
    y20 = vive2.yCoord();
    x2 = med3filt(x20, oldx21, oldx22);
    y2 = med3filt(y20, oldy21, oldy22);

    if (x2 > 8000 || y2 > 8000 || x2 < 1000 || y2 < 1000) {
      x2 = 0;
      y2 = 0;
    } else {
      valid2 = true;
    }
  } else {
    x2 = 0;
    y2 = 0; 
    vive2.sync(5); 
  }

  // ---------------- 小抖动忽略滤波（不改变原逻辑） ----------------
  // 阈值可以自己调，比如 30、50、80 等
  const int THRESH = 40;

  static uint16_t fx1 = 0, fy1 = 0, fx2 = 0, fy2 = 0;
  static bool inited1 = false, inited2 = false;

  if (valid1 && x1 != 0 && y1 != 0) {
    if (!inited1) {
      fx1 = x1;
      fy1 = y1;
      inited1 = true;
    } else {
      if (abs((int)x1 - (int)fx1) < THRESH) x1 = fx1;
      else fx1 = x1;

      if (abs((int)y1 - (int)fy1) < THRESH) y1 = fy1;
      else fy1 = y1;
    }
  } else {
    // 无效时就不更新 fx1/fy1，这样下次 valid 时还能沿用
  }

  if (valid2 && x2 != 0 && y2 != 0) {
    if (!inited2) {
      fx2 = x2;
      fy2 = y2;
      inited2 = true;
    } else {
      if (abs((int)x2 - (int)fx2) < THRESH) x2 = fx2;
      else fx2 = x2;

      if (abs((int)y2 - (int)fy2) < THRESH) y2 = fy2;
      else fy2 = y2;
    }
  } else {
    // 同理，不更新 fx2/fy2
  }
  // ---------------- 抖动滤波结束 ------------------------------------

  // -------- 计算角度（在两个 Vive 都有效时） --------
  float thetaDeg = NAN;
  if (valid1 && valid2 && x1 != 0 && y1 != 0 && x2 != 0 && y2 != 0) {
    int32_t dx = (int32_t)x2 - (int32_t)x1;
    int32_t dy = (int32_t)y2 - (int32_t)y1;
    if (dx != 0 || dy != 0) {
      float thetaRad = atan2((float)dy, (float)dx);   // -pi ~ pi
      thetaDeg = thetaRad * 180.0f / PI;              // 转成度
    }
  }

  // 串口打印两个 Vive 的坐标 + 角度
  if (!isnan(thetaDeg)) {
    Serial.printf("V1 x=%u y=%u   V2 x=%u y=%u   theta=%.2f deg\n",
                  x1, y1, x2, y2, thetaDeg);
  } else {
    Serial.printf("V1 x=%u y=%u   V2 x=%u y=%u   theta=NaN\n",
                  x1, y1, x2, y2);
  }

  delay(10);
}
