// LIS3MDL Magnetometer Heading (带 Z 轴自动爬坡检测)
// 适配板子: Unexpected Maker FeatherS2
// 接线: SDA -> GPIO 8, SCL -> GPIO 9

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <math.h> 

#define MY_SDA 8
#define MY_SCL 9
#define RAMP_ANGLE_DEG 13.0 

Adafruit_LIS3MDL lis3mdl;

// --- 校准数据 (XYZ都需要) ---
float mag_min_x = -4100, mag_max_x = 3332;
float mag_min_y = -2774, mag_max_y = 5041;
float mag_min_z = -10967, mag_max_z = 1000;

// --- 爬坡检测阈值 (需要你实验测定) ---
// 假设平地 Z 读数是 -4000，坡上是 -2000
// 那么阈值可以设为 -3000
// 如果 Z > -3000，说明在爬坡 (这只是举例，取决于你的磁场环境)
float z_flat_avg = 0;   // 平地 Z 均值 (占位符)
float z_climb_avg = 0;  // 坡上 Z 均值 (占位符)
float z_threshold = 0;  // 判定阈值

// 简单的低通滤波变量
float filtered_z = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("LIS3MDL Auto-Tilt Demo");

  Wire.begin(MY_SDA, MY_SCL);

  if (!lis3mdl.begin_I2C()) {          
    if (!lis3mdl.begin_I2C(0x1E)) while (1) delay(10);
  }
  
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, true, false, true);

  // 初始化滤波值为第一次读数
  lis3mdl.read();
  filtered_z = lis3mdl.z;
}

void loop() {
  lis3mdl.read();      
  float raw_x = lis3mdl.x;
  float raw_y = lis3mdl.y;
  float raw_z = lis3mdl.z;

  // 1. 校准更新逻辑
  bool updated = false;
  if (raw_x < mag_min_x) { mag_min_x = raw_x; updated = true; }
  if (raw_x > mag_max_x) { mag_max_x = raw_x; updated = true; }
  if (raw_y < mag_min_y) { mag_min_y = raw_y; updated = true; }
  if (raw_y > mag_max_y) { mag_max_y = raw_y; updated = true; }
  if (raw_z < mag_min_z) { mag_min_z = raw_z; updated = true; }
  if (raw_z > mag_max_z) { mag_max_z = raw_z; updated = true; }

  // 打印校准数据 (仅当数据更新或每隔一段时间打印一次)
  if (updated) {
    Serial.print("Calib Update: ");
    Serial.print("MinX:"); Serial.print(mag_min_x);
    Serial.print(" MaxX:"); Serial.print(mag_max_x);
    Serial.print(" MinY:"); Serial.print(mag_min_y);
    Serial.print(" MaxY:"); Serial.print(mag_max_y);
    Serial.print(" MinZ:"); Serial.print(mag_min_z);
    Serial.print(" MaxZ:"); Serial.println(mag_max_z);
  }

  // 2. Z轴低通滤波 (平滑数据，防止抖动误判)
  // filtered_z = 0.9 * 老值 + 0.1 * 新值
  filtered_z = 0.9 * filtered_z + 0.1 * raw_z;

  // 3. 自动判断是否在爬坡
  // 注意：这里的逻辑取决于 Z 值是变大还是变小，需要你实测
  // 这里假设 Z 值在坡上会显著偏离平地值
  // 你需要先手动填入 z_threshold
  // 比如: if (abs(filtered_z - z_flat_avg) > 1000)
  
  bool is_climbing = false;
  
  // 调试模式：打印 Z 值帮你找规律
  Serial.print("Z_Raw: "); Serial.print(raw_z);
  Serial.print(" \tZ_Filtered: "); Serial.print(filtered_z);

  // 简单的阈值判断逻辑示例 (你需要根据实测修改判断条件 > 或 <)
  // if (filtered_z > -3000) is_climbing = true; 
  
  // 目前先手动控制，观察 Z 值变化
  Serial.print(" \tStatus: ");
  Serial.print(is_climbing ? "CLIMB" : "FLAT");

  // 4. 硬磁校准
  float x = raw_x - (mag_min_x + mag_max_x) / 2.0;
  float y = raw_y - (mag_min_y + mag_max_y) / 2.0;
  float z = raw_z - (mag_min_z + mag_max_z) / 2.0;

  // 5. 倾斜补偿计算
  float pitch_deg = is_climbing ? RAMP_ANGLE_DEG : 0;
  float roll_rad = 0; 
  float pitch_rad = pitch_deg * PI / 180.0; 

  float XH = x * cos(pitch_rad) + z * sin(pitch_rad);
  float YH = x * sin(roll_rad) * sin(pitch_rad) + y * cos(roll_rad) - z * sin(roll_rad) * cos(pitch_rad);

  float heading_rad = atan2(YH, XH);
  float heading_deg = heading_rad * 180.0 / PI;
  if (heading_deg < 0) heading_deg += 360.0;

  Serial.print(" \tHeading: "); 
  Serial.println(heading_deg);

  delay(100); 
}