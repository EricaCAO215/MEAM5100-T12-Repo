/*
 * US-100 非阻塞驱动 (基于 PingSerial 逻辑)
 * 适配: ESP32 Feather S2
 * 版本: 原始数据版 (无 Offset 补偿)
 */

#include <HardwareSerial.h>

// --- 硬件配置 ---
// ESP32 Feather S2 专用引脚
const int RX_PIN = 44; // 接 US-100 的 TX
const int TX_PIN = 43; // 接 US-100 的 RX

// 这里的 Offset 设为 0，输出原汁原味的测量值
const float DISTANCE_OFFSET = 0.0; 

HardwareSerial US100Serial(1);

// 状态机变量
unsigned long lastRequestTime = 0;
const int UPDATE_INTERVAL = 100; // 每 100ms 测一次
bool waitingForData = false;

void setup() {
  Serial.begin(115200);
  // 等待 USB 串口连接
  while (!Serial && millis() < 2000);

  // 初始化传感器串口
  US100Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("--- US-100 Raw Data Mode ---");
  Serial.println("System Ready. Jumper MUST be ON.");
}

void loop() {
  // ---------------------------------------------------------
  // 1. 发送请求 (Request)
  // ---------------------------------------------------------
  // 只有当“没在等数据”且“时间到了”才发送
  if (!waitingForData && (millis() - lastRequestTime >= UPDATE_INTERVAL)) {
    
    // 关键步骤：清空串口缓存
    // 这一步能防止读到上一次没处理完的陈旧数据，保证数据“新鲜”
    while (US100Serial.available() > 0) { US100Serial.read(); }
    
    US100Serial.write(0x55); // 发送测距命令
    lastRequestTime = millis();
    waitingForData = true;   // 标记状态：正在等待回音
  }

  // ---------------------------------------------------------
  // 2. 接收数据 (Receive)
  // ---------------------------------------------------------
  if (waitingForData) {
    // 检查是否有 2 个字节回来了
    if (US100Serial.available() >= 2) {
      unsigned int highByte = US100Serial.read();
      unsigned int lowByte = US100Serial.read();

      unsigned int distanceMm = (highByte * 256) + lowByte;
      
      // 简单的合法性检查
      if (distanceMm > 0 && distanceMm < 4500) {
        float distanceCm = distanceMm / 10.0;
        
        // 直接打印原始值，不加 Offset
        Serial.print("Raw Dist: ");
        Serial.print(distanceCm);
        Serial.println(" cm");
      } 
      else {
        // 如果数据异常 (比如 0 或太远)，打印个标记
        // Serial.println("(Ignored invalid)"); 
      }

      waitingForData = false; // 任务完成，重置状态
    }
    
    // 超时保护：如果发了命令超过 50ms 还没回音，放弃这次
    if (millis() - lastRequestTime > 50) {
      waitingForData = false; 
    }
  }

  // ---------------------------------------------------------
  // 3. 非阻塞演示
  // ---------------------------------------------------------
  // 代码运行到这里时，即使传感器正在测距，CPU 也是空闲的
  // 您可以在这里加 blink 代码或者处理 WiFi，完全不卡
}