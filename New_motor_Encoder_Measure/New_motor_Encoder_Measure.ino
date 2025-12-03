// /*
//  * ESP32-C3 + BTS7960 + CQR37D Motor RPM Test
//  * * 接线定义:
//  * - 编码器 A相: GPIO 5
//  * - 编码器 B相: GPIO 4
//  * - BTS7960 RPWM: GPIO 7
//  * - BTS7960 LPWM: GPIO 6
//  * - BTS7960 R_EN & L_EN: 必须连接到 3.3V 或 5V (高电平)
//  */

// // ================= 引脚定义 =================
// const int ENCODER_A_PIN = 5;  // A相
// const int ENCODER_B_PIN = 4;  // B相

// const int RPWM_PIN = 7;       // 正转 PWM
// const int LPWM_PIN = 6;       // 反转 PWM

// // ================= 电机与编码器参数 =================
// // 基础脉冲数: 16 (电机说明书)
// // 倍频模式: 2 (代码使用 CHANGE 中断，即 2 倍频)
// // 减速比: 70 (70:1 减速箱)
// // 车轮转一圈的总脉冲数 = 16 * 2 * 70 = 2240
// const float COUNTS_PER_REV = 2240.0; 

// // ================= 变量定义 =================
// volatile long duration = 0;       // 100ms 内的脉冲计数
// volatile bool Direction = true;   // 方向标志位
// volatile int encoder0PinALast = LOW;

// // PWM 设置
// const int PWM_FREQ = 10000;
// const int PWM_RES = 8;
// const int TEST_SPEED = 255;       // 测试速度 (0-255)

// // ================= 中断服务函数 =================
// // 统计 A 相的上升沿和下降沿 (2倍频)
// void IRAM_ATTR wheelSpeed() {
//   int Lstate = digitalRead(ENCODER_A_PIN);
  
//   // 检测电平变化
//   if(Lstate != encoder0PinALast) {
//     int val = digitalRead(ENCODER_B_PIN);
    
//     // 简单的正交解码逻辑
//     if(val != Lstate) {
//       Direction = true;  // Forward
//     } else {
//       Direction = false; // Reverse
//     }
    
//     // 计数
//     if(Direction) duration++;
//     else duration--;
//   }
  
//   encoder0PinALast = Lstate;
// }

// // ================= 初始化 =================
// void setup() {
//   Serial.begin(115200);
//   Serial.println("--- Motor RPM Test Start ---");

//   // 1. 配置引脚
//   pinMode(ENCODER_A_PIN, INPUT_PULLUP); 
//   pinMode(ENCODER_B_PIN, INPUT_PULLUP);

//   // 2. 配置中断 (CHANGE = 上升沿+下降沿 = 2倍频)
//   attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), wheelSpeed, CHANGE);

//   // 3. 配置电机 PWM
//   ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RES);
//   ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RES);
  
//   // 初始停止
//   ledcWrite(RPWM_PIN, 0);
//   ledcWrite(LPWM_PIN, 0);
  
//   // 读取初始状态
//   encoder0PinALast = digitalRead(ENCODER_A_PIN);
// }

// // ================= 主循环 =================
// void loop() {
//   // 1. 让电机以设定速度转动 (正转)
//   ledcWrite(LPWM_PIN, 0);
//   ledcWrite(RPWM_PIN, TEST_SPEED); 

//   // 2. 计算与打印
//   long currentPulse = duration;     // 读取当前 0.1秒内的计数
//   duration = 0;                     // 清零，准备下一次统计
  
//   // RPM 计算公式:
//   // (脉冲数 * 600) / 每圈总脉冲数
//   // 乘以 600 是因为我们每 0.1秒 (100ms) 统计一次，要转换成每分钟 (60秒)
//   float rpm = (currentPulse * 600.0) / COUNTS_PER_REV;

//   // 3. 串口打印
//   Serial.print("PWM: ");
//   Serial.print(TEST_SPEED);
//   Serial.print(" | Pulse(0.1s): ");
//   Serial.print(currentPulse);
//   Serial.print(" | Wheel RPM: ");
//   Serial.println(rpm);

//   // 采样间隔 100ms
//   delay(100);
// }

#include <Arduino.h>
#include <esp32-hal-ledc.h> 

// ===================== 全局参数 (恢复旧逻辑) =====================
// 恢复您旧代码中的参数定义 (16 * 2 * 70 = 2240.0)
const float COUNTS_PER_REV = 4480.0; 

// PWM 设置
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int TEST_SPEED = 255; // 固定 PWM 值

// 采样间隔
const int SAMPLE_TIME_MS = 100;

// ===================== 电机 1 (左) 定义 =====================
const int M1_ENCODER_A_PIN = 5;
const int M1_ENCODER_B_PIN = 4; 
const int M1_RPWM_PIN = 7; 
const int M1_LPWM_PIN = 6; 

volatile long M1_duration = 0;
volatile bool M1_Direction = true;
volatile int M1_encoder0PinALast = LOW;

// ===================== 电机 2 (右) 定义 =====================
const int M2_ENCODER_A_PIN = 18;
const int M2_ENCODER_B_PIN = 19; 
const int M2_RPWM_PIN = 0; // GPIO 0
const int M2_LPWM_PIN = 1; // GPIO 1

volatile long M2_duration = 0;
volatile bool M2_Direction = true;
volatile int M2_encoder0PinALast = LOW;


// ===================== 中断服务函数 (不变) =====================
void IRAM_ATTR wheelSpeed_M1() {
  int Lstate = digitalRead(M1_ENCODER_A_PIN);
  if(Lstate != M1_encoder0PinALast) {
    int val = digitalRead(M1_ENCODER_B_PIN);
    if(val != Lstate) M1_Direction = true;  
    else M1_Direction = false;  
    if(M1_Direction) M1_duration++;
    else M1_duration--;
  }
  M1_encoder0PinALast = Lstate;
}

void IRAM_ATTR wheelSpeed_M2() {
  int Lstate = digitalRead(M2_ENCODER_A_PIN);
  if(Lstate != M2_encoder0PinALast) {
    int val = digitalRead(M2_ENCODER_B_PIN);
    if(val != Lstate) M2_Direction = true;  
    else M2_Direction = false;  
    if(M2_Direction) M2_duration++;
    else M2_duration--;
  }
  M2_encoder0PinALast = Lstate;
}


// ===================== 电机驱动函数 (不变) =====================
void set_motor_speed(int rpwm_pin, int lpwm_pin, int speed) {
    if (speed >= 0) {
        ledcWrite(lpwm_pin, 0);          
        ledcWrite(rpwm_pin, speed); 
    } 
}


// ===================== 初始化 (不变) =====================
void setup() {
    delay(1000); 
    
    Serial.begin(115200);
    Serial.printf("--- 双电机固定速度测试启动 (Counts/Rev: %.1f) ---\n", COUNTS_PER_REV);

    // 1. 配置编码器引脚
    pinMode(M1_ENCODER_A_PIN, INPUT_PULLUP); 
    pinMode(M1_ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(M2_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(M2_ENCODER_B_PIN, INPUT_PULLUP);

    // 2. 配置中断
    attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A_PIN), wheelSpeed_M1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A_PIN), wheelSpeed_M2, CHANGE);

    // 3. 配置电机 PWM
    ledcAttach(M1_RPWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M1_LPWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M2_RPWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(M2_LPWM_PIN, PWM_FREQ, PWM_RES);
    
    // 初始停止
    set_motor_speed(M1_RPWM_PIN, M1_LPWM_PIN, 0);
    set_motor_speed(M2_RPWM_PIN, M2_LPWM_PIN, 0);
    
    // 读取初始状态
    M1_encoder0PinALast = digitalRead(M1_ENCODER_A_PIN);
    M2_encoder0PinALast = digitalRead(M2_ENCODER_A_PIN);
}

// ===================== 主循环 =====================
void loop() {
    static unsigned long last_time = 0;
    
    if (millis() - last_time >= SAMPLE_TIME_MS) {
        
        // 1. 驱动电机 (固定正转)
        set_motor_speed(M1_RPWM_PIN, M1_LPWM_PIN, TEST_SPEED);
        set_motor_speed(M2_RPWM_PIN, M2_LPWM_PIN, TEST_SPEED);

        // 我们将使用固定 100ms 采样时间进行计算，简化公式：
        // 600.0 = 60s/min / 0.1s/sample
        const float RPM_CONVERSION_FACTOR = 600.0;
        
        // --- M1 数据处理 ---
        noInterrupts(); 
        long pulse1 = M1_duration; 
        M1_duration = 0;
        interrupts();   
        
        // RPM 计算: (Pulse * 600.0) / COUNTS_PER_REV
        float rpm1 = ((float)pulse1 * RPM_CONVERSION_FACTOR) / COUNTS_PER_REV;

        // --- M2 数据处理 ---
        noInterrupts(); 
        long pulse2 = M2_duration;
        M2_duration = 0;
        interrupts();   
        
        float rpm2 = ((float)pulse2 * RPM_CONVERSION_FACTOR) / COUNTS_PER_REV;
        
        // 2. 串口打印
        Serial.printf("M1 Pulse:%4ld | M1 RPM:%.1f | M2 Pulse:%4ld | M2 RPM:%.1f\n",
                      pulse1, rpm1, pulse2, rpm2);

        last_time = millis();
    }
}