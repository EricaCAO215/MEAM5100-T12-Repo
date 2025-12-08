
// /*
//  * Board: ESP32-S2 (Master) - Reactive Navigation Mode
//  * Role: Web Server, Sensor, PID, Motor Control, State Machine
//  */
// #include <Wire.h>
// #include <VL53L0X.h>
// #include <HardwareSerial.h>
// #include <WiFi.h>
// #include <WebServer.h>

// // ===================== 1. 全局定义和引脚 =====================
// // I2C (ToF)
// #define MY_SDA 8
// #define MY_SCL 9

// // US-100 超声波
// const int US100_RX_PIN = 44;
// const int US100_TX_PIN = 43;
// HardwareSerial US100Serial(1);

// // 【新电机和编码器引脚】
// #define INVERT_M1_ENCODER false // M1 编码器方向配置
// #define INVERT_M2_ENCODER true  // M2 编码器方向配置

// const int M1_ENCODER_A_PIN = 7;
// const int M1_ENCODER_B_PIN = 3;
// const int M1_RPWM_PIN = 33;
// const int M1_LPWM_PIN = 38;

// const int M2_ENCODER_A_PIN = 18;
// const int M2_ENCODER_B_PIN = 14;
// const int M2_RPWM_PIN = 6;
// const int M2_LPWM_PIN = 5;

// // ===================== 2. 变量与对象 =====================
// VL53L0X tofSensor;
// WebServer server(80);

// // WiFi 配置
// const char* ssid = "ESP32_Robot_Master";
// const char* password = "12345678";
// IPAddress local_ip(192, 168, 10, 1);
// IPAddress gateway(192, 168, 10, 1);
// IPAddress subnet(255, 255, 255, 0);

// // 传感器数据
// float val_front = 0.0;
// float val_side = 0.0;
// bool obs_front = false;

// // PID 参数
// float Kp1 = 3.0, Ki1 = 0.0, Kd1 = 0.1;
// float Kp2 = 2.93, Ki2 = 0.0, Kd2 = 0.1;
// const float MAX_SPEED_RPM = 60.0;
// const float COUNTS_PER_REV = 4480.0;

// // 运动控制变量
// volatile float target_rpm_m1 = 0.0;
// volatile float target_rpm_m2 = 0.0;
// float current_rpm1 = 0; 
// float current_rpm2 = 0;
// int pwm1 = 0; int pwm2 = 0;

// // 编码器计时变量
// volatile long M1_duration = 0;
// volatile int M1_encoder0PinALast = LOW;
// volatile long M2_duration = 0;
// volatile int M2_encoder0PinALast = LOW;

// // 传感器计时
// unsigned long lastUS100Time = 0;
// bool waitingForUS100 = false;
// unsigned long last_PID_time = 0;
// const int PID_INTERVAL_MS = 100;

// // 【导航状态和循墙参数】
// #define NAV_STATE_STOP      0
// #define NAV_STATE_FORWARD   1 
// #define NAV_STATE_TURNING   2
// #define NAV_STATE_WALL_FOLLOW 3 
// int nav_state = NAV_STATE_STOP;

// const float AUTO_FORWARD_RPM = 30.0; 
// const float OBSTACLE_THRESHOLD_CM = 20.0;
// const float WALL_TARGET_DISTANCE_CM = 20.0; 
// const float WALL_FOLLOW_KP = 2.0;           
// const float TURN_SPEED_RPM = 30.0;          
// const unsigned long TURN_DURATION_MS = 1000; 
// unsigned long turn_start_time = 0; 

// // ===================== 3. PID 类 / 中断函数 (不变) =====================
// class SimplePID { 
//   public: 
//     float kp,ki,kd,integral=0,prev_error=0; 
//     SimplePID(float p,float i,float d):kp(p),ki(i),kd(d){} 
//     int compute(float t,float c,float dt){
//       float e=t-c;
//       integral+=e*dt;
//       integral=constrain(integral,-255,255);
//       float d_val=(e-prev_error)/dt;
//       prev_error=e;
//       return constrain(kp*e+ki*integral+kd*d_val,-255,255);
//     } 
//     void reset(){integral=0;prev_error=0;}
// };
// SimplePID pid1(Kp1,Ki1,Kd1); 
// SimplePID pid2(Kp2,Ki2,Kd2);

// void IRAM_ATTR wheelSpeed_M1(){
//   int s = digitalRead(M1_ENCODER_A_PIN);
//   if(s != M1_encoder0PinALast){
//     int val = digitalRead(M1_ENCODER_B_PIN);
//     bool dir = (val != s); 
//     if (INVERT_M1_ENCODER) dir = !dir; 
//     if(dir) M1_duration++; else M1_duration--;
//   }
//   M1_encoder0PinALast = s;
// }

// void IRAM_ATTR wheelSpeed_M2(){
//   int s = digitalRead(M2_ENCODER_A_PIN);
//   if(s != M2_encoder0PinALast){
//     int val = digitalRead(M2_ENCODER_B_PIN);
//     bool dir = (val != s); 
//     if (INVERT_M2_ENCODER) dir = !dir; 
//     if(dir) M2_duration++; else M2_duration--;
//   }
//   M2_encoder0PinALast = s;
// }

// void set_motor_pwm(int r,int l,int v){if(v>0){ledcWrite(l,0);ledcWrite(r,v);}else{ledcWrite(l,abs(v));ledcWrite(r,0);}}


// // ===================== 4. 导航逻辑 (核心修改) =====================
// void runNavigationLogic() {
//     // 【增强避障条件】：只要前方障碍物小于 20cm，就必须转弯。
//     bool frontal_blockage = (val_front > 0 && val_front < OBSTACLE_THRESHOLD_CM);
    
//     float base_speed = AUTO_FORWARD_RPM;

//     // 状态机处理
//     switch (nav_state) {
//         case NAV_STATE_FORWARD:
//             // 初始前进状态，立即切换到循墙
//             if (frontal_blockage) {
//                 // 如果检测到前方障碍，立刻开始左转
//                 nav_state = NAV_STATE_TURNING;
//                 turn_start_time = millis();
//                 target_rpm_m1 = -TURN_SPEED_RPM; 
//                 target_rpm_m2 = TURN_SPEED_RPM;
//                 pid1.reset(); 
//                 pid2.reset();
//             } else {
//                 nav_state = NAV_STATE_WALL_FOLLOW;
//             }
//             break;
            
//         case NAV_STATE_WALL_FOLLOW:
//             if (frontal_blockage) {
//                 // 只要前方被堵住 (无论右侧如何)，立刻进入转向
//                 nav_state = NAV_STATE_TURNING;
//                 turn_start_time = millis();
//                 target_rpm_m1 = -TURN_SPEED_RPM; 
//                 target_rpm_m2 = TURN_SPEED_RPM;
//                 pid1.reset(); 
//                 pid2.reset();
//             } else {
//                 // 循墙控制 (右侧)
//                 float wall_error = WALL_TARGET_DISTANCE_CM - val_side;
//                 float steer_command = wall_error * WALL_FOLLOW_KP;
//                 steer_command = constrain(steer_command, -TURN_SPEED_RPM, TURN_SPEED_RPM); 

//                 // 应用差速
//                 target_rpm_m1 = base_speed - steer_command; // 左轮减速/加速 (左转)
//                 target_rpm_m2 = base_speed + steer_command; // 右轮加速/减速 (左转)
                
//                 // 确保速度不为负 (RPM 为负表示反转，但这里是循墙，通常只前进)
//                 if (target_rpm_m1 < 0) target_rpm_m1 = 0;
//             }
//             break;

//         case NAV_STATE_TURNING:
//             if (millis() - turn_start_time >= TURN_DURATION_MS) {
//                 // 转向完成，恢复循墙状态
//                 nav_state = NAV_STATE_WALL_FOLLOW;
//                 pid1.reset(); 
//                 pid2.reset();
//             } else {
//                 // 继续转向 (原地左转)
//             }
//             break;

//         case NAV_STATE_STOP:
//         default:
//             target_rpm_m1 = 0.0;
//             target_rpm_m2 = 0.0;
//             break;
//     }
// }


// // ===================== 5. Web Server 函数 (修改 Auto Start/Stop) =====================
// const char index_html[] PROGMEM = R"rawliteral(
// <!DOCTYPE HTML><html>
// <head>
// <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
// <meta charset="utf-8">
// <title>ROBA Master</title>
// <style>
// body { font-family: sans-serif; text-align: center; margin:0; background: #f4f4f4; color: #333; touch-action: none; }
// h2 { margin: 15px 0; color: #444; }

// .panel { display: flex; justify-content: center; gap: 15px; margin: 20px; }
// .card { background: #fff; padding: 15px; border-radius: 12px; width: 45%; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
// .val { font-size: 2rem; font-weight: bold; color: #2c3e50; }
// .lbl { font-size: 0.9rem; color: #7f8c8d; margin-bottom: 5px; }

// .danger { border: 3px solid #e74c3c; background: #fadbd8; }
// .safe { border: 3px solid #2ecc71; }

// #joy-zone {
// position: relative; width: 240px; height: 240px; margin: 20px auto;
// background: #dde; border-radius: 50%; border: 4px solid #bbb;
// box-shadow: inset 0 0 20px rgba(0,0,0,0.1);
// }
// #joy-knob {
// position: absolute; width: 80px; height: 80px; background: #3498db;
// border-radius: 50%; top: 50%; left: 50%; transform: translate(-50%, -50%);
// box-shadow: 0 5px 15px rgba(0,0,0,0.3);
// }
// .rpm { font-family: monospace; font-size: 1.2rem; margin-top: 20px; color: #555; }
// </style>
// </head>
// <body>
// <h2>ROBA Master Control</h2>

// <div class="panel">
// <div id="c-front" class="card safe">
// <div class="lbl">FRONT (ToF)</div>
// <div id="v-front" class="val">--</div>
// </div>
// <div id="c-side" class="card safe">
// <div class="lbl">SIDE (US)</div>
// <div id="v-side" class="val">--</div>
// </div>
// </div>

// <div id="joy-zone"><div id="joy-knob"></div></div>

// <div class="rpm">
// L: <span id="r1">0</span> | R: <span id="r2">0</span>
// </div>

// <button onclick="sendCommand('auto_start')">AUTO FORWARD (Reactive)</button>
// <button onclick="sendCommand('auto_stop')">STOP AUTO</button>

// <script>
// var zone = document.getElementById("joy-zone");
// var knob = document.getElementById("joy-knob");
// var maxR = 80;
// var rect, cx, cy, drag=false, lastT=0;

// function setRect(){ rect=zone.getBoundingClientRect(); cx=rect.width/2; cy=rect.height/2; }
// setRect(); window.onresize=setRect;

// function start(e){ drag=true; e.preventDefault(); }
// function end(){ drag=false; knob.style.transform="translate(-50%,-50%)"; send(0,0); }
// function move(e){
// if(!drag)return; e.preventDefault();
// var tx=e.touches?e.touches[0].clientX:e.clientX;
// var ty=e.touches?e.touches[0].clientY:e.clientY;
// var x=tx-rect.left-cx; var y=ty-rect.top-cy;
// var d=Math.sqrt(x*x+y*y);
// if(d>maxR){ var a=Math.atan2(y,x); x=Math.cos(a)*maxR; y=Math.sin(a)*maxR; }
// knob.style.transform=`translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;

// if(Date.now()-lastT>100){
// send(Math.round(x/maxR*100), Math.round(y/maxR*-100));
// lastT=Date.now();
// }
// }

// zone.addEventListener("mousedown",start); zone.addEventListener("touchstart",start);
// document.addEventListener("mousemove",move); document.addEventListener("touchmove",move);
// document.addEventListener("mouseup",end); document.addEventListener("touchend",end);

// function sendCommand(cmd){ fetch('/'+cmd); } 

// function send(x,y){ fetch("/joy?x="+x+"&y="+y); }

// setInterval(()=>{
// fetch("/data").then(r=>r.json()).then(d=>{
// document.getElementById("v-front").innerText = d.f.toFixed(1);
// document.getElementById("v-side").innerText = d.s.toFixed(1);
// document.getElementById("r1").innerText = d.m1.toFixed(1);
// document.getElementById("r2").innerText = d.m2.toFixed(1);

// document.getElementById("c-front").className = d.f < 20.0 ? "card danger" : "card safe";
// document.getElementById("c-side").className = d.s < 16.0 ? "card danger" : "card safe";
// });
// }, 200);
// </script>
// </body>
// </html>
// )rawliteral";

// void handleRoot(){server.send(200,"text/html",index_html);}

// // 【修改】自动模式开始：进入 NAV_STATE_FORWARD
// void handleAutoStart() {
//     if (nav_state == NAV_STATE_STOP) {
//         nav_state = NAV_STATE_FORWARD;
//         pid1.reset(); 
//         pid2.reset(); 
//         turn_start_time = millis(); // 设置初始时间戳
//     }
//     server.send(200, "text/plain", "Auto Mode Started");
// }

// // 【修改】自动模式停止：进入 NAV_STATE_STOP
// void handleAutoStop() {
//     nav_state = NAV_STATE_STOP;
//     target_rpm_m1 = 0.0;
//     target_rpm_m2 = 0.0;
//     pid1.reset(); 
//     pid2.reset(); 
//     server.send(200, "text/plain", "Auto Mode Stopped");
// }

// // 【修改】摇杆处理：退出自动模式
// void handleJoy(){
//     if (nav_state != NAV_STATE_STOP) {
//         nav_state = NAV_STATE_STOP;
//         target_rpm_m1 = 0.0;
//         target_rpm_m2 = 0.0;
//         pid1.reset(); 
//         pid2.reset(); 
//     }
    
//     if(server.hasArg("x")){
//         int x=server.arg("x").toInt();
//         int y=server.arg("y").toInt();
        
//         if(obs_front && y > 0) y = 0;

//         float left = (float)y + (float)x;
//         float right = (float)y - (float)x;
//         float m = max(abs(left), abs(right));
//         if(m > 100) { left = left/m*100; right = right/m*100; }
        
//         target_rpm_m1 = (left / 100.0) * MAX_SPEED_RPM;
//         target_rpm_m2 = (right / 100.0) * MAX_SPEED_RPM;

//         if (x==0 && y==0) { pid1.reset(); pid2.reset(); }
        
//         server.send(200,"text/plain","OK");
//     }
// }

// void handleData(){
//     String json="{";
//     json+="\"f\":"+String(val_front)+",";
//     json+="\"s\":"+String(val_side)+",";
//     json+="\"m1\":"+String(current_rpm1, 1)+","; 
//     json+="\"m2\":"+String(current_rpm2, 1);     
//     json+="}";
//     server.send(200,"application/json",json);
// }

// // ===================== 6. Setup & Loop =====================

// void setup() {
//     Serial.begin(115200);

//     // 1. 电机和编码器引脚配置
//     pinMode(M1_ENCODER_A_PIN, INPUT_PULLUP); pinMode(M1_ENCODER_B_PIN, INPUT_PULLUP);
//     pinMode(M2_ENCODER_A_PIN, INPUT_PULLUP); pinMode(M2_ENCODER_B_PIN, INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A_PIN), wheelSpeed_M1, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A_PIN), wheelSpeed_M2, CHANGE);
    
//     // PWM 引脚绑定
//     ledcAttach(M1_RPWM_PIN, 20000, 8); ledcAttach(M1_LPWM_PIN, 20000, 8);
//     ledcAttach(M2_RPWM_PIN, 20000, 8); ledcAttach(M2_LPWM_PIN, 20000, 8);

//     // 2. WiFi AP
//     WiFi.softAPConfig(local_ip, gateway, subnet);
//     WiFi.softAP(ssid, password);
//     server.on("/", handleRoot);
//     server.on("/joy", handleJoy);
//     server.on("/data", handleData);
//     server.on("/auto_start", handleAutoStart); 
//     server.on("/auto_stop", handleAutoStop);   
//     server.begin();

//     // 3. 初始化 ToF - 修正 VL53L0X 常量名
//     Wire.begin(MY_SDA, MY_SCL);
//     tofSensor.init();
//     tofSensor.setTimeout(500);
//     tofSensor.setSignalRateLimit(0.1);
//     tofSensor.setMeasurementTimingBudget(50000);
//     tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
//     tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

//     // 4. 初始化 US-100
//     US100Serial.begin(9600, SERIAL_8N1, US100_RX_PIN, US100_TX_PIN);
// }

// void loop() {
//     server.handleClient();
    
//     // ==========================================================
//     // 传感器读取
//     // ==========================================================
//     unsigned long now = millis();

//     // --- ToF ---
//     uint16_t mm = tofSensor.readRangeSingleMillimeters();
//     if (!tofSensor.timeoutOccurred()) val_front = mm / 10.0;
//     else val_front = -1.0;
    
//     obs_front = (val_front > 0 && val_front < 20.0);

//     // --- US-100 ---
//     if (!waitingForUS100 && (now - lastUS100Time >= 100)) {
//         while (US100Serial.available()) US100Serial.read();
//         US100Serial.write(0x55);
//         lastUS100Time = now;
//         waitingForUS100 = true;
//     }

//     if (waitingForUS100) {
//         if (US100Serial.available() >= 2) {
//             unsigned int h = US100Serial.read();
//             unsigned int l = US100Serial.read();
//             unsigned int dist = (h * 256) + l;
//             if (dist > 0 && dist < 4500) val_side = dist / 10.0;
//             waitingForUS100 = false;
//         }
//         if (now - lastUS100Time > 50) waitingForUS100 = false;
//     }

//     // 【新增】导航逻辑
//     if (nav_state != NAV_STATE_STOP) {
//         runNavigationLogic(); 
//     }
    
//     // ================== PID 控制循环 (每 100ms) ==================
//     if (millis() - last_PID_time >= PID_INTERVAL_MS) {
//         float dt = (millis() - last_PID_time) / 1000.0;
        
//         // 1. 读取编码器和计算 RPM
//         noInterrupts(); 
//         long p1=M1_duration; M1_duration=0; 
//         long p2=M2_duration; M2_duration=0; 
//         interrupts();
        
//         current_rpm1 = (p1 / COUNTS_PER_REV) * 60.0 / dt;
//         current_rpm2 = (p2 / COUNTS_PER_REV) * 60.0 / dt;
        
//         // 2. 计算 PWM
//         if (abs(target_rpm_m1) > 0.1) pwm1 = pid1.compute(target_rpm_m1, current_rpm1, dt); else pwm1 = 0;
//         if (abs(target_rpm_m2) > 0.1) pwm2 = pid2.compute(target_rpm_m2, current_rpm2, dt); else pwm2 = 0;

//         // 3. 设置电机 PWM
//         set_motor_pwm(M1_RPWM_PIN, M1_LPWM_PIN, pwm1);
//         set_motor_pwm(M2_RPWM_PIN, M2_LPWM_PIN, pwm2);
        
//         last_PID_time = millis();
//     }
//     // ==========================================================
// }


/*
 * Board: ESP32-S2 (Master) - Reactive Navigation Mode
 * Role: Web Server, Sensor, PID, Motor Control, State Machine
 * * 导航逻辑:
 * 1. 目标循墙距离改为 16.0 cm。
 * 2. 运行时，执行右侧循墙 (WALL_FOLLOW_KP) 逻辑，保持 16.0cm 距离。
 */
#include <Wire.h>
#include <VL53L0X.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>

// ===================== 1. 全局定义和引脚 =====================
// I2C (ToF)
#define MY_SDA 8
#define MY_SCL 9

// US-100 超声波
const int US100_RX_PIN = 44;
const int US100_TX_PIN = 43;
HardwareSerial US100Serial(1);

// 【新电机和编码器引脚】
#define INVERT_M1_ENCODER false // M1 编码器方向配置
#define INVERT_M2_ENCODER true  // M2 编码器方向配置

const int M1_ENCODER_A_PIN = 7;
const int M1_ENCODER_B_PIN = 3;
const int M1_RPWM_PIN = 33;
const int M1_LPWM_PIN = 38;

const int M2_ENCODER_A_PIN = 18;
const int M2_ENCODER_B_PIN = 14;
const int M2_RPWM_PIN = 6;
const int M2_LPWM_PIN = 5;

// ===================== 2. 变量与对象 =====================
VL53L0X tofSensor;
WebServer server(80);

// WiFi 配置
const char* ssid = "ESP32_Robot_Master";
const char* password = "12345678";
IPAddress local_ip(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

// 传感器数据
float val_front = 0.0;
float val_side = 0.0;
bool obs_front = false;

// PID 参数
float Kp1 = 3.0, Ki1 = 0.0, Kd1 = 0.1;
float Kp2 = 2.93, Ki2 = 0.0, Kd2 = 0.1;
const float MAX_SPEED_RPM = 60.0;
const float COUNTS_PER_REV = 4480.0;

// 运动控制变量
volatile float target_rpm_m1 = 0.0;
volatile float target_rpm_m2 = 0.0;
float current_rpm1 = 0; 
float current_rpm2 = 0;
int pwm1 = 0; int pwm2 = 0;

// 编码器计时变量
volatile long M1_duration = 0;
volatile int M1_encoder0PinALast = LOW;
volatile long M2_duration = 0;
volatile int M2_encoder0PinALast = LOW;

// 传感器计时
unsigned long lastUS100Time = 0;
bool waitingForUS100 = false;
unsigned long last_PID_time = 0;
const int PID_INTERVAL_MS = 100;

// 【导航状态和循墙参数】
#define NAV_STATE_STOP      0
#define NAV_STATE_FORWARD   1 
#define NAV_STATE_TURNING   2
#define NAV_STATE_WALL_FOLLOW 3 
int nav_state = NAV_STATE_STOP;

const float AUTO_FORWARD_RPM = 30.0; // 基础前进和循墙速度
const float OBSTACLE_THRESHOLD_CM = 20.0;
const float WALL_TARGET_DISTANCE_CM = 16.0; // 【已修改】目标循墙距离 (16.0 cm)
const float WALL_FOLLOW_KP = 2.0;           // 循墙 P 控制增益 (需调参)
const float TURN_SPEED_RPM = 30.0;          // 转弯速度
const unsigned long TURN_DURATION_MS = 1000; // 90度转弯时间，需实测调整
unsigned long turn_start_time = 0; 

// 【新增】循墙PD运行时变量 (仅为 P 控制，但保留结构以备升级)
float wall_last_error = 0.0;
unsigned long wall_last_time = 0; 

// ===================== 3. PID 类 / 中断函数 (不变) =====================
class SimplePID { 
  public: 
    float kp,ki,kd,integral=0,prev_error=0; 
    SimplePID(float p,float i,float d):kp(p),ki(i),kd(d){} 
    int compute(float t,float c,float dt){
      float e=t-c;
      integral+=e*dt;
      integral=constrain(integral,-255,255);
      float d_val=(e-prev_error)/dt;
      prev_error=e;
      return constrain(kp*e+ki*integral+kd*d_val,-255,255);
    } 
    void reset(){integral=0;prev_error=0;}
};
SimplePID pid1(Kp1,Ki1,Kd1); 
SimplePID pid2(Kp2,Ki2,Kd2);

void IRAM_ATTR wheelSpeed_M1(){
  int s = digitalRead(M1_ENCODER_A_PIN);
  if(s != M1_encoder0PinALast){
    int val = digitalRead(M1_ENCODER_B_PIN);
    bool dir = (val != s); 
    if (INVERT_M1_ENCODER) dir = !dir; 
    if(dir) M1_duration++; else M1_duration--;
  }
  M1_encoder0PinALast = s;
}

void IRAM_ATTR wheelSpeed_M2(){
  int s = digitalRead(M2_ENCODER_A_PIN);
  if(s != M2_encoder0PinALast){
    int val = digitalRead(M2_ENCODER_B_PIN);
    bool dir = (val != s); 
    if (INVERT_M2_ENCODER) dir = !dir; 
    if(dir) M2_duration++; else M2_duration--;
  }
  M2_encoder0PinALast = s;
}

void set_motor_pwm(int r,int l,int v){if(v>0){ledcWrite(l,0);ledcWrite(r,v);}else{ledcWrite(l,abs(v));ledcWrite(r,0);}}


// ===================== 4. 导航逻辑 (核心修改) =====================
void runNavigationLogic() {
    // 【增强避障条件】：只要前方障碍物小于 20cm，就必须转弯。
    bool frontal_blockage = (val_front > 0 && val_front < OBSTACLE_THRESHOLD_CM);
    
    float base_speed = AUTO_FORWARD_RPM;

    // 状态机处理
    switch (nav_state) {
        case NAV_STATE_FORWARD:
            // 初始前进状态，立即切换到循墙
            if (frontal_blockage) {
                // 如果检测到前方障碍，立刻开始左转
                nav_state = NAV_STATE_TURNING;
                turn_start_time = millis();
                target_rpm_m1 = -TURN_SPEED_RPM; 
                target_rpm_m2 = TURN_SPEED_RPM;
                pid1.reset(); 
                pid2.reset();
            } else {
                nav_state = NAV_STATE_WALL_FOLLOW;
            }
            break;
            
        case NAV_STATE_WALL_FOLLOW:
            if (frontal_blockage) {
                // 只要前方被堵住 (无论右侧如何)，立刻进入转向
                nav_state = NAV_STATE_TURNING;
                turn_start_time = millis();
                target_rpm_m1 = -TURN_SPEED_RPM; 
                target_rpm_m2 = TURN_SPEED_RPM;
                pid1.reset(); 
                pid2.reset();
            } else {
                // 循墙控制 (右侧)
                float wall_error = WALL_TARGET_DISTANCE_CM - val_side;
                float steer_command = wall_error * WALL_FOLLOW_KP;
                // 注意：这里仅使用 P 控制，如果外角抖动大，需要引入 D 项 (参考之前的建议)
                steer_command = constrain(steer_command, -TURN_SPEED_RPM, TURN_SPEED_RPM); 

                // 应用差速
                target_rpm_m1 = base_speed - steer_command; // 左轮减速/加速 (左转)
                target_rpm_m2 = base_speed + steer_command; // 右轮加速/减速 (左转)
                
                // 确保速度不为负 (RPM 为负表示反转，但这里是循墙，通常只前进)
                if (target_rpm_m1 < 0) target_rpm_m1 = 0;
            }
            break;

        case NAV_STATE_TURNING:
            if (millis() - turn_start_time >= TURN_DURATION_MS) {
                // 转向完成，恢复循墙状态
                nav_state = NAV_STATE_WALL_FOLLOW;
                pid1.reset(); 
                pid2.reset();
            } else {
                // 继续转向 (原地左转)
            }
            break;

        case NAV_STATE_STOP:
        default:
            target_rpm_m1 = 0.0;
            target_rpm_m2 = 0.0;
            break;
    }
}


// ===================== 5. Web Server 函数 (修改 Auto Start/Stop) =====================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<meta charset="utf-8">
<title>ROBA Master</title>
<style>
body { font-family: sans-serif; text-align: center; margin:0; background: #f4f4f4; color: #333; touch-action: none; }
h2 { margin: 15px 0; color: #444; }

.panel { display: flex; justify-content: center; gap: 15px; margin: 20px; }
.card { background: #fff; padding: 15px; border-radius: 12px; width: 45%; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
.val { font-size: 2rem; font-weight: bold; color: #2c3e50; }
.lbl { font-size: 0.9rem; color: #7f8c8d; margin-bottom: 5px; }

.danger { border: 3px solid #e74c3c; background: #fadbd8; }
.safe { border: 3px solid #2ecc71; }

#joy-zone {
position: relative; width: 240px; height: 240px; margin: 20px auto;
background: #dde; border-radius: 50%; border: 4px solid #bbb;
box-shadow: inset 0 0 20px rgba(0,0,0,0.1);
}
#joy-knob {
position: absolute; width: 80px; height: 80px; background: #3498db;
border-radius: 50%; top: 50%; left: 50%; transform: translate(-50%, -50%);
box-shadow: 0 5px 15px rgba(0,0,0,0.3);
}
.rpm { font-family: monospace; font-size: 1.2rem; margin-top: 20px; color: #555; }
</style>
</head>
<body>
<h2>ROBA Master Control</h2>

<div class="panel">
<div id="c-front" class="card safe">
<div class="lbl">FRONT (ToF)</div>
<div id="v-front" class="val">--</div>
</div>
<div id="c-side" class="card safe">
<div class="lbl">SIDE (US)</div>
<div id="v-side" class="val">--</div>
</div>
</div>

<div id="joy-zone"><div id="joy-knob"></div></div>

<div class="rpm">
L: <span id="r1">0</span> | R: <span id="r2">0</span>
</div>

<button onclick="sendCommand('auto_start')">AUTO FORWARD (Reactive)</button>
<button onclick="sendCommand('auto_stop')">STOP AUTO</button>

<script>
var zone = document.getElementById("joy-zone");
var knob = document.getElementById("joy-knob");
var maxR = 80;
var rect, cx, cy, drag=false, lastT=0;

function setRect(){ rect=zone.getBoundingClientRect(); cx=rect.width/2; cy=rect.height/2; }
setRect(); window.onresize=setRect;

function start(e){ drag=true; e.preventDefault(); }
function end(){ drag=false; knob.style.transform="translate(-50%,-50%)"; send(0,0); }
function move(e){
if(!drag)return; e.preventDefault();
var tx=e.touches?e.touches[0].clientX:e.clientX;
var ty=e.touches?e.touches[0].clientY:e.clientY;
var x=tx-rect.left-cx; var y=ty-rect.top-cy;
var d=Math.sqrt(x*x+y*y);
if(d>maxR){ var a=Math.atan2(y,x); x=Math.cos(a)*maxR; y=Math.sin(a)*maxR; }
knob.style.transform=`translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;

if(Date.now()-lastT>100){
send(Math.round(x/maxR*100), Math.round(y/maxR*-100));
lastT=Date.now();
}
}

zone.addEventListener("mousedown",start); zone.addEventListener("touchstart",start);
document.addEventListener("mousemove",move); document.addEventListener("touchmove",move);
document.addEventListener("mouseup",end); document.addEventListener("touchend",end);

function sendCommand(cmd){ fetch('/'+cmd); } 

function send(x,y){ fetch("/joy?x="+x+"&y="+y); }

setInterval(()=>{
fetch("/data").then(r=>r.json()).then(d=>{
document.getElementById("v-front").innerText = d.f.toFixed(1);
document.getElementById("v-side").innerText = d.s.toFixed(1);
document.getElementById("r1").innerText = d.m1.toFixed(1);
document.getElementById("r2").innerText = d.m2.toFixed(1);

document.getElementById("c-front").className = d.f < 20.0 ? "card danger" : "card safe";
document.getElementById("c-side").className = d.s < 16.0 ? "card danger" : "card safe";
});
}, 200);
</script>
</body>
</html>
)rawliteral";

void handleRoot(){server.send(200,"text/html",index_html);}

// 【修改】自动模式开始：进入 NAV_STATE_FORWARD
void handleAutoStart() {
    if (nav_state == NAV_STATE_STOP) {
        nav_state = NAV_STATE_FORWARD;
        pid1.reset(); pid2.reset(); 
        wall_last_time = millis(); // 初始化循墙时间
    }
    server.send(200, "text/plain", "Auto Mode Started");
}

// 【修改】自动模式停止：进入 NAV_STATE_STOP
void handleAutoStop() {
    nav_state = NAV_STATE_STOP;
    target_rpm_m1 = 0.0;
    target_rpm_m2 = 0.0;
    pid1.reset(); pid2.reset(); 
    wall_last_error = 0.0; // 重置循墙状态
    server.send(200, "text/plain", "Auto Mode Stopped");
}

// 【修改】摇杆处理：退出自动模式
void handleJoy(){
    if (nav_state != NAV_STATE_STOP) {
        nav_state = NAV_STATE_STOP;
        target_rpm_m1 = 0.0;
        target_rpm_m2 = 0.0;
        pid1.reset(); pid2.reset(); 
        wall_last_error = 0.0; // 重置循墙状态
    }
    
    if(server.hasArg("x")){
        int x=server.arg("x").toInt();
        int y=server.arg("y").toInt();
        
        if(obs_front && y > 0) y = 0;

        float left = (float)y + (float)x;
        float right = (float)y - (float)x;
        float m = max(abs(left), abs(right));
        if(m > 100) { left = left/m*100; right = right/m*100; }
        
        target_rpm_m1 = (left / 100.0) * MAX_SPEED_RPM;
        target_rpm_m2 = (right / 100.0) * MAX_SPEED_RPM;

        if (x==0 && y==0) { pid1.reset(); pid2.reset(); }
        
        server.send(200,"text/plain","OK");
    }
}

void handleData(){
    String json="{";
    json+="\"f\":"+String(val_front)+",";
    json+="\"s\":"+String(val_side)+",";
    json+="\"m1\":"+String(current_rpm1, 1)+","; 
    json+="\"m2\":"+String(current_rpm2, 1);     
    json+="}";
    server.send(200,"application/json",json);
}

// ===================== 6. Setup & Loop =====================

void setup() {
    Serial.begin(115200);

    // 1. 电机和编码器引脚配置
    pinMode(M1_ENCODER_A_PIN, INPUT_PULLUP); pinMode(M1_ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(M2_ENCODER_A_PIN, INPUT_PULLUP); pinMode(M2_ENCODER_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A_PIN), wheelSpeed_M1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A_PIN), wheelSpeed_M2, CHANGE);
    
    // PWM 引脚绑定
    ledcAttach(M1_RPWM_PIN, 20000, 8); ledcAttach(M1_LPWM_PIN, 20000, 8);
    ledcAttach(M2_RPWM_PIN, 20000, 8); ledcAttach(M2_LPWM_PIN, 20000, 8);

    // 2. WiFi AP
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.softAP(ssid, password);
    server.on("/", handleRoot);
    server.on("/joy", handleJoy);
    server.on("/data", handleData);
    server.on("/auto_start", handleAutoStart); 
    server.on("/auto_stop", handleAutoStop);   
    server.begin();

    // 3. 初始化 ToF - 修正 VL53L0X 常量名
    Wire.begin(MY_SDA, MY_SCL);
    tofSensor.init();
    tofSensor.setTimeout(500);
    tofSensor.setSignalRateLimit(0.1);
    tofSensor.setMeasurementTimingBudget(50000);
    tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

    // 4. 初始化 US-100
    US100Serial.begin(9600, SERIAL_8N1, US100_RX_PIN, US100_TX_PIN);
}

void loop() {
    server.handleClient();
    
    // ==========================================================
    // 传感器读取
    // ==========================================================
    unsigned long now = millis();

    // --- ToF ---
    uint16_t mm = tofSensor.readRangeSingleMillimeters();
    if (!tofSensor.timeoutOccurred()) val_front = mm / 10.0;
    else val_front = -1.0;
    
    obs_front = (val_front > 0 && val_front < 20.0);

    // --- US-100 ---
    if (!waitingForUS100 && (now - lastUS100Time >= 100)) {
        while (US100Serial.available()) US100Serial.read();
        US100Serial.write(0x55);
        lastUS100Time = now;
        waitingForUS100 = true;
    }

    if (waitingForUS100) {
        if (US100Serial.available() >= 2) {
            unsigned int h = US100Serial.read();
            unsigned int l = US100Serial.read();
            unsigned int dist = (h * 256) + l;
            if (dist > 0 && dist < 4500) val_side = dist / 10.0;
            waitingForUS100 = false;
        }
        if (now - lastUS100Time > 50) waitingForUS100 = false;
    }

    // 【新增】导航逻辑
    if (nav_state != NAV_STATE_STOP) {
        runNavigationLogic(); 
    }
    
    // ================== PID 控制循环 (每 100ms) ==================
    if (millis() - last_PID_time >= PID_INTERVAL_MS) {
        float dt = (millis() - last_PID_time) / 1000.0;
        
        // 1. 读取编码器和计算 RPM
        noInterrupts(); 
        long p1=M1_duration; M1_duration=0; 
        long p2=M2_duration; M2_duration=0; 
        interrupts();
        
        current_rpm1 = (p1 / COUNTS_PER_REV) * 60.0 / dt;
        current_rpm2 = (p2 / COUNTS_PER_REV) * 60.0 / dt;
        
        // 2. 计算 PWM
        if (abs(target_rpm_m1) > 0.1) pwm1 = pid1.compute(target_rpm_m1, current_rpm1, dt); else pwm1 = 0;
        if (abs(target_rpm_m2) > 0.1) pwm2 = pid2.compute(target_rpm_m2, current_rpm2, dt); else pwm2 = 0;

        // 3. 设置电机 PWM
        set_motor_pwm(M1_RPWM_PIN, M1_LPWM_PIN, pwm1);
        set_motor_pwm(M2_RPWM_PIN, M2_LPWM_PIN, pwm2);
        
        last_PID_time = millis();
    }
    // ==========================================================
}