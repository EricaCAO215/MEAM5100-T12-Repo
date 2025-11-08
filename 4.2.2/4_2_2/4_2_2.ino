//-------------------------------------------------------------------//
// Parameter: Kp = 3.5, Ki = , Kd = 1.18,


// #include <WiFi.h>
// #include <WebServer.h> // Library for Web Server
// #include <esp32-hal-ledc.h>
// #include <esp32-hal-adc.h>

// // ==========================================================
// // ==== Wi-Fi (AP Mode Configuration)
// // ==========================================================
// const char* AP_SSID = "ESP32_PID_CHART"; 
// const char* AP_PASS = "123456789"; 
// IPAddress AP_IP(192, 168, 4, 1);
// // ==========================================================
// // ==== Hardware Pins & PWM Parameters ====
// // ==========================================================
// const int SPEED_PIN = 6;    // PWM Speed Control (H-Bridge EN)
// const int DIR_PIN_1 = 10;   // Direction Input 1 (H-Bridge 1A)
// const int DIR_PIN_2 = 8;    // Direction Input 2 (H-Bridge 2A)
// const int ENCODER_PIN = 1;  // Encoder signal (G1)

// const int PWM_FREQ = 250;     // PWM Frequency (Hz)
// const int PWM_BITS = 8;       // Resolution (8-bit = 0-255 range)

// // ==========================================================
// // ==== PID 闭环控制 (Lab 4.2.2) ====
// // ==========================================================
// float KP = 3.0; 
// float KI = 0.5; 
// float KD = 0.1; 

// float summederror = 0;
// float last_error = 0;

// const long PID_SAMPLE_TIME_MS = 50; 
// unsigned long last_pid_time = 0;

// int DesiredSpeed_Setpoint = 0; 
// float MeasuredVelocity_CountsPerSec = 0; 

// // ==========================================================
// // ==== 实时数据显示的全局变量 ====
// // ==========================================================
// float g_current_error = 0;
// int   g_current_pwm_output = 0;

// // ==========================================================
// // ==== Encoder / Counter Variables (Lab 4.1.5) ====
// // ==========================================================
// volatile long EncoderCounts = 0; 
// volatile long last_counts = 0;   
// volatile int motorDirection = 0; 

// // ==========================================================
// // ==== Interrupt Service Routine (ISR) ====
// // ==========================================================
// void IRAM_ATTR isr_encoder() {
//   EncoderCounts += motorDirection;
// }

// // ==========================================================
// // ==== Web Server HTML and Control Functions ====
// // ==========================================================
// WebServer server(80); 

// void handleRoot() {
//   // --- HTML (已更新, 包含 PID 调参表单 和 实时数据 div) ---
//   String html = "<!DOCTYPE html><html><head><title>ESP32 PID Control</title>";
//   html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
//   html += "<style>body{font-family:Arial; text-align:center; background-color:#222; color:white;}";
//   html += ".btn{display:block; width:80%; margin:10px auto; padding:20px; font-size:24px; color:black; background-color:#00c2ff; text-decoration:none; border-radius:10px; cursor:pointer;}"; 
//   html += ".stop{background-color:#ff4f4f;}";
//   html += ".slider{width:80%; margin:20px auto;}";
//   html += "input[type='number']{width: 80px; font-size: 16px; margin: 5px;}";
//   html += "form{margin: 20px auto; width: 80%; padding: 10px; border: 1px solid #555; border-radius: 10px;}";
//   html += "input[type='submit']{background-color:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:5px; font-size:16px; cursor:pointer;}";
//   html += "#data-container{width:80%; margin:20px auto; padding:10px; border:1px solid #555; border-radius:10px; text-align:left; font-size:18px; line-height:1.6;}";
//   html += "#data-container span{float:right; font-weight:bold; color:#00c2ff;}";
//   // --- SVG 图表样式 ---
//   html += "svg{width:80%; height:120px; background-color:#333; border:1px solid #555; border-radius:10px; margin-top:10px;}";
//   html += "</style>";
//   html += "</head><body>";
//   html += "<h1>PID Motor Control (Lab 4.2.2)</h1>";
  
//   // ==========================================================
//   // ==== 1. 实时 SVG 图表 ====
//   // ==========================================================
//   html += "<h3>Live Chart (Last 5 Sec)</h3>";
//   html += "<svg id='pid-chart' viewBox='0 0 300 100' preserveAspectRatio='none'>";
//   html += "<line id='graph-target' x1='0' y1='50' x2='300' y2='50' style='stroke:#00c2ff; stroke-width:2; stroke-dasharray: 4;' />";
//   html += "<polyline id='graph-measured' points='0,100' style='fill:none; stroke:#4CAF50; stroke-width:2;' />";
//   html += "</svg>";
  
//   // ==========================================================
//   // ==== 2. 实时数据文本 ====
//   // ==========================================================
//   html += "<h3>Live Data</h3>";
//   html += "<div id='data-container'>";
//   html += "Target: <span id='target-val'>0</span> C/s<br>";
//   html += "Measured: <span id='measured-val'>0.0</span> C/s<br>";
//   html += "Error: <span id='error-val'>0.0</span><br>";
//   html += "PWM: <span id='pwm-val'>0</span>";
//   html += "</div>";

//   // --- 方向控制 ---
//   html += "<h3>Direction</h3>";
//   html += "<button class='btn' onclick=\"sendCommand('forward')\">FORWARD</button>";
//   html += "<button class='btn' onclick=\"sendCommand('backward')\">BACKWARD</button>";
//   html += "<button class='btn stop' onclick=\"sendCommand('stop')\">STOP</button>";
  
//   // --- 速度控制 ---
//   html += "<h3 id='speed-label'>Desired Speed (" + String(DesiredSpeed_Setpoint) + " C/s)</h3>";
//   html += "<input type='range' min='0' max='86' value='" + String(DesiredSpeed_Setpoint) + "' class='slider' id='speedSlider'";
//   html += " onchange='setSpeed(this.value)'>";
  
//   // --- PID 调参表单 ---
//   html += "<h3>PID Tuning</h3>";
//   html += "<form onsubmit='updatePID(event)'>";
//   // ==========================================================
//   // ==== 最终修复: 分辨率已设置为 0.01 ====
//   // ==========================================================
//   html += "KP: <input type='number' step='0.1' id='kp' value='" + String(KP) + "'><br>"; // KP 保持 0.1
//   html += "KI: <input type='number' step='0.01' id='ki' value='" + String(KI) + "'><br>";
//   html += "KD: <input type='number' step='0.01' id='kd' value='" + String(KD) + "'><br>";
//   html += "<br><input type='submit' value='Update PID'>";
//   html += "</form>";

//   // --- JavaScript (包含数据轮询) ---
//   html += "<script>";
//   // --- JS 变量用于图表 ---
//   html += "const MAX_HISTORY = 50;"; 
//   html += "let historyMeasured = [];";
//   html += "const CHART_WIDTH = 300;"; 
//   html += "const CHART_HEIGHT = 100;"; 
//   html += "const MAX_SPEED = 86.0;"; 
  
//   html += "function sendCommand(cmd){ var xhr = new XMLHttpRequest(); xhr.open('GET', '/' + cmd, true); xhr.send();}";
//   html += "function setSpeed(val){ var xhr = new XMLHttpRequest(); xhr.open('GET', '/setSpeed?value=' + val, true); xhr.send(); document.getElementById('speed-label').innerHTML = 'Desired Speed (' + val + ' C/s)';}";
  
//   html += "function updatePID(e){";
//   html += "e.preventDefault();"; 
//   html += "var kp=document.getElementById('kp').value;";
//   html += "var ki=document.getElementById('ki').value;";
//   html += "var kd=document.getElementById('kd').value;";
//   html += "var xhr=new XMLHttpRequest();";
//   html += "xhr.open('GET', '/setPID?kp='+kp+'&ki='+ki+'&kd='+kd, true);";
//   html += "xhr.send();";
//   html += "}"; 
  
//   // --- getData() JavaScript 函数和定时器 ---
//   html += "function getData(){";
//   html += "var xhr=new XMLHttpRequest();";
//   html += "xhr.onreadystatechange = function() {";
//   html += "if (this.readyState == 4 && this.status == 200) {";
//   html += "var data = JSON.parse(this.responseText);"; 
//   // 1. 更新文本数据
//   html += "document.getElementById('target-val').innerHTML = data.target;";
//   html += "document.getElementById('measured-val').innerHTML = data.measured.toFixed(1);"; 
//   html += "document.getElementById('error-val').innerHTML = data.error.toFixed(1);"; 
//   html += "document.getElementById('pwm-val').innerHTML = data.pwm;";
//   // 2. 更新图表
//   html += "updateGraph(data.measured, data.target);";
//   html += "}";
//   html += "};";
//   html += "xhr.open('GET', '/getData', true);";
//   html += "xhr.send();";
//   html += "}";
  
//   // --- updateGraph() JavaScript 函数 ---
//   html += "function updateGraph(measuredVal, targetVal) {";
//   html += "historyMeasured.push(measuredVal);";
//   html += "if (historyMeasured.length > MAX_HISTORY) { historyMeasured.shift(); }";
  
//   // --- 绘制 Target (蓝色虚线) ---
//   html += "var y_target = CHART_HEIGHT - (targetVal / MAX_SPEED) * CHART_HEIGHT;";
//   html += "if (y_target < 0) y_target = 0; if (y_target > CHART_HEIGHT) y_target = CHART_HEIGHT;";
//   html += "var targetLine = document.getElementById('graph-target');";
//   html += "targetLine.setAttribute('y1', y_target);";
//   html += "targetLine.setAttribute('y2', y_target);";
  
//   // --- 绘制 Measured (绿色折线) ---
//   html += "var points = '';"; 
//   html += "for (var i = 0; i < historyMeasured.length; i++) {";
//   html += "var x = (i / (MAX_HISTORY - 1)) * CHART_WIDTH;";
//   html += "var y = CHART_HEIGHT - (historyMeasured[i] / MAX_SPEED) * CHART_HEIGHT;";
//   html += "if (y < 0) y = 0; if (y > CHART_HEIGHT) y = CHART_HEIGHT;";
//   html += "points += x + ',' + y + ' ';"; 
//   html += "}";
//   html += "document.getElementById('graph-measured').setAttribute('points', points);";
//   html += "}"; 
  
//   // 刷新间隔已设置为 100ms
//   html += "setInterval(getData, 100);"; 
//   html += "</script>";
  
//   html += "</body></html>";
//   server.send(200, "text/html", html); 
// }

// // --- Control functions (MODIFIED FOR PID) ---
// void handleForward() {
//   digitalWrite(DIR_PIN_1, LOW);
//   digitalWrite(DIR_PIN_2, HIGH);
//   motorDirection = 1; 
//   summederror = 0; 
//   last_error = 0;
//   server.send(200, "text/plain", "OK - Forward");
// }

// void handleBackward() {
//   digitalWrite(DIR_PIN_1, HIGH);
//   digitalWrite(DIR_PIN_2, LOW);
//   motorDirection = -1; 
//   summederror = 0; 
//   last_error = 0;
//   server.send(200, "text/plain", "OK - Backward");
// }

// void handleStop() {
//   motorDirection = 0;
//   DesiredSpeed_Setpoint = 0;
//   summederror = 0; 
//   last_error = 0;
//   server.send(200, "text/plain", "OK - Stopped");
// }

// void handleSetSpeed() {
//   String speedStr = server.arg("value");
//   if (speedStr != "") {
//     DesiredSpeed_Setpoint = speedStr.toInt();
//   }
//   server.send(200, "text/plain", "OK - Speed set to " + String(DesiredSpeed_Setpoint));
// }

// // ==========================================================
// // ==== PID 调参处理器 ====
// // ==========================================================
// void handleSetPID() {
//   if (server.hasArg("kp")) {
//     KP = server.arg("kp").toFloat(); 
//   }
//   if (server.hasArg("ki")) {
//     KI = server.arg("ki").toFloat();
//   }
//   if (server.hasArg("kd")) {
//     KD = server.arg("kd").toFloat();
//   }
//   summederror = 0;
//   last_error = 0;
  
//   Serial.printf("PID Gains Updated: KP=%.2f, KI=%.2f, KD=%.2f\n", KP, KI, KD);
//   server.send(200, "text/plain", "OK - PID Updated");
// }

// // ==========================================================
// // ==== 实时数据发送器 (JSON) ====
// // ==========================================================
// void handleGetData() {
//   // 创建一个 JSON 字符串
//   String json = "{";
//   json += "\"target\": " + String(DesiredSpeed_Setpoint);
//   json += ", \"measured\": " + String(MeasuredVelocity_CountsPerSec, 1); // 1位小数
//   json += ", \"error\": " + String(g_current_error, 1); // 1位小数
//   json += ", \"pwm\": " + String(g_current_pwm_output);
//   json += "}";
  
//   // 发送 JSON 响应
//   server.send(200, "application/json", json);
// }


// // ==========================================================
// // ==== SETUP & LOOP (All Functions Included) ====
// // ==========================================================
// void setup() {
//   Serial.begin(115200);

//   pinMode(DIR_PIN_1, OUTPUT);
//   pinMode(DIR_PIN_2, OUTPUT);
//   pinMode(SPEED_PIN, OUTPUT);
//   ledcAttach(SPEED_PIN, PWM_FREQ, PWM_BITS);
  
//   pinMode(ENCODER_PIN, INPUT_PULLUP); 
//   // 修复 1: 将中断模式更改为 CHANGE
//   attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), isr_encoder, CHANGE);
  
//   // Default stop
//   digitalWrite(DIR_PIN_1, LOW);
//   digitalWrite(DIR_PIN_2, LOW);
//   ledcWrite(SPEED_PIN, 0);

//   // Start AP mode
//   WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
//   WiFi.softAP(AP_SSID, AP_PASS);
//   Serial.print("\nAP IP address: ");
//   Serial.println(WiFi.softAPIP());

//   // --- 注册所有的服务器端点 ---
//   server.on("/", handleRoot);           
//   server.on("/forward", handleForward); 
//   server.on("/backward", handleBackward);
//   server.on("/stop", handleStop);
//   server.on("/setSpeed", handleSetSpeed);
//   server.on("/setPID", handleSetPID);     
//   server.on("/getData", handleGetData); 
  
//   server.begin(); // 服务器启动
//   Serial.println("Web server started! Connect to AP and visit http://192.168.4.1");
  
//   last_pid_time = millis();
// }

// // ================================================================
// // ==== 修复 2: 完整的 LOOP 函数已更新为使用 counts/second ====
// // ================================================================
// void loop() {
//   server.handleClient(); // 处理网页请求

//   // ============================================
//   // ==== PID 闭环控制循环 (Lab 4.2.2) ====
//   // ============================================
  
//   if (millis() - last_pid_time < PID_SAMPLE_TIME_MS) {
//     return; // 还没到 50ms, 退出 loop
//   }
  
//   // --- 1. 测量 (获取编码器数据) ---
//   noInterrupts(); // 进入临界区
//   long current_counts = EncoderCounts;
//   long counts_delta = current_counts - last_counts; 
//   last_counts = current_counts; 
//   interrupts(); // 退出临界区

//   // --- 单位修复 (从 counts/sample 转换为 counts/second) ---
//   MeasuredVelocity_CountsPerSec = (float)counts_delta * (1000.0 / PID_SAMPLE_TIME_MS);
  
//   int pwm_output = 0;

//   // --- 2. 控制 (如果电机停止，则跳过 PID) ---
//   if (motorDirection == 0) {
//     pwm_output = 0;
//     summederror = 0;
//     last_error = 0;
//     g_current_error = 0; // 更新全局变量
//   } else {
//     // --- 3. PID 计算 (现在以 counts/second 为单位) ---
    
//     // P-Term: 比例
//     float error = DesiredSpeed_Setpoint - MeasuredVelocity_CountsPerSec;
//     g_current_error = error; // 更新全局变量以供网页显示

//     // I-Term: 积分 (带抗饱和)
//     summederror += error;
    
//     float LARGE_SUM = 2000.0; 
//     if (summederror > LARGE_SUM) summederror = LARGE_SUM;
//     if (summederror < -LARGE_SUM) summederror = -LARGE_SUM;

//     // D-Term: 微分
//     float d_term_error = error - last_error;
//     last_error = error;

//     // --- 最终 PID 输出 ---
//     float pid_output_float = (KP * error) + (KI * summederror) + (KD * d_term_error);

//     // --- 4. 执行 (PWM 范围限制) ---
//     if (pid_output_float > 255) pid_output_float = 255;
//     if (pid_output_float < 0) pid_output_float = 0;
    
//     pwm_output = (int)pid_output_float;
//   }
  
//   // --- 5. 应用最终的 PWM 值 ---
//   g_current_pwm_output = pwm_output; // 更新全局变量以供网页显示
//   ledcWrite(SPEED_PIN, g_current_pwm_output);

//   // 打印调试信息 (单位已更新)
//   Serial.printf("Target: %d C/s, Measured: %.1f C/s, Error: %.1f, PWM: %d\n", 
//                 DesiredSpeed_Setpoint, 
//                 MeasuredVelocity_CountsPerSec, 
//                 g_current_error,
//                 g_current_pwm_output);
                
//   last_pid_time = millis(); // 在循环 *结束* 时重置 PID 计时器
// }

// ESP32-C3 Dual Motor PID (No interrupts) — PCNT-based encoder counting
// Lab 4.2.2 minimal full sketch

#include <WiFi.h>
#include <WebServer.h>
#include <esp32-hal-ledc.h>
#include <esp32-hal-adc.h>
#include "driver/pcnt.h"   // 使用硬件脉冲计数器

// ===================== Wi-Fi (AP) =====================
const char* AP_SSID = "ESP32_PID_CHART_DUAL";
const char* AP_PASS = "123456789";
IPAddress AP_IP(192,168,4,1);

// ===================== Pins & PWM =====================
// 左电机
const int SPEED_PIN_L   = 6;   // EN (PWM)
const int DIR_PIN_1     = 10;  // 1A
const int DIR_PIN_2     = 8;   // 2A
const int ENCODER_PIN_L = 1;   // 编码器输入（单通道上升沿计数）
// 右电机
const int SPEED_PIN_R   = 7;   // EN (PWM)
const int DIR_PIN_3     = 4;   // 4A
const int DIR_PIN_4     = 5;   // 3A
const int ENCODER_PIN_R = 0;   // 编码器输入（单通道上升沿计数）

const int PWM_FREQ = 250;
const int PWM_BITS = 8;

// ===================== PID =====================
float KP_L = 3.0, KI_L = 0.5, KD_L = 0.1;
float KP_R = 3.0, KI_R = 0.5, KD_R = 0.1;

float summederror_L = 0, last_error_L = 0;
float summederror_R = 0, last_error_R = 0;

const long PID_SAMPLE_TIME_MS = 50;
unsigned long last_pid_time = 0;

int   DesiredSpeed_Setpoint = 0;
float MeasuredVelocity_CountsPerSec_L = 0;
float MeasuredVelocity_CountsPerSec_R = 0;

// ===================== Runtime/State =====================
float g_current_error_L = 0;
int   g_current_pwm_output_L = 0;
float g_current_error_R = 0;
int   g_current_pwm_output_R = 0;

volatile int motorDirection_L = 0; // 1/-1/0 由方向按键决定
volatile int motorDirection_R = 0;

// ===================== PCNT =====================
// 使用两个单元分别计数左右编码器的上升沿
static pcnt_unit_t PCNT_UNIT_L = PCNT_UNIT_0;
static pcnt_unit_t PCNT_UNIT_R = PCNT_UNIT_1;

int16_t pcnt_last_L = 0;
int16_t pcnt_last_R = 0;

static void pcnt_init_one(pcnt_unit_t unit, gpio_num_t pin)
{
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = pin;                 // 脉冲输入脚
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;   // 不用方向控制脚
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = unit;
  // 仅在上升沿计数（下降沿不计数）
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;
  // 高/低电平控制都不改变计数方向
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  // 计数上下限（int16_t），保证不会触发溢出中断
  cfg.counter_h_lim = 30000;
  cfg.counter_l_lim = -30000;

  pcnt_unit_config(&cfg);

  // 启用硬件去抖/毛刺滤波（忽略过窄脉冲）
  // 滤波值单位为 APB 时钟周期（80MHz），例如 1000 ≈ 12.5us
  pcnt_set_filter_value(unit, 1000);
  pcnt_filter_enable(unit);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

static inline int16_t pcnt_read_and_get_delta(pcnt_unit_t unit, int16_t &last)
{
  int16_t now = 0;
  pcnt_get_counter_value(unit, &now);
  int16_t delta = now - last;
  last = now;
  return delta;
}

// ===================== Web =====================
WebServer server(80);

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Dual PID Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial; text-align:center; background:#222; color:#fff}";
  html += ".btn{display:block; width:80%; margin:10px auto; padding:20px; font-size:24px; color:#000; background:#00c2ff; border-radius:10px; text-decoration:none}";
  html += ".stop{background:#ff4f4f}.slider{width:80%; margin:20px auto}";
  html += "input[type=number]{width:80px; font-size:16px; margin:5px}";
  html += "form{margin:20px auto; width:90%; padding:10px; border:1px solid #555; border-radius:10px}";
  html += "form div{display:inline-block; width:48%; vertical-align:top; box-sizing:border-box}";
  html += "input[type=submit]{background:#4CAF50; color:#fff; padding:10px 20px; border:none; border-radius:5px; font-size:16px; margin-top:10px}";
  html += "#data-container{width:80%; margin:20px auto; padding:10px; border:1px solid #555; border-radius:10px; text-align:left; font-size:18px; line-height:1.6}";
  html += "#data-container span{float:right; font-weight:bold; color:#00c2ff}";
  html += "svg{width:80%; height:120px; background:#333; border:1px solid #555; border-radius:10px; margin-top:10px}";
  html += "</style></head><body>";

  html += "<h1>Dual PID Motor Control (PCNT, no interrupts)</h1>";
  html += "<h3>Live Chart (Last 5 Sec)</h3>";
  html += "<svg id='pid-chart' viewBox='0 0 300 100' preserveAspectRatio='none'>";
  html += "<line id='graph-target' x1='0' y1='50' x2='300' y2='50' style='stroke:#00c2ff;stroke-width:2;stroke-dasharray:4'/>";
  html += "<polyline id='graph-measured-l' points='0,100' style='fill:none;stroke:#4CAF50;stroke-width:2'/>";
  html += "<polyline id='graph-measured-r' points='0,100' style='fill:none;stroke:#FF6347;stroke-width:2'/>";
  html += "</svg>";

  html += "<h3>Live Data</h3><div id='data-container'>";
  html += "Target: <span id='target-val'>0</span> C/s<br><hr style='border-color:#444'>";
  html += "Measured (L): <span id='measured-l-val'>0.0</span> C/s<br>";
  html += "Error (L): <span id='error-l-val'>0.0</span><br>";
  html += "PWM (L): <span id='pwm-l-val'>0</span><br><hr style='border-color:#444'>";
  html += "Measured (R): <span id='measured-r-val'>0.0</span> C/s<br>";
  html += "Error (R): <span id='error-r-val'>0.0</span><br>";
  html += "PWM (R): <span id='pwm-r-val'>0</span></div>";

  html += "<h3>Direction</h3>";
  html += "<button class='btn' onclick=\"sendCommand('forward')\">FORWARD</button>";
  html += "<button class='btn' onclick=\"sendCommand('backward')\">BACKWARD</button>";
  html += "<button class='btn stop' onclick=\"sendCommand('stop')\">STOP</button>";

  html += "<h3 id='speed-label'>Desired Speed (0 C/s)</h3>";
  html += "<input type='range' min='0' max='86' value='0' class='slider' id='speedSlider' onchange='setSpeed(this.value)'>";

  html += "<h3>PID Tuning (Independent)</h3>";
  html += "<form onsubmit='updatePID(event)'>";
  html += "<div><b>Left Motor</b><br>";
  html += "KP (L): <input type='number' step='0.1' id='kp_l' value='3.0'><br>";
  html += "KI (L): <input type='number' step='0.01' id='ki_l' value='0.5'><br>";
  html += "KD (L): <input type='number' step='0.01' id='kd_l' value='0.1'><br></div>";
  html += "<div><b>Right Motor</b><br>";
  html += "KP (R): <input type='number' step='0.1' id='kp_r' value='3.0'><br>";
  html += "KI (R): <input type='number' step='0.01' id='ki_r' value='0.5'><br>";
  html += "KD (R): <input type='number' step='0.01' id='kd_r' value='0.1'><br></div>";
  html += "<br><br><input type='submit' value='Update All PID'></form>";

  html += "<script>";
  html += "const MAX_HISTORY=50, CHART_WIDTH=300, CHART_HEIGHT=100, MAX_SPEED=86.0;";
  html += "let histL=[], histR=[];";
  html += "function sendCommand(c){var x=new XMLHttpRequest();x.open('GET','/'+c,true);x.send();}";
  html += "function setSpeed(v){var x=new XMLHttpRequest();x.open('GET','/setSpeed?value='+v,true);x.send();document.getElementById('speed-label').innerHTML='Desired Speed ('+v+' C/s)';}";
  html += "function updatePID(e){e.preventDefault();var kp_l=kp_l.value,ki_l=ki_l.value,kd_l=kd_l.value,kp_r=kp_r.value,ki_r=ki_r.value,kd_r=kd_r.value;var x=new XMLHttpRequest();x.open('GET','/setPID?kp_l='+kp_l+'&ki_l='+ki_l+'&kd_l='+kd_l+'&kp_r='+kp_r+'&ki_r='+ki_r+'&kd_r='+kd_r,true);x.send();}";
  html += "function getData(){var x=new XMLHttpRequest();x.onreadystatechange=function(){if(this.readyState==4&&this.status==200){var d=JSON.parse(this.responseText);";
  html += "document.getElementById('target-val').innerHTML=d.target;";
  html += "document.getElementById('measured-l-val').innerHTML=d.measured_l.toFixed(1);";
  html += "document.getElementById('error-l-val').innerHTML=d.error_l.toFixed(1);";
  html += "document.getElementById('pwm-l-val').innerHTML=d.pwm_l;";
  html += "document.getElementById('measured-r-val').innerHTML=d.measured_r.toFixed(1);";
  html += "document.getElementById('error-r-val').innerHTML=d.error_r.toFixed(1);";
  html += "document.getElementById('pwm-r-val').innerHTML=d.pwm_r;updateGraph(d.measured_l,d.measured_r,d.target);}};x.open('GET','/getData',true);x.send();}";
  html += "function updateGraph(vL,vR,t){histL.push(vL);histR.push(vR);if(histL.length>MAX_HISTORY)histL.shift();if(histR.length>MAX_HISTORY)histR.shift();";
  html += "var yt=CHART_HEIGHT-(t/MAX_SPEED)*CHART_HEIGHT;yt=Math.max(0,Math.min(CHART_HEIGHT,yt));";
  html += "document.getElementById('graph-target').setAttribute('y1',yt);document.getElementById('graph-target').setAttribute('y2',yt);";
  html += "var pL='',pR='';for(var i=0;i<histL.length;i++){var x=(i/(MAX_HISTORY-1))*CHART_WIDTH;var yL=CHART_HEIGHT-(histL[i]/MAX_SPEED)*CHART_HEIGHT;yL=Math.max(0,Math.min(CHART_HEIGHT,yL));pL+=x+','+yL+' ';if(i<histR.length){var yR=CHART_HEIGHT-(histR[i]/MAX_SPEED)*CHART_HEIGHT;yR=Math.max(0,Math.min(CHART_HEIGHT,yR));pR+=x+','+yR+' ';}}";
  html += "document.getElementById('graph-measured-l').setAttribute('points',pL);document.getElementById('graph-measured-r').setAttribute('points',pR);}setInterval(getData,100);";
  html += "</script></body></html>";
  server.send(200,"text/html",html);
}

void handleForward() {
  // L
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, HIGH);
  motorDirection_L = 1;
  summederror_L = last_error_L = 0;
  // R
  digitalWrite(DIR_PIN_3, LOW);
  digitalWrite(DIR_PIN_4, HIGH);
  motorDirection_R = 1;
  summederror_R = last_error_R = 0;
  server.send(200,"text/plain","OK - Forward");
}
void handleBackward() {
  // L
  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(DIR_PIN_2, LOW);
  motorDirection_L = -1;
  summederror_L = last_error_L = 0;
  // R
  digitalWrite(DIR_PIN_3, HIGH);
  digitalWrite(DIR_PIN_4, LOW);
  motorDirection_R = -1;
  summederror_R = last_error_R = 0;
  server.send(200,"text/plain","OK - Backward");
}
void handleStop() {
  motorDirection_L = 0; summederror_L = last_error_L = 0;
  motorDirection_R = 0; summederror_R = last_error_R = 0;
  DesiredSpeed_Setpoint = 0;
  server.send(200,"text/plain","OK - Stopped");
}
void handleSetSpeed() {
  String s = server.arg("value");
  if (s!="") DesiredSpeed_Setpoint = s.toInt();
  server.send(200,"text/plain","OK - Speed set to " + String(DesiredSpeed_Setpoint));
}
void handleSetPID() {
  if (server.hasArg("kp_l")) KP_L = server.arg("kp_l").toFloat();
  if (server.hasArg("ki_l")) KI_L = server.arg("ki_l").toFloat();
  if (server.hasArg("kd_l")) KD_L = server.arg("kd_l").toFloat();
  if (server.hasArg("kp_r")) KP_R = server.arg("kp_r").toFloat();
  if (server.hasArg("ki_r")) KI_R = server.arg("ki_r").toFloat();
  if (server.hasArg("kd_r")) KD_R = server.arg("kd_r").toFloat();
  summederror_L = last_error_L = 0;
  summederror_R = last_error_R = 0;
  server.send(200,"text/plain","OK - PID Updated");
}
void handleGetData() {
  String json = "{";
  json += "\"target\":"   + String(DesiredSpeed_Setpoint);
  json += ",\"measured_l\":" + String(MeasuredVelocity_CountsPerSec_L,1);
  json += ",\"error_l\":"    + String(g_current_error_L,1);
  json += ",\"pwm_l\":"      + String(g_current_pwm_output_L);
  json += ",\"measured_r\":" + String(MeasuredVelocity_CountsPerSec_R,1);
  json += ",\"error_r\":"    + String(g_current_error_R,1);
  json += ",\"pwm_r\":"      + String(g_current_pwm_output_R);
  json += "}";
  server.send(200,"application/json",json);
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);

  // 电机 IO
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(SPEED_PIN_L, OUTPUT);

  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(DIR_PIN_4, OUTPUT);
  pinMode(SPEED_PIN_R, OUTPUT);

  // PWM
  ledcAttach(SPEED_PIN_L, PWM_FREQ, PWM_BITS);
  ledcAttach(SPEED_PIN_R, PWM_FREQ, PWM_BITS);

  // 编码器输入脚上拉（建议硬件再加 10k 上拉与 RC）
  pinMode(ENCODER_PIN_L, INPUT_PULLUP);
  pinMode(ENCODER_PIN_R, INPUT_PULLUP);

  // 初始化 PCNT 两路
  pcnt_init_one(PCNT_UNIT_L, (gpio_num_t)ENCODER_PIN_L);
  pcnt_init_one(PCNT_UNIT_R, (gpio_num_t)ENCODER_PIN_R);

  // 停止默认
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, LOW);
  ledcWrite(SPEED_PIN_L, 0);

  digitalWrite(DIR_PIN_3, LOW);
  digitalWrite(DIR_PIN_4, LOW);
  ledcWrite(SPEED_PIN_R, 0);

  // Wi-Fi AP
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("\nAP IP address: "); Serial.println(WiFi.softAPIP());

  // Web handlers
  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/stop", handleStop);
  server.on("/setSpeed", handleSetSpeed);
  server.on("/setPID", handleSetPID);
  server.on("/getData", handleGetData);
  server.begin();
  Serial.println("Web server started! http://192.168.4.1");

  last_pid_time = millis();
}

// ===================== Loop =====================
void loop() {
  server.handleClient();

  if (millis() - last_pid_time < PID_SAMPLE_TIME_MS) return;

  // ---- 1) 读 PCNT 计数差值（50ms）并换算 counts/s ----
  int16_t dL = pcnt_read_and_get_delta(PCNT_UNIT_L, pcnt_last_L);
  int16_t dR = pcnt_read_and_get_delta(PCNT_UNIT_R, pcnt_last_R);

  // 方向符号来自当前指令；只要编码器是单通道，上升沿数目即速度大小
  float cpsL = (float)dL * (1000.0f / PID_SAMPLE_TIME_MS);
  float cpsR = (float)dR * (1000.0f / PID_SAMPLE_TIME_MS);
  MeasuredVelocity_CountsPerSec_L = (motorDirection_L == 0) ? 0 : (motorDirection_L > 0 ? cpsL : cpsL); // 单通道仅有幅值
  MeasuredVelocity_CountsPerSec_R = (motorDirection_R == 0) ? 0 : (motorDirection_R > 0 ? cpsR : cpsR);

  // ---- 2) PID 双路 ----
  const float LARGE_SUM = 2000.0f;

  int pwmL = 0, pwmR = 0;

  if (motorDirection_L == 0) {
    pwmL = 0; summederror_L = 0; last_error_L = 0; g_current_error_L = 0;
  } else {
    float errL = DesiredSpeed_Setpoint - MeasuredVelocity_CountsPerSec_L;
    g_current_error_L = errL;
    summederror_L += errL;
    if (summederror_L >  LARGE_SUM) summederror_L =  LARGE_SUM;
    if (summederror_L < -LARGE_SUM) summederror_L = -LARGE_SUM;
    float dL_err = errL - last_error_L; last_error_L = errL;
    float uL = KP_L*errL + KI_L*summederror_L + KD_L*dL_err;
    if (uL < 0) uL = 0; if (uL > 255) uL = 255;
    pwmL = (int)uL;
  }

  if (motorDirection_R == 0) {
    pwmR = 0; summederror_R = 0; last_error_R = 0; g_current_error_R = 0;
  } else {
    float errR = DesiredSpeed_Setpoint - MeasuredVelocity_CountsPerSec_R;
    g_current_error_R = errR;
    summederror_R += errR;
    if (summederror_R >  LARGE_SUM) summederror_R =  LARGE_SUM;
    if (summederror_R < -LARGE_SUM) summederror_R = -LARGE_SUM;
    float dR_err = errR - last_error_R; last_error_R = errR;
    float uR = KP_R*errR + KI_R*summederror_R + KD_R*dR_err;
    if (uR < 0) uR = 0; if (uR > 255) uR = 255;
    pwmR = (int)uR;
  }

  // ---- 3) 输出 PWM ----
  g_current_pwm_output_L = pwmL;
  g_current_pwm_output_R = pwmR;
  ledcWrite(SPEED_PIN_L, g_current_pwm_output_L);
  ledcWrite(SPEED_PIN_R, g_current_pwm_output_R);

  // 调试
  Serial.printf("L: T:%d M:%.1f E:%.1f P:%d | R: T:%d M:%.1f E:%.1f P:%d\n",
    DesiredSpeed_Setpoint, MeasuredVelocity_CountsPerSec_L, g_current_error_L, g_current_pwm_output_L,
    DesiredSpeed_Setpoint, MeasuredVelocity_CountsPerSec_R, g_current_error_R, g_current_pwm_output_R);

  last_pid_time = millis();
}
