#include <WiFi.h>
#include <WebServer.h> // Library for Web Server
#include <esp32-hal-ledc.h>
#include <esp32-hal-adc.h>

// ==========================================================
// ==== Wi-Fi (AP Mode Configuration)
// ==========================================================
const char* AP_SSID = "ESP32_PID_CHART_DUAL"; 
const char* AP_PASS = "123456789"; 
IPAddress AP_IP(192, 168, 4, 1);
// ==========================================================
// ==== Hardware Pins & PWM Parameters (L298N) ====
// ==========================================================
// --- Motor 1 (Left / A 通道 / G1) ---
const int SPEED_PIN_L = 18;   // ENA (G18)
const int DIR_PIN_1 = 19;   // IN1 (G19)
const int DIR_PIN_2 = 20;   // IN2 (G20)
const int ENCODER_PIN_L = 1;  // G1

// --- Motor 2 (Right / B 通道 / G0) ---
const int SPEED_PIN_R = 7;    // ENB (G7)
const int DIR_PIN_3 = 4;    // IN3 (G4)
const int DIR_PIN_4 = 5;    // IN4 (G5)
const int ENCODER_PIN_R = 0;  // G0 

// --- PWM Configuration ---
const int PWM_FREQ = 250;     // PWM Frequency (Hz)
const int PWM_BITS = 8;       // Resolution (8-bit = 0-255 range)

// ==========================================================
// ==== PID 闭环控制 (Lab 4.2.2) ====
// ==========================================================
// --- PID 增益 (L/R) --- (两套独立增益)
float KP_L = 3.0; 
float KI_L = 0.5; 
float KD_L = 0.1; 
float KP_R = 3.0; 
float KI_R = 0.5; 
float KD_R = 0.1; 

// --- PID 状态变量 (L/R) ---
float summederror_L = 0;
float last_error_L = 0;
float summederror_R = 0;
float last_error_R = 0;

const long PID_SAMPLE_TIME_MS = 50; 
unsigned long last_pid_time = 0;

// --- 速度和设定值变量 ---
int DesiredSpeed_Setpoint = 0; // 共享的目标速度
float MeasuredVelocity_CountsPerSec_L = 0; 
float MeasuredVelocity_CountsPerSec_R = 0; 

// ==========================================================
// ==== 实时数据显示的全局变量 (L/R) ====
// ==========================================================
float g_current_error_L = 0;
int   g_current_pwm_output_L = 0;
float g_current_error_R = 0;
int   g_current_pwm_output_R = 0;

// ==========================================================
// ==== Encoder / Counter Variables (L/R) ====
// ==========================================================
volatile long EncoderCounts_L = 0; 
volatile long last_counts_L = 0;   
volatile int motorDirection_L = 0; 
volatile long EncoderCounts_R = 0; 
volatile long last_counts_R = 0;   
volatile int motorDirection_R = 0; 

// ==========================================================
// ==== Interrupt Service Routines (L/R) ====
// ==========================================================
// (我们使用 attachInterrupt 方法)
void IRAM_ATTR isr_encoder_L() {
  EncoderCounts_L += motorDirection_L;
}
void IRAM_ATTR isr_encoder_R() {
  EncoderCounts_R += motorDirection_R;
}

// ==========================================================
// ==== Web Server HTML and Control Functions ====
// ==========================================================
WebServer server(80); 

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Dual PID Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial; text-align:center; background-color:#222; color:white;}";
  html += ".btn{display:block; width:80%; margin:10px auto; padding:20px; font-size:24px; color:black; background-color:#00c2ff; text-decoration:none; border-radius:10px; cursor:pointer;}"; 
  html += ".stop{background-color:#ff4f4f;}";
  html += ".slider{width:80%; margin:20px auto;}";
  html += "input[type='number']{width: 80px; font-size: 16px; margin: 5px;}";
  html += "form{margin: 20px auto; width: 90%; padding: 10px; border: 1px solid #555; border-radius: 10px;}";
  html += "form div{display:inline-block; width:48%; vertical-align:top; box-sizing: border-box;}";
  html += "input[type='submit']{background-color:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:5px; font-size:16px; cursor:pointer; margin-top:10px;}";
  html += "#data-container{width:80%; margin:20px auto; padding:10px; border:1px solid #555; border-radius:10px; text-align:left; font-size:18px; line-height:1.6;}";
  html += "#data-container span{float:right; font-weight:bold; color:#00c2ff;}";
  html += "svg{width:80%; height:120px; background-color:#333; border:1px solid #555; border-radius:10px; margin-top:10px;}";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>Dual PID Motor Control (L298N)</h1>"; 
  
  // --- 1. 实时 SVG 图表 ---
  html += "<h3>Live Chart (Last 5 Sec)</h3>";
  html += "<svg id='pid-chart' viewBox='0 0 300 100' preserveAspectRatio='none'>";
  html += "<line id='graph-target' x1='0' y1='50' x2='300' y2='50' style='stroke:#00c2ff; stroke-width:2; stroke-dasharray: 4;' />";
  html += "<polyline id='graph-measured-l' points='0,100' style='fill:none; stroke:#4CAF50; stroke-width:2;' />"; // 绿色 (Left)
  html += "<polyline id='graph-measured-r' points='0,100' style='fill:none; stroke:#FF6347; stroke-width:2;' />"; // 红色 (Right)
  html += "</svg>";
  
  // --- 2. 实时数据文本 ---
  html += "<h3>Live Data</h3>";
  html += "<div id='data-container'>";
  html += "Target: <span id='target-val'>0</span> C/s<br>";
  html += "<hr style='border-color:#444;'>";
  html += "Measured (L - G1): <span id='measured-l-val'>0.0</span> C/s<br>";
  html += "Error (L): <span id='error-l-val'>0.0</span><br>";
  html += "PWM (L): <span id='pwm-l-val'>0</span><br>";
  html += "<hr style='border-color:#444;'>";
  html += "Measured (R - G0): <span id='measured-r-val'>0.0</span> C/s<br>";
  html += "Error (R): <span id='error-r-val'>0.0</span><br>";
  html += "PWM (R): <span id='pwm-r-val'>0</span>";
  html += "</div>";

  // --- 方向控制 ---
  html += "<h3>Direction</h3>";
  html += "<button class='btn' onclick=\"sendCommand('forward')\">FORWARD</button>";
  html += "<button class='btn' onclick=\"sendCommand('backward')\">BACKWARD</button>";
  html += "<button class='btn stop' onclick=\"sendCommand('stop')\">STOP</button>";
  
  // --- 速度控制 (设置目标速度) ---
  html += "<h3 id='speed-label'>Desired Speed (" + String(DesiredSpeed_Setpoint) + " C/s)</h3>";
  
  // ==========================================================
  // ==== 最终修复: 滑块最大值已设置为 200 ====
  // ==========================================================
  html += "<input type='range' min='0' max='200' value='" + String(DesiredSpeed_Setpoint) + "' class='slider' id='speedSlider'";
  html += " onchange='setSpeed(this.value)'>";
  
  // --- PID 调参表单 ---
  html += "<h3>PID Tuning (Independent)</h3>";
  html += "<form onsubmit='updatePID(event)'>";
  html += "<div><b>Left Motor (G1)</b><br>"; // Left Column
  html += "KP (L): <input type='number' step='0.1' id='kp_l' value='" + String(KP_L) + "'><br>";
  html += "KI (L): <input type='number' step='0.01' id='ki_l' value='" + String(KI_L) + "'><br>";
  html += "KD (L): <input type='number' step='0.01' id='kd_l' value='" + String(KD_L) + "'><br>";
  html += "</div>";
  html += "<div><b>Right Motor (G0)</b><br>"; // Right Column
  html += "KP (R): <input type='number' step='0.1' id='kp_r' value='" + String(KP_R) + "'><br>";
  html += "KI (R): <input type='number' step='0.01' id='ki_r' value='" + String(KI_R) + "'><br>";
  html += "KD (R): <input type='number' step='0.01' id='kd_r' value='" + String(KD_R) + "'><br>";
  html += "</div>";
  html += "<br><br><input type='submit' value='Update All PID'>"; 
  html += "</form>";

  // --- JavaScript (包含数据轮询) ---
  html += "<script>";
  html += "const MAX_HISTORY = 50;"; 
  html += "let historyMeasured_L = [];";
  html += "let historyMeasured_R = [];";
  html += "const CHART_WIDTH = 300;"; 
  html += "const CHART_HEIGHT = 100;"; 
  
  // ==========================================================
  // ==== 最终修复: JS Y 轴最大值已设置为 200 ====
  // ==========================================================
  html += "const MAX_SPEED = 200.0;"; 
  
  html += "function sendCommand(cmd){ var xhr = new XMLHttpRequest(); xhr.open('GET', '/' + cmd, true); xhr.send();}";
  html += "function setSpeed(val){ var xhr = new XMLHttpRequest(); xhr.open('GET', '/setSpeed?value=' + val, true); xhr.send(); document.getElementById('speed-label').innerHTML = 'Desired Speed (' + val + ' C/s)';}";
  
  html += "function updatePID(e){";
  html += "e.preventDefault();"; 
  html += "var kp_l=document.getElementById('kp_l').value;";
  html += "var ki_l=document.getElementById('ki_l').value;";
  html += "var kd_l=document.getElementById('kd_l').value;";
  html += "var kp_r=document.getElementById('kp_r').value;";
  html += "var ki_r=document.getElementById('ki_r').value;";
  html += "var kd_r=document.getElementById('kd_r').value;";
  html += "var xhr=new XMLHttpRequest();";
  html += "xhr.open('GET', '/setPID?kp_l='+kp_l+'&ki_l='+ki_l+'&kd_l='+kd_l+'&kp_r='+kp_r+'&ki_r='+ki_r+'&kd_r='+kd_r, true);";
  html += "xhr.send();";
  html += "}"; 
  
  html += "function getData(){";
  html += "var xhr=new XMLHttpRequest();";
  html += "xhr.onreadystatechange = function() {";
  html += "if (this.readyState == 4 && this.status == 200) {";
  html += "var data = JSON.parse(this.responseText);"; 
  // 1. 更新文本数据 (L/R)
  html += "document.getElementById('target-val').innerHTML = data.target;";
  html += "document.getElementById('measured-l-val').innerHTML = data.measured_l.toFixed(1);"; 
  html += "document.getElementById('error-l-val').innerHTML = data.error_l.toFixed(1);"; 
  html += "document.getElementById('pwm-l-val').innerHTML = data.pwm_l;";
  html += "document.getElementById('measured-r-val').innerHTML = data.measured_r.toFixed(1);"; 
  html += "document.getElementById('error-r-val').innerHTML = data.error_r.toFixed(1);"; 
  html += "document.getElementById('pwm-r-val').innerHTML = data.pwm_r;";
  // 2. 更新图表
  html += "updateGraph(data.measured_l, data.measured_r, data.target);";
  html += "}";
  html += "};";
  html += "xhr.open('GET', '/getData', true);";
  html += "xhr.send();";
  html += "}";
  
  html += "function updateGraph(val_L, val_R, targetVal) {";
  html += "historyMeasured_L.push(val_L);";
  html += "historyMeasured_R.push(val_R);";
  html += "if (historyMeasured_L.length > MAX_HISTORY) { historyMeasured_L.shift(); }";
  html += "if (historyMeasured_R.length > MAX_HISTORY) { historyMeasured_R.shift(); }";
  
  html += "var y_target = CHART_HEIGHT - (targetVal / MAX_SPEED) * CHART_HEIGHT;";
  html += "if (y_target < 0) y_target = 0; if (y_target > CHART_HEIGHT) y_target = CHART_HEIGHT;";
  html += "var targetLine = document.getElementById('graph-target');";
  html += "targetLine.setAttribute('y1', y_target);";
  html += "targetLine.setAttribute('y2', y_target);";
  
  html += "var points_L = ''; var points_R = '';"; 
  html += "for (var i = 0; i < historyMeasured_L.length; i++) {";
  html += "var x = (i / (MAX_HISTORY - 1)) * CHART_WIDTH;";
  // L
  html += "var y_L = CHART_HEIGHT - (historyMeasured_L[i] / MAX_SPEED) * CHART_HEIGHT;";
  html += "if (y_L < 0) y_L = 0; if (y_L > CHART_HEIGHT) y_L = CHART_HEIGHT;";
  html += "points_L += x + ',' + y_L + ' ';"; 
  // R
  html += "if (i < historyMeasured_R.length) {";
  html += "var y_R = CHART_HEIGHT - (historyMeasured_R[i] / MAX_SPEED) * CHART_HEIGHT;";
  html += "if (y_R < 0) y_R = 0; if (y_R > CHART_HEIGHT) y_R = CHART_HEIGHT;";
  html += "points_R += x + ',' + y_R + ' ';"; 
  html += "}";
  html += "}";
  html += "document.getElementById('graph-measured-l').setAttribute('points', points_L);";
  html += "document.getElementById('graph-measured-r').setAttribute('points', points_R);";
  html += "}"; 
  
  html += "setInterval(getData, 100);"; 
  html += "</script>";
  
  html += "</body></html>";
  server.send(200, "text/html", html); 
}

// ==========================================================
// ==== 适配 L298N 的控制函数 ====
// ==========================================================

// --- Control functions (MODIFIED FOR L298N) ---
void handleForward() {
  // L298N: IN1=HIGH, IN2=LOW 为正转
  // Left Motor
  digitalWrite(DIR_PIN_1, HIGH); // IN1 (G19)
  digitalWrite(DIR_PIN_2, LOW);  // IN2 (G20)
  motorDirection_L = 1; 
  summederror_L = 0; 
  last_error_L = 0;
  
  // L298N: IN3=HIGH, IN4=LOW 为正转
  // Right Motor
  digitalWrite(DIR_PIN_3, HIGH); // IN3 (G4)
  digitalWrite(DIR_PIN_4, LOW);  // IN4 (G5)
  motorDirection_R = 1; 
  summederror_R = 0; 
  last_error_R = 0;
  
  server.send(200, "text/plain", "OK - Forward");
}

void handleBackward() {
  // L298N: IN1=LOW, IN2=HIGH 为反转
  // Left Motor
  digitalWrite(DIR_PIN_1, LOW);  // IN1 (G19)
  digitalWrite(DIR_PIN_2, HIGH); // IN2 (G20)
  motorDirection_L = -1; 
  summederror_L = 0; 
  last_error_L = 0;

  // L298N: IN3=LOW, IN4=HIGH 为反转
  // Right Motor
  digitalWrite(DIR_PIN_3, LOW);  // IN3 (G4)
  digitalWrite(DIR_PIN_4, HIGH); // IN4 (G5)
  motorDirection_R = -1; 
  summederror_R = 0; 
  last_error_R = 0;
  
  server.send(200, "text/plain", "OK - Backward");
}

void handleStop() {
  // L298N: IN1=LOW, IN2=LOW 为快速制动 (Fast Brake)
  // Left Motor
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, LOW);
  motorDirection_L = 0;
  summederror_L = 0; 
  last_error_L = 0;

  // L298N: IN3=LOW, IN4=LOW 为快速制动 (Fast Brake)
  // Right Motor
  digitalWrite(DIR_PIN_3, LOW);
  digitalWrite(DIR_PIN_4, LOW);
  motorDirection_R = 0;
  summederror_R = 0; 
  last_error_R = 0;
  
  // 共享
  DesiredSpeed_Setpoint = 0;
  
  server.send(200, "text/plain", "OK - Stopped");
}

void handleSetSpeed() {
  String speedStr = server.arg("value");
  if (speedStr != "") {
    DesiredSpeed_Setpoint = speedStr.toInt(); // 设置共享的目标速度
  }
  server.send(200, "text/plain", "OK - Speed set to " + String(DesiredSpeed_Setpoint));
}

// ==========================================================
// ==== PID 调参处理器 (L/R) ====
// ==========================================================
void handleSetPID() {
  // 解析 Left Gains
  if (server.hasArg("kp_l")) { KP_L = server.arg("kp_l").toFloat(); }
  if (server.hasArg("ki_l")) { KI_L = server.arg("ki_l").toFloat(); }
  if (server.hasArg("kd_l")) { KD_L = server.arg("kd_l").toFloat(); }
  // 解析 Right Gains
  if (server.hasArg("kp_r")) { KP_R = server.arg("kp_r").toFloat(); }
  if (server.hasArg("ki_r")) { KI_R = server.arg("ki_r").toFloat(); }
  if (server.hasArg("kd_r")) { KD_R = server.arg("kd_r").toFloat(); }
  
  // 重置 L/R 状态
  summederror_L = 0;
  last_error_L = 0;
  summederror_R = 0;
  last_error_R = 0;
  
  Serial.printf("PID L Gains Updated: KP_L=%.2f, KI_L=%.2f, KD_L=%.2f\n", KP_L, KI_L, KD_L);
  Serial.printf("PID R Gains Updated: KP_R=%.2f, KI_R=%.2f, KD_R=%.2f\n", KP_R, KI_R, KD_R);
  server.send(200, "text/plain", "OK - PID Updated");
}

// ==========================================================
// ==== 实时数据发送器 (JSON - L/R) ====
// ==========================================================
void handleGetData() {
  // 创建一个 JSON 字符串
  String json = "{";
  json += "\"target\": " + String(DesiredSpeed_Setpoint);
  json += ", \"measured_l\": " + String(MeasuredVelocity_CountsPerSec_L, 1);
  json += ", \"error_l\": " + String(g_current_error_L, 1);
  json += ", \"pwm_l\": " + String(g_current_pwm_output_L);
  json += ", \"measured_r\": " + String(MeasuredVelocity_CountsPerSec_R, 1);
  json += ", \"error_r\": " + String(g_current_error_R, 1);
  json += ", \"pwm_r\": " + String(g_current_pwm_output_R);
  json += "}";
  
  // 发送 JSON 响应
  server.send(200, "application/json", json);
}


// ==========================================================
// ==== SETUP & LOOP (All Functions Included) ====
// ==========================================================
void setup() {
  Serial.begin(115200);

  // --- 配置 Motor 1 (Left / G1) - 新引脚 ---
  pinMode(DIR_PIN_1, OUTPUT); // G19
  pinMode(DIR_PIN_2, OUTPUT); // G20
  pinMode(SPEED_PIN_L, OUTPUT); // G18
  pinMode(ENCODER_PIN_L, INPUT_PULLUP); 
  ledcAttach(SPEED_PIN_L, PWM_FREQ, PWM_BITS); // 附加 Pin 18
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), isr_encoder_L, CHANGE); // 附加中断 L
  
  // --- 配置 Motor 2 (Right / G0) - 不变 ---
  pinMode(DIR_PIN_3, OUTPUT); // G4
  pinMode(DIR_PIN_4, OUTPUT); // G5
  pinMode(SPEED_PIN_R, OUTPUT); // G7
  pinMode(ENCODER_PIN_R, INPUT_PULLUP); 
  ledcAttach(SPEED_PIN_R, PWM_FREQ, PWM_BITS); // 附加 Pin 7
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), isr_encoder_R, CHANGE); // 附加中断 R

  // Default stop
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, LOW);
  ledcWrite(SPEED_PIN_L, 0); 
  digitalWrite(DIR_PIN_3, LOW);
  digitalWrite(DIR_PIN_4, LOW);
  ledcWrite(SPEED_PIN_R, 0); 

  // Start AP mode
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("\nAP IP address: ");
  Serial.println(WiFi.softAPIP());

  // --- 注册所有的服务器端点 ---
  server.on("/", handleRoot);           
  server.on("/forward", handleForward); 
  server.on("/backward", handleBackward);
  server.on("/stop", handleStop);
  server.on("/setSpeed", handleSetSpeed);
  server.on("/setPID", handleSetPID);     
  server.on("/getData", handleGetData); 
  
  server.begin(); // 服务器启动
  Serial.println("Web server started! Connect to AP and visit http://192.168.4.1");
  
  last_pid_time = millis();
}

// ================================================================
// ==== 完整的 DUAL PID LOOP 函数 (使用 L/R 增益) ====
// ================================================================
void loop() {
  server.handleClient(); // 处理网页请求

  if (millis() - last_pid_time < PID_SAMPLE_TIME_MS) {
    return; // 还没到 50ms, 退出 loop
  }
  
  // --- 1. 测量 (获取 L/R 编码器数据) ---
  noInterrupts(); // 进入临界区
  long current_counts_L = EncoderCounts_L;
  long counts_delta_L = current_counts_L - last_counts_L; 
  last_counts_L = current_counts_L; 
  long current_counts_R = EncoderCounts_R;
  long counts_delta_R = current_counts_R - last_counts_R; 
  last_counts_R = current_counts_R; 
  interrupts(); // 退出临界区

  // --- 单位修复 (L/R) ---
  MeasuredVelocity_CountsPerSec_L = (float)counts_delta_L * (1000.0 / PID_SAMPLE_TIME_MS);
  MeasuredVelocity_CountsPerSec_R = (float)counts_delta_R * (1000.0 / PID_SAMPLE_TIME_MS);
  
  int pwm_output_L = 0;
  int pwm_output_R = 0;
  float LARGE_SUM = 2000.0; // 抗饱和限制

  // --- 2. 控制 (Motor L) ---
  if (motorDirection_L == 0) {
    pwm_output_L = 0;
    summederror_L = 0;
    last_error_L = 0;
    g_current_error_L = 0; 
  } else {
    // --- 3. PID 计算 (Motor L) ---
    float error_L = DesiredSpeed_Setpoint - MeasuredVelocity_CountsPerSec_L;
    g_current_error_L = error_L; 

    summederror_L += error_L;
    if (summederror_L > LARGE_SUM) summederror_L = LARGE_SUM;
    if (summederror_L < -LARGE_SUM) summederror_L = -LARGE_SUM;

    float d_term_error_L = error_L - last_error_L;
    last_error_L = error_L;

    // --- 使用独立的 L 增益 ---
    float pid_output_float_L = (KP_L * error_L) + (KI_L * summederror_L) + (KD_L * d_term_error_L);

    // --- 4. 执行 (PWM 范围限制 L) ---
    if (pid_output_float_L > 255) pid_output_float_L = 255;
    if (pid_output_float_L < 0) pid_output_float_L = 0;
    
    pwm_output_L = (int)pid_output_float_L;
  }
  
  // --- 2. 控制 (Motor R) ---
  if (motorDirection_R == 0) {
    pwm_output_R = 0;
    summederror_R = 0;
    last_error_R = 0;
    g_current_error_R = 0; 
  } else {
    // --- 3. PID 计算 (Motor R) ---
    float error_R = DesiredSpeed_Setpoint - MeasuredVelocity_CountsPerSec_R;
    g_current_error_R = error_R; 

    summederror_R += error_R;
    if (summederror_R > LARGE_SUM) summederror_R = LARGE_SUM;
    if (summederror_R < -LARGE_SUM) summederror_R = -LARGE_SUM;

    float d_term_error_R = error_R - last_error_R;
    last_error_R = error_R;

    // --- 使用独立的 R 增益 ---
    float pid_output_float_R = (KP_R * error_R) + (KI_R * summederror_R) + (KD_R * d_term_error_R);

    // --- 4. 执行 (PWM 范围限制 R) ---
    if (pid_output_float_R > 255) pid_output_float_R = 255;
    if (pid_output_float_R < 0) pid_output_float_R = 0;
    
    pwm_output_R = (int)pid_output_float_R;
  }

  // --- 5. 应用最终的 PWM 值 (L/R) ---
  g_current_pwm_output_L = pwm_output_L; 
  g_current_pwm_output_R = pwm_output_R;
  
  ledcWrite(SPEED_PIN_L, g_current_pwm_output_L); // 写入 G18
  ledcWrite(SPEED_PIN_R, g_current_pwm_output_R); // 写入 G7

  // --- 打印调试信息 (L/R) ---
  Serial.printf("L: T:%d M:%.1f E:%.1f P:%d | R: T:%d M:%.1f E:%.1f P:%d\n", 
                DesiredSpeed_Setpoint, MeasuredVelocity_CountsPerSec_L, g_current_error_L, g_current_pwm_output_L,
                DesiredSpeed_Setpoint, MeasuredVelocity_CountsPerSec_R, g_current_error_R, g_current_pwm_output_R);
                
  last_pid_time = millis(); // 在循环 *结束* 时重置 PID 计时器
}