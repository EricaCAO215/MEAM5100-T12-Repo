#include <WiFi.h>
#include <WebServer.h> // Library for Web Server
#include <esp32-hal-ledc.h>
#include <esp32-hal-adc.h>

// ==========================================================
// ==== Wi-Fi (AP Mode Configuration)
// ==========================================================
const char* AP_SSID = "ESP32_ENCODER_TEST"; // 新的 AP 名称
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

// --- 状态变量 ---
int g_current_pwm = 0; // 存储滑块的当前 PWM 值
int g_motor_direction = 0; // 0=Stop, 1=Fwd, -1=Rev

// ==========================================================
// ==== 编码器反馈 (无 PID) ====
// ==========================================================
const long ENCODER_SAMPLE_TIME_MS = 50; 
unsigned long last_sample_time = 0;

float MeasuredVelocity_CountsPerSec_L = 0; 
float MeasuredVelocity_CountsPerSec_R = 0; 

volatile long EncoderCounts_L = 0; 
volatile long last_counts_L = 0;   
volatile int motorDirection_L = 0; // (仅用于 ISR)
volatile long EncoderCounts_R = 0; 
volatile long last_counts_R = 0;   
volatile int motorDirection_R = 0; // (仅用于 ISR)

// ==========================================================
// ==== Interrupt Service Routines (L/R) ====
// ==========================================================
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
  String html = "<!DOCTYPE html><html><head><title>Dual Motor Encoder Test</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial; text-align:center; background-color:#222; color:white;}";
  html += ".btn{display:block; width:80%; margin:10px auto; padding:20px; font-size:24px; color:black; background-color:#00c2ff; text-decoration:none; border-radius:10px; cursor:pointer;}"; 
  html += ".stop{background-color:#ff4f4f;}";
  html += ".slider{width:80%; margin:20px auto;}";
  html += "#data-container{width:80%; margin:20px auto; padding:10px; border:1px solid #555; border-radius:10px; text-align:left; font-size:18px; line-height:1.6;}";
  html += "#data-container span{float:right; font-weight:bold; color:#00c2ff;}";
  html += "svg{width:80%; height:120px; background-color:#333; border:1px solid #555; border-radius:10px; margin-top:10px;}";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>L298N Encoder Test (No PID)</h1>"; // 标题已更新
  
  // --- 1. 实时 SVG 图表 ---
  html += "<h3>Live Chart (Last 5 Sec)</h3>";
  html += "<svg id='pid-chart' viewBox='0 0 300 100' preserveAspectRatio='none'>";
  html += "<polyline id='graph-measured-l' points='0,100' style='fill:none; stroke:#4CAF50; stroke-width:2;' />"; // 绿色 (Left)
  html += "<polyline id='graph-measured-r' points='0,100' style='fill:none; stroke:#FF6347; stroke-width:2;' />"; // 红色 (Right)
  html += "</svg>";
  
  // --- 2. 实时数据文本 ---
  html += "<h3>Live Encoder Data</h3>";
  html += "<div id='data-container'>";
  html += "Measured (L - G1): <span id='measured-l-val'>0.0</span> C/s<br>";
  html += "Measured (R - G0): <span id='measured-r-val'>0.0</span> C/s<br>";
  html += "</div>";

  // --- 方向控制 ---
  html += "<h3>Direction</h3>";
  html += "<button class='btn' onclick=\"fetch('/forward')\">FORWARD</button>";
  html += "<button class='btn' onclick=\"fetch('/backward')\">BACKWARD</button>";
  html += "<button class='btn stop' onclick=\"fetch('/stop')\">STOP</button>";
  
  // --- 速度控制 (直接控制 PWM) ---
  html += "<h3 id='speed-label'>Desired PWM (" + String(g_current_pwm) + ")</h3>";
  html += "<input type='range' min='0' max='255' value='" + String(g_current_pwm) + "' class='slider' id='speedSlider'";
  html += " onchange='setSpeed(this.value)'>";
  
  // --- JavaScript (包含数据轮询) ---
  html += "<script>";
  html += "const MAX_HISTORY = 50;"; 
  html += "let historyMeasured_L = [];";
  html += "let historyMeasured_R = [];";
  html += "const CHART_WIDTH = 300;"; 
  html += "const CHART_HEIGHT = 100;"; 
  html += "const MAX_SPEED = 86.0;"; // Y 轴最大值 (C/s) (保持不变以便与 PID 版本比较)
  
  html += "function setSpeed(val) {";
  html += " document.getElementById('speed-label').innerHTML = 'Desired PWM (' + val + ')';";
  html += " fetch('/setSpeed?value=' + val);";
  html += "}";
  // fetch() 是一种更现代的发送网页请求的方式
  html += "function sendCommand(cmd){ fetch('/' + cmd); }";
  
  // --- getData() JavaScript 函数和定时器 ---
  html += "function getData(){";
  html += "var xhr=new XMLHttpRequest();";
  html += "xhr.onreadystatechange = function() {";
  html += "if (this.readyState == 4 && this.status == 200) {";
  html += "var data = JSON.parse(this.responseText);"; // 解析 JSON
  // 1. 更新文本数据 (L/R)
  html += "document.getElementById('measured-l-val').innerHTML = data.measured_l.toFixed(1);"; 
  html += "document.getElementById('measured-r-val').innerHTML = data.measured_r.toFixed(1);"; 
  // 2. 更新图表
  html += "updateGraph(data.measured_l, data.measured_r);";
  html += "}";
  html += "};";
  html += "xhr.open('GET', '/getData', true);";
  html += "xhr.send();";
  html += "}";
  
  // --- updateGraph() JavaScript 函数 ---
  html += "function updateGraph(val_L, val_R) {";
  html += "historyMeasured_L.push(val_L);";
  html += "historyMeasured_R.push(val_R);";
  html += "if (historyMeasured_L.length > MAX_HISTORY) { historyMeasured_L.shift(); }";
  html += "if (historyMeasured_R.length > MAX_HISTORY) { historyMeasured_R.shift(); }";
  
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

// --- 控制函数 (L298N 逻辑) ---
void handleForward() {
  Serial.println("Command: FORWARD");
  g_motor_direction = 1;
  motorDirection_L = 1; // 告诉 ISR 开始正向计数
  motorDirection_R = 1; 
  
  digitalWrite(DIR_PIN_1, HIGH); // IN1 (G19)
  digitalWrite(DIR_PIN_2, LOW);  // IN2 (G20)
  digitalWrite(DIR_PIN_3, HIGH); // IN3 (G4)
  digitalWrite(DIR_PIN_4, LOW);  // IN4 (G5)
  
  ledcWrite(SPEED_PIN_L, g_current_pwm);
  ledcWrite(SPEED_PIN_R, g_current_pwm);
  
  server.send(200, "text/plain", "OK - Forward");
}

void handleBackward() {
  Serial.println("Command: BACKWARD");
  g_motor_direction = -1;
  motorDirection_L = -1; // 告诉 ISR 开始反向计数
  motorDirection_R = -1;
  
  digitalWrite(DIR_PIN_1, LOW);  // IN1 (G19)
  digitalWrite(DIR_PIN_2, HIGH); // IN2 (G20)
  digitalWrite(DIR_PIN_3, LOW);  // IN3 (G4)
  digitalWrite(DIR_PIN_4, HIGH); // IN4 (G5)
  
  ledcWrite(SPEED_PIN_L, g_current_pwm);
  ledcWrite(SPEED_PIN_R, g_current_pwm);
  
  server.send(200, "text/plain", "OK - Backward");
}

void handleStop() {
  Serial.println("Command: STOP");
  g_motor_direction = 0;
  motorDirection_L = 0; // 告诉 ISR 停止计数
  motorDirection_R = 0;

  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, LOW);
  digitalWrite(DIR_PIN_3, LOW);
  digitalWrite(DIR_PIN_4, LOW);
  
  ledcWrite(SPEED_PIN_L, 0);
  ledcWrite(SPEED_PIN_R, 0);

  server.send(200, "text/plain", "OK - Stopped");
}

void handleSetSpeed() {
  String speedStr = server.arg("value");
  if (speedStr != "") {
    g_current_pwm = speedStr.toInt();
    Serial.printf("Command: Set PWM to %d\n", g_current_pwm);
    // 只有在电机*没有*停止时才更新 PWM
    if (g_motor_direction != 0) {
      ledcWrite(SPEED_PIN_L, g_current_pwm);
      ledcWrite(SPEED_PIN_R, g_current_pwm);
    }
  }
  server.send(200, "text/plain", "OK - Speed set");
}

// ==========================================================
// ==== 实时数据发送器 (JSON - L/R) ====
// ==========================================================
void handleGetData() {
  // 创建一个 JSON 字符串 (只发送测量值)
  String json = "{";
  json += "\"measured_l\": " + String(MeasuredVelocity_CountsPerSec_L, 1);
  json += ", \"measured_r\": " + String(MeasuredVelocity_CountsPerSec_R, 1);
  json += "}";
  
  // 发送 JSON 响应
  server.send(200, "application/json", json);
}


// ==========================================================
// ==== SETUP & LOOP (All Functions Included) ====
// ==========================================================
void setup() {
  Serial.begin(115200);

  // --- 配置 Motor 1 (Left) - 新引脚 ---
  pinMode(DIR_PIN_1, OUTPUT); // G19
  pinMode(DIR_PIN_2, OUTPUT); // G20
  pinMode(SPEED_PIN_L, OUTPUT); // G18
  pinMode(ENCODER_PIN_L, INPUT_PULLUP); 
  ledcAttach(SPEED_PIN_L, PWM_FREQ, PWM_BITS); // 附加 Pin 18
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), isr_encoder_L, CHANGE); // 附加中断 L
  
  // --- 配置 Motor 2 (Right) - 不变 ---
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

  // --- 注册服务器端点 ---
  server.on("/", handleRoot);           
  server.on("/forward", handleForward); 
  server.on("/backward", handleBackward);
  server.on("/stop", handleStop);
  server.on("/setSpeed", handleSetSpeed);
  server.on("/getData", handleGetData); // 新增数据端点
  
  server.begin(); 
  Serial.println("Web server started! Connect to AP and visit http://192.168.4.1");
  
  last_sample_time = millis();
}

// ================================================================
// ==== 仅轮询编码器 (无 PID) ====
// ================================================================
void loop() {
  server.handleClient(); // 处理网页请求

  // ============================================
  // ==== 编码器采样循环 ====
  // ============================================
  
  if (millis() - last_sample_time < ENCODER_SAMPLE_TIME_MS) {
    return; // 还没到 50ms, 退出 loop
  }
  last_sample_time = millis(); // 重置计时器
  
  // --- 1. 测量 (从中断变量中读取计数值) ---
  noInterrupts(); // 进入临界区
  long current_counts_L = EncoderCounts_L;
  long counts_delta_L = current_counts_L - last_counts_L; 
  last_counts_L = current_counts_L; 
  long current_counts_R = EncoderCounts_R;
  long counts_delta_R = current_counts_R - last_counts_R; 
  last_counts_R = current_counts_R; 
  interrupts(); // 退出临界区

  // --- 2. 单位修复 (从 counts/50ms 转换为 counts/second) ---
  float conversion_factor = (1000.0 / ENCODER_SAMPLE_TIME_MS);
  MeasuredVelocity_CountsPerSec_L = (float)counts_delta_L * conversion_factor;
  MeasuredVelocity_CountsPerSec_R = (float)counts_delta_R * conversion_factor;

  // --- 3. 打印调试信息 ---
  Serial.printf("L: %.1f C/s | R: %.1f C/s\n", 
                MeasuredVelocity_CountsPerSec_L, 
                MeasuredVelocity_CountsPerSec_R);
}


