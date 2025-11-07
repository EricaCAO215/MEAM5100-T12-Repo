#include <WiFi.h>
#include <WebServer.h> // Library for Web Server
#include <esp32-hal-ledc.h>
#include <esp32-hal-adc.h>

// ==========================================================
// ==== Wi-Fi (AP Mode Configuration)
// ==========================================================
const char* AP_SSID = "ESP32_PID_DASHBOARD";
const char* AP_PASS = "123456789"; // 密码必须至少8个字符
IPAddress AP_IP(192, 168, 4, 1);
// ==========================================================
// ==== Hardware Pins & PWM Parameters ====
// ==========================================================
const int SPEED_PIN = 6;    // PWM Speed Control (H-Bridge EN)
const int DIR_PIN_1 = 10;   // Direction Input 1 (H-Bridge 1A)
const int DIR_PIN_2 = 8;    // Direction Input 2 (H-Bridge 2A)
const int ENCODER_PIN = 1;  // Encoder signal (G1)

const int PWM_FREQ = 250;     // PWM Frequency (Hz)
const int PWM_BITS = 8;       // Resolution (8-bit = 0-255 range)

// ==========================================================
// ==== PID 闭环控制 (Lab 4.2.2) ====
// ==========================================================
// --- PID 增益 (Tuning) ---
float KP = 3.0; 
float KI = 0.5; 
float KD = 0.1; 

// --- PID 状态变量 ---
float summederror = 0;
float last_error = 0;

// --- PID 采样时间 ---
const long PID_SAMPLE_TIME_MS = 50; // 每 50ms 运行一次 PID 计算
unsigned long last_pid_time = 0;

// --- 速度和设定值变量 ---
// DesiredSpeed_Setpoint (来自滑块) 现在表示 (counts / second)
int DesiredSpeed_Setpoint = 0; 
// MeasuredVelocity_CountsPerSec 是编码器测得的速度 (counts / second)
float MeasuredVelocity_CountsPerSec = 0; 

// ==========================================================
// ==== 实时数据显示的全局变量 ====
// ==========================================================
// 这些变量将被 loop() 更新, 并被 handleGetData() 读取
float g_current_error = 0;
int   g_current_pwm_output = 0;

// ==========================================================
// ==== Encoder / Counter Variables (Lab 4.1.5) ====
// ==========================================================
volatile long EncoderCounts = 0; // 总脉冲计数
volatile long last_counts = 0;   // 上一个 PID 周期的脉冲计数
volatile int motorDirection = 0; // Motor direction (+1: FWD, -1: REV, 0: STOP)

// ==========================================================
// ==== Interrupt Service Routine (ISR) ====
// ==========================================================
void IRAM_ATTR isr_encoder() {
  EncoderCounts += motorDirection;
}

// ==========================================================
// ==== Web Server HTML and Control Functions ====
// ==========================================================
WebServer server(80); // Create server on standard HTTP port 80

void handleRoot() {
  // --- HTML (已更新, 包含 PID 调参表单 和 实时数据 div) ---
  String html = "<!DOCTYPE html><html><head><title>ESP32 PID Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial; text-align:center; background-color:#222; color:white;}";
  html += ".btn{display:block; width:80%; margin:10px auto; padding:20px; font-size:24px; color:black; background-color:#00c2ff; text-decoration:none; border-radius:10px; cursor:pointer;}"; 
  html += ".stop{background-color:#ff4f4f;}";
  html += ".slider{width:80%; margin:20px auto;}";
  html += "input[type='number']{width: 80px; font-size: 16px; margin: 5px;}";
  html += "form{margin: 20px auto; width: 80%; padding: 10px; border: 1px solid #555; border-radius: 10px;}";
  html += "input[type='submit']{background-color:#4CAF50; color:white; padding:10px 20px; border:none; border-radius:5px; font-size:16px; cursor:pointer;}";
  // --- 实时数据容器的样式 ---
  html += "#data-container{width:80%; margin:20px auto; padding:10px; border:1px solid #555; border-radius:10px; text-align:left; font-size:18px; line-height:1.6;}";
  html += "#data-container span{float:right; font-weight:bold; color:#00c2ff;}";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>PID Motor Control (Lab 4.2.2)</h1>";
  
  // ==========================================================
  // ==== 实时数据容器 (Live Data Container) ====
  // ==========================================================
  html += "<h3>Live Data</h3>";
  html += "<div id='data-container'>";
  html += "Target: <span id='target-val'>0</span> C/s<br>";
  html += "Measured: <span id='measured-val'>0.0</span> C/s<br>";
  html += "Error: <span id='error-val'>0.0</span><br>";
  html += "PWM: <span id='pwm-val'>0</span>";
  html += "</div>";

  // --- 方向控制 ---
  html += "<h3>Direction</h3>";
  html += "<button class='btn' onclick=\"sendCommand('forward')\">FORWARD</button>";
  html += "<button class='btn' onclick=\"sendCommand('backward')\">BACKWARD</button>";
  html += "<button class='btn stop' onclick=\"sendCommand('stop')\">STOP</button>";
  
  // --- 速度控制 ---
  html += "<h3 id='speed-label'>Desired Speed (" + String(DesiredSpeed_Setpoint) + " C/s)</h3>";
  
  // ==========================================================
  // ==== 最终修复: 滑块最大值已设置为 86 ====
  // ==========================================================
  html += "<input type='range' min='0' max='86' value='" + String(DesiredSpeed_Setpoint) + "' class='slider' id='speedSlider'";
  html += " onchange='setSpeed(this.value)'>";
  
  // --- PID 调参表单 ---
  html += "<h3>PID Tuning</h3>";
  html += "<form onsubmit='updatePID(event)'>";
  html += "KP: <input type='number' step='0.1' id='kp' value='" + String(KP) + "'><br>";
  html += "KI: <input type='number' step='0.1' id='ki' value='" + String(KI) + "'><br>";
  html += "KD: <input type='number' step='0.1' id='kd' value='" + String(KD) + "'><br>";
  html += "<br><input type='submit' value='Update PID'>";
  html += "</form>";

  // --- JavaScript (包含数据轮询) ---
  html += "<script>";
  html += "function sendCommand(cmd){ var xhr = new XMLHttpRequest(); xhr.open('GET', '/' + cmd, true); xhr.send();}";
  html += "function setSpeed(val){ var xhr = new XMLHttpRequest(); xhr.open('GET', '/setSpeed?value=' + val, true); xhr.send(); document.getElementById('speed-label').innerHTML = 'Desired Speed (' + val + ' C/s)';}";
  
  html += "function updatePID(e){";
  html += "e.preventDefault();"; // 阻止表单提交导致页面刷新
  html += "var kp=document.getElementById('kp').value;";
  html += "var ki=document.getElementById('ki').value;";
  html += "var kd=document.getElementById('kd').value;";
  html += "var xhr=new XMLHttpRequest();";
  html += "xhr.open('GET', '/setPID?kp='+kp+'&ki='+ki+'&kd='+kd, true);";
  html += "xhr.send();";
  html += "}"; 
  
  // --- getData() JavaScript 函数和定时器 ---
  html += "function getData(){";
  html += "var xhr=new XMLHttpRequest();";
  html += "xhr.onreadystatechange = function() {";
  html += "if (this.readyState == 4 && this.status == 200) {";
  html += "var data = JSON.parse(this.responseText);"; // 解析 ESP32 发来的 JSON
  // 更新 HTML 页面上的值
  html += "document.getElementById('target-val').innerHTML = data.target;";
  html += "document.getElementById('measured-val').innerHTML = data.measured.toFixed(1);"; // 保留1位小数
  html += "document.getElementById('error-val').innerHTML = data.error.toFixed(1);"; // 保留1位小数
  html += "document.getElementById('pwm-val').innerHTML = data.pwm;";
  html += "}";
  html += "};";
  html += "xhr.open('GET', '/getData', true);";
  html += "xhr.send();";
  html += "}";
  // --- 启动定时器: 每 500ms 自动调用一次 getData() ---
  html += "setInterval(getData, 500);"; 
  html += "</script>";
  
  html += "</body></html>";
  server.send(200, "text/html", html); 
}

// --- Control functions (MODIFIED FOR PID) ---
void handleForward() {
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, HIGH);
  motorDirection = 1; 
  summederror = 0; 
  last_error = 0;
  server.send(200, "text/plain", "OK - Forward");
}

void handleBackward() {
  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(DIR_PIN_2, LOW);
  motorDirection = -1; 
  summederror = 0; 
  last_error = 0;
  server.send(200, "text/plain", "OK - Backward");
}

void handleStop() {
  motorDirection = 0;
  DesiredSpeed_Setpoint = 0;
  summederror = 0; 
  last_error = 0;
  server.send(200, "text/plain", "OK - Stopped");
}

void handleSetSpeed() {
  String speedStr = server.arg("value");
  if (speedStr != "") {
    DesiredSpeed_Setpoint = speedStr.toInt();
  }
  server.send(200, "text/plain", "OK - Speed set to " + String(DesiredSpeed_Setpoint));
}

// ==========================================================
// ==== PID 调参处理器 ====
// ==========================================================
void handleSetPID() {
  if (server.hasArg("kp")) {
    KP = server.arg("kp").toFloat(); 
  }
  if (server.hasArg("ki")) {
    KI = server.arg("ki").toFloat();
  }
  if (server.hasArg("kd")) {
    KD = server.arg("kd").toFloat();
  }
  summederror = 0;
  last_error = 0;
  
  Serial.printf("PID Gains Updated: KP=%.2f, KI=%.2f, KD=%.2f\n", KP, KI, KD);
  server.send(200, "text/plain", "OK - PID Updated");
}

// ==========================================================
// ==== 实时数据发送器 (JSON) ====
// ==========================================================
void handleGetData() {
  // 创建一个 JSON 字符串
  // 格式: {"target": 50, "measured": 48.5, "error": 1.5, "pwm": 120}
  String json = "{";
  json += "\"target\": " + String(DesiredSpeed_Setpoint);
  json += ", \"measured\": " + String(MeasuredVelocity_CountsPerSec, 1); // 1位小数
  json += ", \"error\": " + String(g_current_error, 1); // 1位小数
  json += ", \"pwm\": " + String(g_current_pwm_output);
  json += "}";
  
  // 发送 JSON 响应
  server.send(200, "application/json", json);
}


// ==========================================================
// ==== SETUP & LOOP (All Functions Included) ====
// ==========================================================
void setup() {
  Serial.begin(115200);

  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(SPEED_PIN, OUTPUT);
  ledcAttach(SPEED_PIN, PWM_FREQ, PWM_BITS);
  
  pinMode(ENCODER_PIN, INPUT_PULLUP); 
  // 修复 1: 将中断模式更改为 CHANGE
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), isr_encoder, CHANGE);
  
  // Default stop
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, LOW);
  ledcWrite(SPEED_PIN, 0);

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
  server.on("/setPID", handleSetPID);     // (用于 PID 调参)
  server.on("/getData", handleGetData); // (用于实时数据轮询)
  
  server.begin(); // 服务器启动
  Serial.println("Web server started! Connect to AP and visit http://192.168.4.1");
  
  last_pid_time = millis();
}

// ================================================================
// ==== 修复 2: 完整的 LOOP 函数已更新为使用 counts/second ====
// ================================================================
void loop() {
  server.handleClient(); // 处理网页请求

  // ============================================
  // ==== PID 闭环控制循环 (Lab 4.2.2) ====
  // ============================================
  
  if (millis() - last_pid_time < PID_SAMPLE_TIME_MS) {
    return; // 还没到 50ms, 退出 loop
  }
  
  // --- 1. 测量 (获取编码器数据) ---
  noInterrupts(); // 进入临界区
  long current_counts = EncoderCounts;
  long counts_delta = current_counts - last_counts; 
  last_counts = current_counts; 
  interrupts(); // 退出临界区

  // --- 单位修复 (从 counts/sample 转换为 counts/second) ---
  MeasuredVelocity_CountsPerSec = (float)counts_delta * (1000.0 / PID_SAMPLE_TIME_MS);
  
  int pwm_output = 0;

  // --- 2. 控制 (如果电机停止，则跳过 PID) ---
  if (motorDirection == 0) {
    pwm_output = 0;
    summederror = 0;
    last_error = 0;
    g_current_error = 0; // 更新全局变量
  } else {
    // --- 3. PID 计算 (现在以 counts/second 为单位) ---
    
    // P-Term: 比例
    float error = DesiredSpeed_Setpoint - MeasuredVelocity_CountsPerSec;
    g_current_error = error; // 更新全局变量以供网页显示

    // I-Term: 积分 (带抗饱和)
    summederror += error;
    
    float LARGE_SUM = 2000.0; 
    if (summederror > LARGE_SUM) summederror = LARGE_SUM;
    if (summederror < -LARGE_SUM) summederror = -LARGE_SUM;

    // D-Term: 微分
    float d_term_error = error - last_error;
    last_error = error;

    // --- 最终 PID 输出 ---
    float pid_output_float = (KP * error) + (KI * summederror) + (KD * d_term_error);

    // --- 4. 执行 (PWM 范围限制) ---
    if (pid_output_float > 255) pid_output_float = 255;
    if (pid_output_float < 0) pid_output_float = 0;
    
    pwm_output = (int)pid_output_float;
  }
  
  // --- 5. 应用最终的 PWM 值 ---
  g_current_pwm_output = pwm_output; // 更新全局变量以供网页显示
  ledcWrite(SPEED_PIN, g_current_pwm_output);

  // 打印调试信息 (单位已更新)
  Serial.printf("Target: %d C/s, Measured: %.1f C/s, Error: %.1f, PWM: %d\n", 
                DesiredSpeed_Setpoint, 
                MeasuredVelocity_CountsPerSec, 
                g_current_error,
                g_current_pwm_output);
                
  last_pid_time = millis(); // 在循环 *结束* 时重置 PID 计时器
}