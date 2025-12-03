#include <WiFi.h>
#include <WebServer.h>
#include <esp32-hal-ledc.h>
#include <esp32-hal-adc.h>

// ================== 硬件说明 ==================
// 双电机 + 双 BTS7960 + 金属编码电机 (AB 两相)
// 左电机 BTS7960：RPWM <- GPIO18, LPWM <- GPIO19, R_EN/L_EN -> 5V
// 右电机 BTS7960：RPWM <- GPIO7,  LPWM <- GPIO6,  R_EN/L_EN -> 5V
// 左编码器：A(黄)->GPIO1, B(白)->GPIO2
// 右编码器：A(黄)->GPIO5, B(白)->GPIO4
// 所有 GND 必须共地
// ==================================================

// ========== Wi-Fi ==========
const char* AP_SSID = "ESP32_PID_CHART_DUAL";
const char* AP_PASS = "123456789";
IPAddress AP_IP(192,168,4,1);

// ========== 电机控制脚（BTS7960） ==========
// 左电机：1 PWM + 1 DIR
const int SPEED_PIN_L = 7;  // 左：RPWM（速度 PWM）
const int DIR_PIN_L   = 6;  // 左：LPWM（方向电平，高/低切换正反转）

// 右电机：1 PWM + 1 DIR
const int SPEED_PIN_R = 0;   // 右：RPWM（速度 PWM）
const int DIR_PIN_R   = 1;   // 右：LPWM（方向电平）

const int PWM_FREQ = 250;
const int PWM_BITS = 8;

// ========== 编码器脚（AB 两相） ==========
// 左编码器
const int ENCODER_L_A = 18;   // 左 A 相（黄）
const int ENCODER_L_B = 19;   // 左 B 相（白）
// 右编码器
const int ENCODER_R_A = 5;   // 右 A 相（黄）
const int ENCODER_R_B = 4;   // 右 B 相（白）

// ========== PID 参数 ==========
float KP_L=3.0, KI_L=0.0, KD_L=0.1;
float KP_R=3.0, KI_R=0.0, KD_R=0.1;

float summederror_L=0, last_error_L=0;
float summederror_R=0, last_error_R=0;

const long PID_SAMPLE_TIME_MS = 50;
unsigned long last_pid_time = 0;

int   DesiredSpeed_Setpoint_L = 0;
int   DesiredSpeed_Setpoint_R = 0;
int   BaseSpeed_Setpoint      = 0;

float MeasuredVelocity_CountsPerSec_L = 0;
float MeasuredVelocity_CountsPerSec_R = 0;

float g_current_error_L=0; int g_current_pwm_output_L=0;
float g_current_error_R=0; int g_current_pwm_output_R=0;

// 编码器正负计数
volatile long EncoderCounts_L = 0;
volatile long EncoderCounts_R = 0;
long last_counts_L = 0;
long last_counts_R = 0;

// 方向状态：1 = 正转，-1 = 反转，0 = 停止
int motorDirection_L = 0;
int motorDirection_R = 0;

// 正交解码用：记录 A 相前一状态 + 去抖
int  prevAL = HIGH, prevAR = HIGH;
unsigned long lastEdgeUsL = 0, lastEdgeUsR = 0;
const unsigned long DEBOUNCE_US = 200;

WebServer server(80);

// ========== 函数声明 ==========
void handleRoot();
void handleForward();
void handleBackward();
void handleStop();
void handleSetSpeed();
void handleSetPID();
void handleGetData();
void handleSetSpeedLR();
void handleSteer();
void handlePivot();

static inline int clamp255(float x){ if (x<0) return 0; if (x>255) return 255; return (int)x; }

void setup() {
  Serial.begin(115200);

  // 电机控制脚
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);
  pinMode(SPEED_PIN_L, OUTPUT);
  pinMode(SPEED_PIN_R, OUTPUT);

  // 编码器脚：全部上拉输入
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  prevAL = digitalRead(ENCODER_L_A);
  prevAR = digitalRead(ENCODER_R_A);

  // PWM
  ledcAttach(SPEED_PIN_L, PWM_FREQ, PWM_BITS);
  ledcAttach(SPEED_PIN_R, PWM_FREQ, PWM_BITS);
  ledcWrite(SPEED_PIN_L, 0);
  ledcWrite(SPEED_PIN_R, 0);

  digitalWrite(DIR_PIN_L, LOW);
  digitalWrite(DIR_PIN_R, LOW);

  // Wi-Fi AP
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println(WiFi.softAPIP());

  // HTTP 路由
  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/stop", handleStop);
  server.on("/setSpeed", handleSetSpeed);
  server.on("/setPID", handleSetPID);
  server.on("/getData", handleGetData);
  server.on("/setSpeedLR", handleSetSpeedLR);
  server.on("/steer", handleSteer);
  server.on("/pivot", handlePivot);
  server.begin();

  last_pid_time = millis();
}

// ========== 编码器正交解码：A/B 两相 ==========
inline void pollEncoders() {
  // ---- 左电机 ----
  int curAL = digitalRead(ENCODER_L_A);
  int curBL = digitalRead(ENCODER_L_B);

  if (prevAL == LOW && curAL == HIGH) {  // A 上升沿
    unsigned long now = micros();
    if (now - lastEdgeUsL > DEBOUNCE_US) {
      int dir = (curBL == HIGH) ? +1 : -1;  // B=HIGH 定义为 + 方向
      EncoderCounts_L += dir;
      lastEdgeUsL = now;
    }
  }
  prevAL = curAL;

  // ---- 右电机 ----
  int curAR = digitalRead(ENCODER_R_A);
  int curBR = digitalRead(ENCODER_R_B);

  if (prevAR == LOW && curAR == HIGH) {  // A 上升沿
    unsigned long now = micros();
    if (now - lastEdgeUsR > DEBOUNCE_US) {
      int dir = (curBR == HIGH) ? +1 : -1;
      EncoderCounts_R += dir;
      lastEdgeUsR = now;
    }
  }
  prevAR = curAR;
}

// ========== 主循环 ==========
void loop() {
  server.handleClient();
  pollEncoders();

  if (millis() - last_pid_time < PID_SAMPLE_TIME_MS) return;

  noInterrupts();
  long current_counts_L = EncoderCounts_L;
  long current_counts_R = EncoderCounts_R;
  interrupts();

  long counts_delta_L = current_counts_L - last_counts_L;
  long counts_delta_R = current_counts_R - last_counts_R;
  last_counts_L = current_counts_L;
  last_counts_R = current_counts_R;

  float velL = (float)counts_delta_L * (1000.0 / PID_SAMPLE_TIME_MS);
  float velR = (float)counts_delta_R * (1000.0 / PID_SAMPLE_TIME_MS);

  // PID 只关心速度大小，方向用 motorDirection_* 控制
  MeasuredVelocity_CountsPerSec_L = fabs(velL);
  MeasuredVelocity_CountsPerSec_R = fabs(velR);

  int pwm_output_L = 0, pwm_output_R = 0;
  float LARGE_SUM = 2000.0;

  if (motorDirection_L == 0) {
    pwm_output_L = 0; summederror_L = 0; last_error_L = 0; g_current_error_L = 0;
  } else {
    float error_L = (float)DesiredSpeed_Setpoint_L - MeasuredVelocity_CountsPerSec_L;
    g_current_error_L = error_L;
    summederror_L += error_L;
    if (summederror_L >  LARGE_SUM) summederror_L =  LARGE_SUM;
    if (summederror_L < -LARGE_SUM) summederror_L = -LARGE_SUM;
    float dL = error_L - last_error_L; last_error_L = error_L;
    pwm_output_L = clamp255(KP_L*error_L + KI_L*summederror_L + KD_L*dL);
  }

  if (motorDirection_R == 0) {
    pwm_output_R = 0; summederror_R = 0; last_error_R = 0; g_current_error_R = 0;
  } else {
    float error_R = (float)DesiredSpeed_Setpoint_R - MeasuredVelocity_CountsPerSec_R;
    g_current_error_R = error_R;
    summederror_R += error_R;
    if (summederror_R >  LARGE_SUM) summederror_R =  LARGE_SUM;
    if (summederror_R < -LARGE_SUM) summederror_R = -LARGE_SUM;
    float dR = error_R - last_error_R; last_error_R = error_R;
    pwm_output_R = clamp255(KP_R*error_R + KI_R*summederror_R + KD_R*dR);
  }

  g_current_pwm_output_L = pwm_output_L;
  g_current_pwm_output_R = pwm_output_R;

  // 根据 motorDirection_* 设置方向电平，然后写 PWM
  if (motorDirection_L > 0) {
    digitalWrite(DIR_PIN_L, HIGH);   // 正转
  } else if (motorDirection_L < 0) {
    digitalWrite(DIR_PIN_L, LOW);    // 反转
  }
  if (motorDirection_R > 0) {
    digitalWrite(DIR_PIN_R, HIGH);
  } else if (motorDirection_R < 0) {
    digitalWrite(DIR_PIN_R, LOW);
  }

  ledcWrite(SPEED_PIN_L, g_current_pwm_output_L);
  ledcWrite(SPEED_PIN_R, g_current_pwm_output_R);

  Serial.printf("L: TL:%d M:%.1f E:%.1f P:%d | R: TR:%d M:%.1f E:%.1f P:%d\n",
    DesiredSpeed_Setpoint_L, MeasuredVelocity_CountsPerSec_L, g_current_error_L, g_current_pwm_output_L,
    DesiredSpeed_Setpoint_R, MeasuredVelocity_CountsPerSec_R, g_current_error_R, g_current_pwm_output_R);

  last_pid_time = millis();
}

// =================== 网页 ===================

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
  html += ".turn-section{margin-top:30px;}";
  html += ".turn-btn{display:inline-block; width:38%; margin:5px; padding:15px; font-size:18px; color:black; background-color:#ffaa00; border:none; border-radius:10px; cursor:pointer;}";
  html += "</style></head><body>";
  html += "<h1>Dual PID Motor Control (BTS7960 + AB Encoders)</h1>";

  html += "<h3>Live Chart (Last 5 Sec)</h3>";
  html += "<svg id='pid-chart' viewBox='0 0 300 100' preserveAspectRatio='none'>";
  html += "<line id='graph-target' x1='0' y1='50' x2='300' y2='50' style='stroke:#00c2ff; stroke-width:2; stroke-dasharray: 4;' />";
  html += "<polyline id='graph-measured-l' points='0,100' style='fill:none; stroke:#4CAF50; stroke-width:2;' />";
  html += "<polyline id='graph-measured-r' points='0,100' style='fill:none; stroke:#FF6347; stroke-width:2;' />";
  html += "</svg>";

  html += "<h3>Live Data</h3>";
  html += "<div id='data-container'>";
  html += "Target: <span id='target-val'>0</span> C/s<br><hr style='border-color:#444;'>";
  html += "Measured (L): <span id='measured-l-val'>0.0</span> C/s<br>";
  html += "Error (L): <span id='error-l-val'>0.0</span><br>";
  html += "PWM (L): <span id='pwm-l-val'>0</span><br><hr style='border-color:#444;'>";
  html += "Measured (R): <span id='measured-r-val'>0.0</span> C/s<br>";
  html += "Error (R): <span id='error-r-val'>0.0</span><br>";
  html += "PWM (R): <span id='pwm-r-val'>0</span></div>";

  html += "<h3>Direction</h3>";
  html += "<button class='btn' onclick=\"sendCommand('forward')\">FORWARD</button>";
  html += "<button class='btn' onclick=\"sendCommand('backward')\">BACKWARD</button>";
  html += "<button class='btn stop' onclick=\"sendCommand('stop')\">STOP</button>";

  html += "<h3 id='speed-label'>Desired Speed (0 C/s)</h3>";
  html += "<input type='range' min='0' max='200' value='0' class='slider' id='speedSlider' onchange='setSpeed(this.value)'>";

  html += "<h3>PID Tuning (Independent)</h3>";
  html += "<form onsubmit='updatePID(event)'>";
  html += "<div><b>Left Motor</b><br>";
  html += "KP (L): <input type='number' step='0.1' id='kp_l' value='3.0'><br>";
  html += "KI (L): <input type='number' step='0.01' id='ki_l' value='0.0'><br>";
  html += "KD (L): <input type='number' step='0.01' id='kd_l' value='0.1'><br></div>";
  html += "<div><b>Right Motor</b><br>";
  html += "KP (R): <input type='number' step='0.1' id='kp_r' value='3.0'><br>";
  html += "KI (R): <input type='number' step='0.01' id='ki_r' value='0.0'><br>";
  html += "KD (R): <input type='number' step='0.01' id='kd_r' value='0.1'><br></div>";
  html += "<br><br><input type='submit' value='Update All PID'></form>";

  html += "<div class='turn-section'><h3>Turning Control</h3>";
  html += "<button class='turn-btn' onclick=\"turnRight()\">Steer Left</button>";
  html += "<button class='turn-btn' onclick=\"turnLeft()\">Steer Right</button><br>";
  html += "<button class='turn-btn' onclick=\"pivotLeft()\">Pivot Left</button>";
  html += "<button class='turn-btn' onclick=\"pivotRight()\">Pivot Right</button></div>";

  html += "<script>";
  html += "const MAX_HISTORY=50,CHART_WIDTH=300,CHART_HEIGHT=100,MAX_SPEED=200.0;";
  html += "let hL=[],hR=[];";
  html += "function sendCommand(cmd){var x=new XMLHttpRequest();x.open('GET','/'+cmd,true);x.send();}";
  html += "function setSpeed(v){var x=new XMLHttpRequest();x.open('GET','/setSpeed?value='+v,true);x.send();document.getElementById('speed-label').innerHTML='Desired Speed ('+v+' C/s)';}";
  html += "function updatePID(e){e.preventDefault();var kp_l=document.getElementById('kp_l').value,ki_l=document.getElementById('ki_l').value,kd_l=document.getElementById('kd_l').value,kp_r=document.getElementById('kp_r').value,ki_r=document.getElementById('ki_r').value,kd_r=document.getElementById('kd_r').value;var x=new XMLHttpRequest();x.open('GET','/setPID?kp_l='+kp_l+'&ki_l='+ki_l+'&kd_l='+kd_l+'&kp_r='+kp_r+'&ki_r='+ki_r+'&kd_r='+kd_r,true);x.send();}";
  html += "function getData(){var x=new XMLHttpRequest();x.onreadystatechange=function(){if(this.readyState==4&&this.status==200){var d=JSON.parse(this.responseText);document.getElementById('target-val').innerHTML=d.target;document.getElementById('measured-l-val').innerHTML=d.measured_l.toFixed(1);document.getElementById('error-l-val').innerHTML=d.error_l.toFixed(1);document.getElementById('pwm-l-val').innerHTML=d.pwm_l;document.getElementById('measured-r-val').innerHTML=d.measured_r.toFixed(1);document.getElementById('error-r-val').innerHTML=d.error_r.toFixed(1);document.getElementById('pwm-r-val').innerHTML=d.pwm_r;updateGraph(d.measured_l,d.measured_r,d.target);}};x.open('GET','/getData',true);x.send();}";
  html += "function updateGraph(vL,vR,t){hL.push(vL);hR.push(vR);if(hL.length>MAX_HISTORY)hL.shift();if(hR.length>MAX_HISTORY)hR.shift();var yT=CHART_HEIGHT-(t/MAX_SPEED)*CHART_HEIGHT;if(yT<0)yT=0;if(yT>CHART_HEIGHT)yT=CHART_HEIGHT;var tl=document.getElementById('graph-target');tl.setAttribute('y1',yT);tl.setAttribute('y2',yT);var pL='',pR='';for(var i=0;i<hL.length;i++){var x=(i/(MAX_HISTORY-1))*CHART_WIDTH;var yL=CHART_HEIGHT-(hL[i]/MAX_SPEED)*CHART_HEIGHT;if(yL<0)yL=0;if(yL>CHART_HEIGHT)yL=CHART_HEIGHT;pL+=x+','+yL+' ';if(i<hR.length){var yR=CHART_HEIGHT-(hR[i]/MAX_SPEED)*CHART_HEIGHT;if(yR<0)yR=0;if(yR>CHART_HEIGHT)yR=CHART_HEIGHT;pR+=x+','+yR+' ';}}document.getElementById('graph-measured-l').setAttribute('points',pL);document.getElementById('graph-measured-r').setAttribute('points',pR);}";
  html += "function turnLeft(){var x=new XMLHttpRequest();x.open('GET','/steer?delta=-50',true);x.send();}";
  html += "function turnRight(){var x=new XMLHttpRequest();x.open('GET','/steer?delta=50',true);x.send();}";
  html += "function pivotLeft(){var x=new XMLHttpRequest();x.open('GET','/pivot?dir=left&spd=100',true);x.send();}";
  html += "function pivotRight(){var x=new XMLHttpRequest();x.open('GET','/pivot?dir=right&spd=100',true);x.send();}";
  html += "setInterval(getData,100);</script></body></html>";
  server.send(200,"text/html",html);
}

// ========== 动作接口：前进 / 后退 / 停止 / 转向 / 原地转 ==========

// 前进：两个 motorDirection = +1
void handleForward(){
  motorDirection_L = 1;
  motorDirection_R = 1;
  DesiredSpeed_Setpoint_L = BaseSpeed_Setpoint;
  DesiredSpeed_Setpoint_R = BaseSpeed_Setpoint;
  summederror_L = summederror_R = 0;
  last_error_L = last_error_R = 0;
  server.send(200,"text/plain","OK - Forward");
}

// 后退：两个 motorDirection = -1
void handleBackward(){
  motorDirection_L = -1;
  motorDirection_R = -1;
  DesiredSpeed_Setpoint_L = BaseSpeed_Setpoint;
  DesiredSpeed_Setpoint_R = BaseSpeed_Setpoint;
  summederror_L = summederror_R = 0;
  last_error_L = last_error_R = 0;
  server.send(200,"text/plain","OK - Backward");
}

// 停止：motorDirection = 0，目标速度 = 0
void handleStop(){
  motorDirection_L = 0;
  motorDirection_R = 0;
  DesiredSpeed_Setpoint_L = 0;
  DesiredSpeed_Setpoint_R = 0;
  BaseSpeed_Setpoint      = 0;
  summederror_L = summederror_R = 0;
  last_error_L  = last_error_R  = 0;
  server.send(200,"text/plain","OK - Stopped");
}

// 设置基础速度
void handleSetSpeed(){
  String s = server.arg("value");
  if (s!="") {
    int v = s.toInt();
    if (v < 0) v = 0;
    BaseSpeed_Setpoint      = v;
    DesiredSpeed_Setpoint_L = v;
    DesiredSpeed_Setpoint_R = v;
  }
  server.send(200,"text/plain","OK");
}

// 单独设置左右目标速度
void handleSetSpeedLR(){
  if (server.hasArg("l")) {
    long lv = server.arg("l").toInt();
    if (lv < 0) lv = 0;
    DesiredSpeed_Setpoint_L = (int)lv;
  }
  if (server.hasArg("r")) {
    long rv = server.arg("r").toInt();
    if (rv < 0) rv = 0;
    DesiredSpeed_Setpoint_R = (int)rv;
  }
  server.send(200,"text/plain","OK - SpeedLR");
}

// 差速转向：delta<0 左转，delta>0 右转
void handleSteer(){
  int delta = server.hasArg("delta") ? server.arg("delta").toInt() : 0;
  if (delta > 100) delta = 100;
  if (delta < -100) delta = -100;
  float k = delta / 100.0f;
  int l = (int)(BaseSpeed_Setpoint * (1.0f - k));
  int r = (int)(BaseSpeed_Setpoint * (1.0f + k));
  if (l < 0) l = 0;
  if (r < 0) r = 0;
  DesiredSpeed_Setpoint_L = l;
  DesiredSpeed_Setpoint_R = r;
  server.send(200,"text/plain",
              "OK - Steer delta=" + String(delta) +
              " L=" + String(l) + " R=" + String(r));
}

// 原地转：一边前进一边后退
void handlePivot(){
  String dir = server.arg("dir");
  int spd = server.hasArg("spd") ? server.arg("spd").toInt() : BaseSpeed_Setpoint;
  if (spd < 0) spd = 0;

  if (dir == "left") {
    motorDirection_L = -1;
    motorDirection_R =  1;
  } else if (dir == "right") {
    motorDirection_L =  1;
    motorDirection_R = -1;
  } else {
    server.send(400,"text/plain","usage: /pivot?dir=left|right&spd=0..");
    return;
  }

  DesiredSpeed_Setpoint_L = spd;
  DesiredSpeed_Setpoint_R = spd;
  summederror_L = summederror_R = 0;
  last_error_L  = last_error_R  = 0;

  server.send(200,"text/plain",
              "OK - Pivot " + dir + " spd=" + String(spd));
}

// PID 参数更新
void handleSetPID(){
  if (server.hasArg("kp_l")) KP_L = server.arg("kp_l").toFloat();
  if (server.hasArg("ki_l")) KI_L = server.arg("ki_l").toFloat();
  if (server.hasArg("kd_l")) KD_L = server.arg("kd_l").toFloat();
  if (server.hasArg("kp_r")) KP_R = server.arg("kp_r").toFloat();
  if (server.hasArg("ki_r")) KI_R = server.arg("ki_r").toFloat();
  if (server.hasArg("kd_r")) KD_R = server.arg("kd_r").toFloat();
  summederror_L = summederror_R = 0;
  last_error_L  = last_error_R  = 0;
  server.send(200,"text/plain","OK - PID Updated");
}

// 返回 JSON 数据给网页画图
void handleGetData(){
  String json = "{";
  json += "\"target\":"   + String(BaseSpeed_Setpoint);
  json += ",\"measured_l\":" + String(MeasuredVelocity_CountsPerSec_L,1);
  json += ",\"error_l\":"    + String(g_current_error_L,1);
  json += ",\"pwm_l\":"      + String(g_current_pwm_output_L);
  json += ",\"measured_r\":" + String(MeasuredVelocity_CountsPerSec_R,1);
  json += ",\"error_r\":"    + String(g_current_error_R,1);
  json += ",\"pwm_r\":"      + String(g_current_pwm_output_R);
  json += "}";
  server.send(200,"application/json",json);
}
