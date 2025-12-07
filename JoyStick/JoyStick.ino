#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
// #include <esp32-hal-ledc.h> 

// ===================== 用户配置区 =====================
#define INVERT_M1_ENCODER false
#define INVERT_M2_ENCODER true   // 右轮反接修正

const char *ssid = "ESP32_Joystick_Car"; // 热点名字
const char *password = "12345678";

// ===================== PID 参数 =====================
float Kp1 = 3.0, Ki1 = 0.0, Kd1 = 0.1;
float Kp2 = 2.93, Ki2 = 0.0, Kd2 = 0.1;

// 最大速度限制 (RPM)
const float MAX_SPEED_RPM = 60.0;

// ===================== 全局参数 =====================
const float COUNTS_PER_REV = 4480.0;
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int SAMPLE_TIME_MS = 100; 

volatile float target_rpm_m1 = 0.0;
volatile float target_rpm_m2 = 0.0;

float current_rpm1 = 0;
float current_rpm2 = 0;
int pwm1 = 0;
int pwm2 = 0;

// ===================== PID 类定义 =====================
class SimplePID {
  public:
    float kp, ki, kd;
    float integral = 0;
    float prev_error = 0;
    
    SimplePID(float p, float i, float d) : kp(p), ki(i), kd(d) {}

    int compute(float target, float current, float dt) {
      float error = target - current;
      integral += error * dt;
      integral = constrain(integral, -255.0, 255.0); 
      float derivative = (error - prev_error) / dt;
      prev_error = error;
      float output = (kp * error) + (ki * integral) + (kd * derivative);
      return (int)constrain(output, -255, 255);
    }
    
    void reset() {
      integral = 0;
      prev_error = 0;
    }
};

SimplePID pid1(Kp1, Ki1, Kd1);
SimplePID pid2(Kp2, Ki2, Kd2);

// ===================== 电机定义 =====================
const int M1_ENCODER_A_PIN = 5;
const int M1_ENCODER_B_PIN = 4;
const int M1_RPWM_PIN = 7;
const int M1_LPWM_PIN = 6;
volatile long M1_duration = 0;
volatile int M1_encoder0PinALast = LOW;

const int M2_ENCODER_A_PIN = 18;
const int M2_ENCODER_B_PIN = 19;
const int M2_RPWM_PIN = 0;
const int M2_LPWM_PIN = 1;
volatile long M2_duration = 0;
volatile int M2_encoder0PinALast = LOW;

WebServer server(80);

// ===================== 网页代码 (修改：白色背景、中文修复、小摇杆) =====================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <meta charset="utf-8"> <!-- 显式声明编码 -->
  <title>ESP32 Joystick</title>
  <style>
    /* 1. 背景改为白色，文字改为深色 */
    body { font-family: 'Microsoft YaHei', Arial, sans-serif; text-align: center; margin:0; padding:0; background: #ffffff; color: #333; overflow: hidden; touch-action: none;}
    h2 { margin: 15px 0; font-size: 1.4rem; color: #444; }
    
    /* 2. 摇杆容器尺寸缩小 (300 -> 200) */
    #joystick-container {
      position: relative;
      width: 200px;
      height: 200px;
      margin: 20px auto;
      background: #f0f0f0; /* 浅灰底色 */
      border-radius: 50%;
      border: 2px solid #ccc; /* 灰色边框 */
      box-shadow: inset 0 0 10px rgba(0,0,0,0.1);
    }
    
    /* 3. 摇杆头尺寸缩小 (80 -> 60) */
    #joystick-knob {
      position: absolute;
      width: 60px;
      height: 60px;
      background: #007bff; /* 蓝色 */
      border-radius: 50%;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      box-shadow: 0 4px 10px rgba(0,0,0,0.3);
      cursor: pointer;
    }
    
    .data-box { display: flex; justify-content: space-around; margin-top: 30px; font-size: 1rem; }
    .val { font-weight: bold; font-family: monospace; font-size: 1.3rem; }
    
    /* 调整文字颜色以适应白底 */
    .lbl-l { color: #d32f2f; }
    .lbl-r { color: #1976d2; }
  </style>
</head>
<body>
  <h2>🕹️ 虚拟摇杆控制</h2>
  
  <div id="joystick-container">
    <div id="joystick-knob"></div>
  </div>

  <div class="data-box">
    <div class="lbl-l">左轮 RPM: <span id="rpm1" class="val">0</span></div>
    <div class="lbl-r">右轮 RPM: <span id="rpm2" class="val">0</span></div>
  </div>
  <p style="color:#888; font-size:0.9rem; margin-top: 20px;">拖动蓝球控制方向，松手停止</p>

<script>
  var container = document.getElementById("joystick-container");
  var knob = document.getElementById("joystick-knob");
  
  // 4. 重新计算最大半径
  // 容器半径(100) - 摇杆半径(30) = 70
  var maxRadius = 70; 
  
  var rect = container.getBoundingClientRect();
  var centerX = rect.width / 2;
  var centerY = rect.height / 2;
  var isDragging = false;
  var lastSendTime = 0;

  // 触摸/鼠标事件监听
  knob.addEventListener("mousedown", startDrag);
  knob.addEventListener("touchstart", startDrag);
  document.addEventListener("mousemove", moveDrag);
  document.addEventListener("touchmove", moveDrag);
  document.addEventListener("mouseup", endDrag);
  document.addEventListener("touchend", endDrag);

  function startDrag(e) {
    isDragging = true;
    e.preventDefault();
  }

  function moveDrag(e) {
    if (!isDragging) return;
    e.preventDefault();
    
    var clientX = e.touches ? e.touches[0].clientX : e.clientX;
    var clientY = e.touches ? e.touches[0].clientY : e.clientY;
    
    rect = container.getBoundingClientRect();
    var x = clientX - rect.left - centerX;
    var y = clientY - rect.top - centerY;
    
    var distance = Math.sqrt(x*x + y*y);
    if (distance > maxRadius) {
      var angle = Math.atan2(y, x);
      x = Math.cos(angle) * maxRadius;
      y = Math.sin(angle) * maxRadius;
    }
    
    knob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
    
    var now = Date.now();
    if (now - lastSendTime > 100) { 
      var normX = Math.round((x / maxRadius) * 100);
      var normY = Math.round((y / maxRadius) * -100); 
      sendJoystick(normX, normY);
      lastSendTime = now;
    }
  }

  function endDrag() {
    isDragging = false;
    knob.style.transform = "translate(-50%, -50%)"; 
    sendJoystick(0, 0); 
  }

  function sendJoystick(x, y) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/joystick?x=" + x + "&y=" + y, true);
    xhr.send();
  }

  setInterval(function() {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        var json = JSON.parse(this.responseText);
        document.getElementById("rpm1").innerHTML = json.m1.toFixed(1);
        document.getElementById("rpm2").innerHTML = json.m2.toFixed(1);
      }
    };
    xhr.open("GET", "/data", true);
    xhr.send();
  }, 200);
</script>
</body>
</html>
)rawliteral";

// ===================== 中断服务 =====================
void IRAM_ATTR wheelSpeed_M1() {
  int Lstate = digitalRead(M1_ENCODER_A_PIN);
  if(Lstate != M1_encoder0PinALast) {
    int val = digitalRead(M1_ENCODER_B_PIN);
    bool dir = (val != Lstate);
    if (INVERT_M1_ENCODER) dir = !dir;
    if(dir) M1_duration++; else M1_duration--;
  }
  M1_encoder0PinALast = Lstate;
}

void IRAM_ATTR wheelSpeed_M2() {
  int Lstate = digitalRead(M2_ENCODER_A_PIN);
  if(Lstate != M2_encoder0PinALast) {
    int val = digitalRead(M2_ENCODER_B_PIN);
    bool dir = (val != Lstate);
    if (INVERT_M2_ENCODER) dir = !dir;
    if(dir) M2_duration++; else M2_duration--;
  }
  M2_encoder0PinALast = Lstate;
}

// ===================== 驱动逻辑 =====================
void set_motor_pwm(int rpwm_pin, int lpwm_pin, int pwm_val) {
  if (pwm_val > 0) {
    ledcWrite(lpwm_pin, 0);          
    ledcWrite(rpwm_pin, pwm_val); 
  } else {
    ledcWrite(lpwm_pin, abs(pwm_val));          
    ledcWrite(rpwm_pin, 0); 
  }
}

// ===================== Web Handlers =====================
void handleRoot() { 
  // 关键修改：在这里强制指定 charset=utf-8，解决乱码问题
  server.send(200, "text/html; charset=utf-8", index_html); 
}

void handleJoystick() {
  if (server.hasArg("x") && server.hasArg("y")) {
    int x = server.arg("x").toInt(); 
    int y = server.arg("y").toInt(); 
    
    float left = y + x;
    float right = y - x;

    float maxVal = max(abs(left), abs(right));
    if (maxVal > 100) {
      left = (left / maxVal) * 100;
      right = (right / maxVal) * 100;
    }

    target_rpm_m1 = (left / 100.0) * MAX_SPEED_RPM;
    target_rpm_m2 = (right / 100.0) * MAX_SPEED_RPM;

    if (x == 0 && y == 0) {
      pid1.reset();
      pid2.reset();
    }

    server.send(200, "text/plain", "OK");
  }
}

void handleData() {
  String json = "{\"m1\":" + String(current_rpm1) + ",\"m2\":" + String(current_rpm2) + "}";
  server.send(200, "application/json", json);
}

// ===================== Setup & Loop =====================
void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/joystick", handleJoystick); 
  server.on("/data", handleData);
  server.begin();

  pinMode(M1_ENCODER_A_PIN, INPUT_PULLUP); 
  pinMode(M1_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(M2_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(M2_ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A_PIN), wheelSpeed_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A_PIN), wheelSpeed_M2, CHANGE);

  ledcAttach(M1_RPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M1_LPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_RPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(M2_LPWM_PIN, PWM_FREQ, PWM_RES);
}

void loop() {
  server.handleClient();

  static unsigned long last_time = 0;
  unsigned long now = millis();
  
  if (now - last_time >= SAMPLE_TIME_MS) {
    float dt = (now - last_time) / 1000.0; 
    
    noInterrupts();
    long p1 = M1_duration; M1_duration = 0;
    long p2 = M2_duration; M2_duration = 0;
    interrupts();

    current_rpm1 = (p1 / COUNTS_PER_REV) * 60.0 / dt;
    current_rpm2 = (p2 / COUNTS_PER_REV) * 60.0 / dt;

    if (abs(target_rpm_m1) > 0.1) pwm1 = pid1.compute(target_rpm_m1, current_rpm1, dt);
    else pwm1 = 0;

    if (abs(target_rpm_m2) > 0.1) pwm2 = pid2.compute(target_rpm_m2, current_rpm2, dt);
    else pwm2 = 0;

    set_motor_pwm(M1_RPWM_PIN, M1_LPWM_PIN, pwm1);
    set_motor_pwm(M2_RPWM_PIN, M2_LPWM_PIN, pwm2);

    last_time = now;
  }
}