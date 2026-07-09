/*
  MAZE SOLVER ROBOT - COMPLETE DUAL-CORE SYSTEM
  
  Hardware:
  - 3x ToF sensors (front: -25°, 0°, +25°)
  - 2x IR sensors (sides: perpendicular, failsafe)
  - 2x N20 gear motors via DRV8833
  - ESP32 dev module (dual-core)
  
  PIN CONFIGURATION (from user):
  Motors:
    - IN1: 27 (LEFT FWD)
    - IN2: 26 (LEFT REV)
    - IN3: 25 (RIGHT FWD)
    - IN4: 23 (RIGHT REV)
  
  ToF & I2C:
    - XSHUT LEFT: 4
    - XSHUT CENTER: 5
    - XSHUT RIGHT: 18
    - SDA: 21
    - SCL: 22
  
  IR Failsafe:
    - LEFT IR: 32 (ADC)
    - RIGHT IR: 33 (ADC)
  
  Architecture:
  - Core 0: 50 Hz PID control loop + motor drive
  - Core 1: WiFi, WebSocket server, live telemetry
  - Persistent storage: PID values in flash via Preferences
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Preferences.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

// ============= CONFIGURATION =============

const char* ssid = "Tinker Space";
const char* password = "123tinkerspace";

// ===== PIN DEFINITIONS (FROM USER) =====
#define LEFT_MOTOR_FWD 27
#define LEFT_MOTOR_REV 26
#define RIGHT_MOTOR_FWD 25
#define RIGHT_MOTOR_REV 23

#define LEFT_IR_PIN 32
#define RIGHT_IR_PIN 33

#define TOF_LEFT_XSHUT 4
#define TOF_CENTER_XSHUT 5
#define TOF_RIGHT_XSHUT 18

#define I2C_SDA 21
#define I2C_SCL 22

// ToF sensor addresses (assigned at boot)
#define TOF_LEFT_ADDR 0x50
#define TOF_CENTER_ADDR 0x51
#define TOF_RIGHT_ADDR 0x52

// Control loop frequency (Hz)
#define CONTROL_LOOP_HZ 50
#define CONTROL_LOOP_MS (1000 / CONTROL_LOOP_HZ)

// IR thresholds
#define IR_MAX_DISTANCE_CM 30
#define IR_MIN_DISTANCE_CM 5
#define IR_FAILSAFE_DISTANCE_CM 8  // Stop if walls too close

// ToF thresholds (mm)
#define TOF_TURN_THRESHOLD_MM 150
#define TOF_COLLISION_THRESHOLD_MM 80

// ESP32 PWM setup for DRV8833 motor inputs
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RESOLUTION 8
#define LEFT_FWD_CH 0
#define LEFT_REV_CH 1
#define RIGHT_FWD_CH 2
#define RIGHT_REV_CH 3

// ============= GLOBAL STATE =============

// PID tuning parameters (volatile for live updates)
volatile float Kp = 0.5;
volatile float Ki = 0.01;
volatile float Kd = 2.0;
volatile float base_speed = 150;

// Control state
float pid_integral = 0.0;
float pid_prev_error = 0.0;

// Sensor readings (updated by Core 0)
volatile float left_ir_cm = 20;
volatile float right_ir_cm = 20;
volatile int16_t tof_left_mm = 200;
volatile int16_t tof_center_mm = 200;
volatile int16_t tof_right_mm = 200;

// Telemetry
volatile float telemetry_error = 0;
volatile float telemetry_pid_output = 0;
volatile float telemetry_left_pwm = 0;
volatile float telemetry_right_pwm = 0;
volatile uint32_t loop_count = 0;

// ToF sensor objects
VL53L0X tof_left, tof_center, tof_right;
bool tof_left_ok = false;
bool tof_center_ok = false;
bool tof_right_ok = false;

// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Preferences
Preferences prefs;

// ============= MOTOR PWM COMPATIBILITY =============

void attach_pwm_pin(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
#else
  ledcSetup(channel, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(pin, channel);
#endif
}

void write_pwm_pin(uint8_t pin, uint8_t channel, int duty) {
  duty = constrain(duty, 0, 255);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void write_motor_outputs(int left_fwd, int left_rev, int right_fwd, int right_rev) {
  write_pwm_pin(LEFT_MOTOR_FWD, LEFT_FWD_CH, left_fwd);
  write_pwm_pin(LEFT_MOTOR_REV, LEFT_REV_CH, left_rev);
  write_pwm_pin(RIGHT_MOTOR_FWD, RIGHT_FWD_CH, right_fwd);
  write_pwm_pin(RIGHT_MOTOR_REV, RIGHT_REV_CH, right_rev);
}

// ============= SENSOR INITIALIZATION =============

void init_i2c() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Serial.println("[I2C] Initialized at 400 kHz");
}

void init_tof_sensors() {
  Serial.println("\n[ToF] Initializing sensors...");
  
  // Set all XSHUT pins low first
  pinMode(TOF_LEFT_XSHUT, OUTPUT);
  pinMode(TOF_CENTER_XSHUT, OUTPUT);
  pinMode(TOF_RIGHT_XSHUT, OUTPUT);
  
  digitalWrite(TOF_LEFT_XSHUT, LOW);
  digitalWrite(TOF_CENTER_XSHUT, LOW);
  digitalWrite(TOF_RIGHT_XSHUT, LOW);
  
  delay(10);
  
  // Bring up LEFT sensor
  Serial.print("  LEFT (-25°)... ");
  digitalWrite(TOF_LEFT_XSHUT, HIGH);
  delay(10);
  if (!tof_left.init()) {
    Serial.println("FAIL");
  } else {
    tof_left_ok = true;
    tof_left.setAddress(TOF_LEFT_ADDR);
    tof_left.startContinuous();
    Serial.println("OK (0x50)");
  }
  
  // Bring up CENTER sensor
  Serial.print("  CENTER (0°)... ");
  digitalWrite(TOF_CENTER_XSHUT, HIGH);
  delay(10);
  if (!tof_center.init()) {
    Serial.println("FAIL");
  } else {
    tof_center_ok = true;
    tof_center.setAddress(TOF_CENTER_ADDR);
    tof_center.startContinuous();
    Serial.println("OK (0x51)");
  }
  
  // Bring up RIGHT sensor
  Serial.print("  RIGHT (+25°)... ");
  digitalWrite(TOF_RIGHT_XSHUT, HIGH);
  delay(10);
  if (!tof_right.init()) {
    Serial.println("FAIL");
  } else {
    tof_right_ok = true;
    tof_right.setAddress(TOF_RIGHT_ADDR);
    tof_right.startContinuous();
    Serial.println("OK (0x52)");
  }
}

void init_ir_sensors() {
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  Serial.println("[IR] Failsafe sensors ready");
}

void init_motors() {
  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_REV, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_REV, OUTPUT);

  attach_pwm_pin(LEFT_MOTOR_FWD, LEFT_FWD_CH);
  attach_pwm_pin(LEFT_MOTOR_REV, LEFT_REV_CH);
  attach_pwm_pin(RIGHT_MOTOR_FWD, RIGHT_FWD_CH);
  attach_pwm_pin(RIGHT_MOTOR_REV, RIGHT_REV_CH);

  write_motor_outputs(0, 0, 0, 0);
  
  Serial.println("[Motors] Initialized and stopped");
}

// ============= SENSOR READING =============

float read_ir_distance_cm(uint8_t pin) {
  int raw = analogRead(pin);
  float voltage = raw * (3.3 / 4095.0);
  
  // Sharp IR sensor inverse mapping (adjust for your specific model)
  float distance_cm = 27.0 / (voltage - 0.1);
  distance_cm = constrain(distance_cm, IR_MIN_DISTANCE_CM, IR_MAX_DISTANCE_CM);
  
  return distance_cm;
}

void read_tof_sensors() {
  int16_t left = tof_left_ok ? tof_left.readRangeContinuousMillimeters() : 1000;
  int16_t center = tof_center_ok ? tof_center.readRangeContinuousMillimeters() : 1000;
  int16_t right = tof_right_ok ? tof_right.readRangeContinuousMillimeters() : 1000;
  
  if (tof_left_ok && tof_left.timeoutOccurred()) left = 1000;
  if (tof_center_ok && tof_center.timeoutOccurred()) center = 1000;
  if (tof_right_ok && tof_right.timeoutOccurred()) right = 1000;

  tof_left_mm = constrain(left, 0, 1000);
  tof_center_mm = constrain(center, 0, 1000);
  tof_right_mm = constrain(right, 0, 1000);
}

void read_ir_sensors() {
  left_ir_cm = read_ir_distance_cm(LEFT_IR_PIN);
  right_ir_cm = read_ir_distance_cm(RIGHT_IR_PIN);
}

// ============= MOTOR CONTROL =============

void set_motor_speed(int left_speed, int right_speed) {
  // left_speed, right_speed: -255 to +255
  // Positive = forward, negative = backward
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  int left_fwd = left_speed > 0 ? left_speed : 0;
  int left_rev = left_speed < 0 ? -left_speed : 0;
  int right_fwd = right_speed > 0 ? right_speed : 0;
  int right_rev = right_speed < 0 ? -right_speed : 0;

  write_motor_outputs(left_fwd, left_rev, right_fwd, right_rev);
}

// ============= PID CONTROL LOOP (Core 0) =============

void core0_control_loop(void* param) {
  TickType_t last_wake = xTaskGetTickCount();
  
  Serial.println("\n[Core 0] Control loop started");
  
  while (1) {
    // Read all sensors
    read_tof_sensors();
    read_ir_sensors();
    
    // ===== FAILSAFE: IR sensors =====
    if (left_ir_cm < IR_FAILSAFE_DISTANCE_CM || right_ir_cm < IR_FAILSAFE_DISTANCE_CM) {
      set_motor_speed(0, 0);
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_LOOP_MS));
      continue;
    }
    
    // ===== TURN DETECTION (front ToF) =====
    int turn_signal = 0;
    float speed_reduction = 1.0;
    
    if (tof_center_mm < TOF_TURN_THRESHOLD_MM) {
      if (tof_right_mm > tof_left_mm + 20) {
        turn_signal = 1;
        speed_reduction = 0.6;
      } else if (tof_left_mm > tof_right_mm + 20) {
        turn_signal = -1;
        speed_reduction = 0.6;
      }
    }
    
    if (tof_center_mm < TOF_COLLISION_THRESHOLD_MM) {
      set_motor_speed(0, 0);
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_LOOP_MS));
      continue;
    }
    
    // ===== CENTERING PID (side IR error) =====
    float error = right_ir_cm - left_ir_cm;
    
    pid_integral += error * (CONTROL_LOOP_MS / 1000.0);
    pid_integral = constrain(pid_integral, -50, 50);
    
    float pid_derivative = (error - pid_prev_error) / (CONTROL_LOOP_MS / 1000.0);
    pid_prev_error = error;
    
    float pid_output = Kp * error + Ki * pid_integral + Kd * pid_derivative;
    
    float steering = constrain(pid_output, -100, 100);
    
    float left_base = base_speed * speed_reduction;
    float right_base = base_speed * speed_reduction;
    
    float left_motor = left_base + steering + (turn_signal * 50);
    float right_motor = right_base - steering - (turn_signal * 50);
    
    left_motor = constrain(left_motor, 0, 255);
    right_motor = constrain(right_motor, 0, 255);
    
    set_motor_speed((int)left_motor, (int)right_motor);
    
    // Store telemetry
    telemetry_error = error;
    telemetry_pid_output = pid_output;
    telemetry_left_pwm = left_motor;
    telemetry_right_pwm = right_motor;
    loop_count++;
    
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_LOOP_MS));
  }
}

// ============= WiFi & WEB SERVER (Core 1) =============

bool extract_json_float(const String& message, const char* key, float& value) {
  String pattern = "\"" + String(key) + "\"";
  int key_pos = message.indexOf(pattern);
  if (key_pos < 0) return false;

  int colon_pos = message.indexOf(':', key_pos + pattern.length());
  if (colon_pos < 0) return false;

  int start = colon_pos + 1;
  while (start < (int)message.length() && isspace((unsigned char)message[start])) {
    start++;
  }

  int end = start;
  while (end < (int)message.length()) {
    char c = message[end];
    if (!(isDigit(c) || c == '-' || c == '+' || c == '.')) break;
    end++;
  }

  if (end == start) return false;
  value = message.substring(start, end).toFloat();
  return true;
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("[WebSocket] Client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("[WebSocket] Client disconnected");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      if (info->opcode == WS_TEXT) {
        String message;
        message.reserve(len + 1);
        for (size_t i = 0; i < len; i++) {
          message += (char)data[i];
        }
        
        // Parse small JSON messages such as {"kp":0.5} or {"speed":150}
        float parsed_value = 0.0;
        bool updated = false;
        if (extract_json_float(message, "kp", parsed_value)) {
          Kp = constrain(parsed_value, 0.0, 5.0);
          updated = true;
        }
        if (extract_json_float(message, "ki", parsed_value)) {
          Ki = constrain(parsed_value, 0.0, 1.0);
          updated = true;
        }
        if (extract_json_float(message, "kd", parsed_value)) {
          Kd = constrain(parsed_value, 0.0, 10.0);
          updated = true;
        }
        if (extract_json_float(message, "speed", parsed_value)) {
          base_speed = constrain(parsed_value, 0.0, 255.0);
          updated = true;
        }

        if (updated) {
          // Save to flash
          prefs.putFloat("kp", Kp);
          prefs.putFloat("ki", Ki);
          prefs.putFloat("kd", Kd);
          prefs.putFloat("speed", base_speed);
          
          Serial.printf("[PID] Updated: Kp=%.2f Ki=%.2f Kd=%.2f Speed=%.0f\n", Kp, Ki, Kd, base_speed);
        }
      }
    }
  }
}

String get_html_dashboard() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Maze Robot PID Tuner</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js"></script>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      min-height: 100vh;
      padding: 20px;
    }
    .container {
      max-width: 1200px;
      margin: 0 auto;
      background: white;
      border-radius: 12px;
      box-shadow: 0 20px 60px rgba(0,0,0,0.3);
      padding: 30px;
    }
    h1 {
      color: #333;
      margin-bottom: 30px;
      text-align: center;
      font-size: 28px;
    }
    .layout {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 30px;
    }
    @media (max-width: 768px) {
      .layout { grid-template-columns: 1fr; }
    }
    .panel {
      background: #f8f9fa;
      border-radius: 8px;
      padding: 20px;
      border-left: 4px solid #667eea;
    }
    .panel h2 {
      font-size: 18px;
      margin-bottom: 15px;
      color: #333;
    }
    .control-group {
      margin-bottom: 20px;
    }
    label {
      display: block;
      font-size: 12px;
      font-weight: 600;
      color: #666;
      text-transform: uppercase;
      margin-bottom: 8px;
    }
    input[type="range"] {
      width: 100%;
      height: 6px;
      border-radius: 3px;
      background: #ddd;
      outline: none;
    }
    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: #667eea;
      cursor: pointer;
    }
    input[type="range"]::-moz-range-thumb {
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: #667eea;
      cursor: pointer;
      border: none;
    }
    .value-display {
      display: inline-block;
      background: white;
      padding: 4px 8px;
      border-radius: 4px;
      font-weight: 600;
      color: #667eea;
      font-family: 'Courier New', monospace;
      margin-left: 10px;
    }
    .telemetry {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      font-size: 13px;
    }
    .telemetry-item {
      background: white;
      padding: 12px;
      border-radius: 6px;
      border-left: 3px solid #764ba2;
    }
    .telemetry-item span {
      display: block;
      color: #999;
      font-size: 11px;
      margin-bottom: 4px;
    }
    .telemetry-item strong {
      color: #333;
      font-size: 16px;
      font-family: 'Courier New', monospace;
    }
    canvas {
      margin-top: 20px;
    }
    .status {
      padding: 12px;
      border-radius: 6px;
      margin-top: 15px;
      text-align: center;
      font-weight: 600;
    }
    .status.connected {
      background: #d4edda;
      color: #155724;
    }
    .status.disconnected {
      background: #f8d7da;
      color: #721c24;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🤖 Maze Robot PID Tuner</h1>
    
    <div class="layout">
      <div class="panel">
        <h2>PID Parameters</h2>
        
        <div class="control-group">
          <label>Proportional (Kp)</label>
          <input type="range" id="kp" min="0" max="5" step="0.1" value="0.5">
          <span class="value-display" id="kp-val">0.50</span>
        </div>
        
        <div class="control-group">
          <label>Integral (Ki)</label>
          <input type="range" id="ki" min="0" max="1" step="0.01" value="0.01">
          <span class="value-display" id="ki-val">0.01</span>
        </div>
        
        <div class="control-group">
          <label>Derivative (Kd)</label>
          <input type="range" id="kd" min="0" max="10" step="0.1" value="2.0">
          <span class="value-display" id="kd-val">2.00</span>
        </div>
        
        <div class="control-group">
          <label>Base Speed (0-255)</label>
          <input type="range" id="speed" min="0" max="255" step="5" value="150">
          <span class="value-display" id="speed-val">150</span>
        </div>
        
        <div id="status" class="status disconnected">⚠️ Disconnected</div>
      </div>
      
      <div class="panel">
        <h2>Live Telemetry</h2>
        
        <div class="telemetry">
          <div class="telemetry-item">
            <span>Centering Error (cm)</span>
            <strong id="error">0.00</strong>
          </div>
          <div class="telemetry-item">
            <span>PID Output</span>
            <strong id="pid-out">0.00</strong>
          </div>
          <div class="telemetry-item">
            <span>Left PWM</span>
            <strong id="left-pwm">0</strong>
          </div>
          <div class="telemetry-item">
            <span>Right PWM</span>
            <strong id="right-pwm">0</strong>
          </div>
        </div>
        
        <canvas id="errorChart"></canvas>
      </div>
    </div>
  </div>

  <script>
    const ws = new WebSocket('ws://' + window.location.host + '/ws');
    const status = document.getElementById('status');
    
    ws.onopen = () => {
      status.textContent = '✅ Connected';
      status.className = 'status connected';
    };
    
    ws.onclose = () => {
      status.textContent = '⚠️ Disconnected';
      status.className = 'status disconnected';
    };
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        if (data.error !== undefined) document.getElementById('error').textContent = data.error.toFixed(2);
        if (data.pid_out !== undefined) document.getElementById('pid-out').textContent = data.pid_out.toFixed(1);
        if (data.left_pwm !== undefined) document.getElementById('left-pwm').textContent = data.left_pwm.toFixed(0);
        if (data.right_pwm !== undefined) document.getElementById('right-pwm').textContent = data.right_pwm.toFixed(0);
        
        if (chartData.labels.length > 50) {
          chartData.labels.shift();
          chartData.datasets[0].data.shift();
        }
        chartData.labels.push((Date.now() % 10000).toString().slice(-4));
        chartData.datasets[0].data.push(data.error || 0);
        chart.update('none');
      } catch (e) {}
    };
    
    ['kp', 'ki', 'kd', 'speed'].forEach(id => {
      const slider = document.getElementById(id);
      const display = document.getElementById(id + '-val');
      
      slider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        display.textContent = value.toFixed(id === 'speed' ? 0 : 2);
        
        const msg = {};
        msg[id] = value;
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify(msg));
        }
      });
    });
    
    const chartData = {
      labels: [],
      datasets: [{
        label: 'Centering Error (cm)',
        data: [],
        borderColor: '#667eea',
        backgroundColor: 'rgba(102, 126, 234, 0.1)',
        borderWidth: 2,
        tension: 0.3,
        fill: true
      }]
    };
    
    const chart = new Chart(document.getElementById('errorChart'), {
      type: 'line',
      data: chartData,
      options: {
        responsive: true,
        maintainAspectRatio: true,
        plugins: { legend: { display: false } },
        scales: { y: { min: -5, max: 5 }, x: { display: false } }
      }
    });
  </script>
</body>
</html>
  )rawliteral";
}

void core1_web_server(void* param) {
  Serial.println("\n[Core 1] Web server starting...");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  Serial.print("  Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("  Connected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("  WiFi failed (optional)");
  }
  
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", get_html_dashboard());
  });
  
  server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{\"error\":" + String(telemetry_error, 2) +
                  ",\"pid_out\":" + String(telemetry_pid_output, 1) +
                  ",\"left_pwm\":" + String((int)telemetry_left_pwm) +
                  ",\"right_pwm\":" + String((int)telemetry_right_pwm) +
                  ",\"loop_count\":" + String(loop_count) + "}";
    request->send(200, "application/json", json);
  });
  
  server.begin();
  Serial.println("[Web] Server started");
  
  uint32_t last_broadcast = millis();
  while (1) {
    if (ws.count() > 0 && millis() - last_broadcast > 50) {
      String json = "{\"error\":" + String(telemetry_error, 2) +
                    ",\"pid_out\":" + String(telemetry_pid_output, 1) +
                    ",\"left_pwm\":" + String((int)telemetry_left_pwm) +
                    ",\"right_pwm\":" + String((int)telemetry_right_pwm) + "}";
      ws.textAll(json);
      last_broadcast = millis();
    }
    vTaskDelay(10);
  }
}

// ============= SETUP =============

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║  MAZE SOLVER ROBOT - DUAL CORE INIT    ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Initialize preferences
  prefs.begin("maze_robot", false);
  Kp = prefs.getFloat("kp", 0.5);
  Ki = prefs.getFloat("ki", 0.01);
  Kd = prefs.getFloat("kd", 2.0);
  base_speed = prefs.getFloat("speed", 150);
  
  Serial.println("\n[Flash] Loaded PID values:");
  Serial.printf("  Kp=%.2f  Ki=%.2f  Kd=%.2f  Speed=%.0f\n", Kp, Ki, Kd, base_speed);
  
  // Initialize hardware
  Serial.println("\n[Hardware] Initializing...");
  init_i2c();
  init_tof_sensors();
  init_ir_sensors();
  init_motors();
  
  // Start dual-core tasks
  Serial.println("\n[FreeRTOS] Starting dual-core tasks...");
  
  xTaskCreatePinnedToCore(
    core0_control_loop,
    "ControlLoop",
    2048,
    NULL,
    2,
    NULL,
    0
  );
  
  xTaskCreatePinnedToCore(
    core1_web_server,
    "WebServer",
    4096,
    NULL,
    1,
    NULL,
    1
  );
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  SYSTEM RUNNING                        ║");
  Serial.println("╚════════════════════════════════════════╝\n");
}

void loop() {
  delay(10000);
}
