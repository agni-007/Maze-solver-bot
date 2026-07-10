/*
  MAZE SOLVER ROBOT - COMPLETE DUAL-CORE SYSTEM
  
  Hardware:
  - 3x ToF sensors (front: -45°, 0°, +45°)
  - 2x N20 gear motors via DRV8833
  - ESP32 dev module (dual-core)
  
  PIN CONFIGURATION (from user):
  Motors:
    - IN1: 27 (LEFT FWD)
    - IN2: 26 (LEFT REV)
    - IN3: 33 (RIGHT FWD)
    - IN4: 25 (RIGHT REV)
  
  ToF & I2C:
    - XSHUT LEFT: 4
    - XSHUT CENTER: 5
    - XSHUT RIGHT: 18
    - SDA: 21
    - SCL: 22
  
  Architecture:
  - Core 0: 50 Hz PID control loop + motor drive
  - Core 1: direct WiFi access point, HTTP dashboard and telemetry
  - Persistent storage: PID values in flash via Preferences
*/

#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Preferences.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

// ============= CONFIGURATION =============

const char* ap_ssid = "MazeRobot";
const char* ap_password = "maze1234";

// ===== PIN DEFINITIONS (FROM USER) =====
#define LEFT_MOTOR_FWD 27
#define LEFT_MOTOR_REV 26
#define RIGHT_MOTOR_FWD 33
#define RIGHT_MOTOR_REV 25

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

// ToF defaults tuned for:
// - 23 cm track width
// - 14 cm bot width, so centered side clearance is about 4.5 cm per side
// - side ToFs about 2.5 cm from center and angled about 45 degrees
// Expected centered angled side reading is roughly 125-140 mm.
#define DEFAULT_TOF_TURN_THRESHOLD_MM 260
#define DEFAULT_TOF_COLLISION_THRESHOLD_MM 140
#define DEFAULT_TOF_SIDE_MAX_MM 450
#define EXPECTED_CENTER_SIDE_READING_MM 130
#define TOF_INVALID_READING_MM 8190
#define DEFAULT_SEARCH_SPEED 125
#define DEFAULT_TURN_STRENGTH 90
#define TOF_TIMEOUT_MS 40
#define TOF_TIMING_BUDGET_US 20000
#define TOF_CONTINUOUS_PERIOD_MS 25
#define TURN_MIN_DURATION_MS 250
#define TURN_MAX_DURATION_MS 1200
#define TURN_CLEAR_MARGIN_MM 80

// ESP32 PWM setup for DRV8833 motor inputs
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RESOLUTION 8
#define LEFT_FWD_CH 0
#define LEFT_REV_CH 1
#define RIGHT_FWD_CH 2
#define RIGHT_REV_CH 3

// ============= GLOBAL STATE =============

// PID tuning parameters (volatile for live updates)
volatile float Kp = 1.2;
volatile float Ki = 0.0;
volatile float Kd = 0.8;
volatile float base_speed = 180;
volatile float search_speed = DEFAULT_SEARCH_SPEED;
volatile float turn_threshold_mm = DEFAULT_TOF_TURN_THRESHOLD_MM;
volatile float collision_threshold_mm = DEFAULT_TOF_COLLISION_THRESHOLD_MM;
volatile float side_max_mm = DEFAULT_TOF_SIDE_MAX_MM;
volatile float turn_strength = DEFAULT_TURN_STRENGTH;

// Control state
float pid_integral = 0.0;
float pid_prev_error = 0.0;
int8_t active_turn_direction = 0;
uint32_t active_turn_started_ms = 0;

// Sensor readings (updated by Core 0)
volatile int16_t tof_left_mm = 200;
volatile int16_t tof_center_mm = 200;
volatile int16_t tof_right_mm = 200;
volatile bool tof_left_valid = false;
volatile bool tof_center_valid = false;
volatile bool tof_right_valid = false;
volatile bool tof_left_fault = true;
volatile bool tof_center_fault = true;
volatile bool tof_right_fault = true;

// Telemetry
volatile float telemetry_error = 0;
volatile float telemetry_pid_output = 0;
volatile float telemetry_left_pwm = 0;
volatile float telemetry_right_pwm = 0;
volatile const char* telemetry_status = "BOOT";
volatile uint32_t loop_count = 0;
volatile uint32_t tof_timeout_count = 0;
volatile uint32_t last_sensor_update_ms = 0;

// ToF sensor objects
VL53L0X tof_left, tof_center, tof_right;
bool tof_left_ok = false;
bool tof_center_ok = false;
bool tof_right_ok = false;

// Web server
AsyncWebServer server(80);
DNSServer dns_server;

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
  Serial.print("  LEFT (-45°)... ");
  digitalWrite(TOF_LEFT_XSHUT, HIGH);
  delay(10);
  if (!tof_left.init()) {
    Serial.println("FAIL");
  } else {
    tof_left_ok = true;
    tof_left.setAddress(TOF_LEFT_ADDR);
    tof_left.setTimeout(TOF_TIMEOUT_MS);
    tof_left.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    tof_left.startContinuous(TOF_CONTINUOUS_PERIOD_MS);
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
    tof_center.setTimeout(TOF_TIMEOUT_MS);
    tof_center.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    tof_center.startContinuous(TOF_CONTINUOUS_PERIOD_MS);
    Serial.println("OK (0x51)");
  }
  
  // Bring up RIGHT sensor
  Serial.print("  RIGHT (+45°)... ");
  digitalWrite(TOF_RIGHT_XSHUT, HIGH);
  delay(10);
  if (!tof_right.init()) {
    Serial.println("FAIL");
  } else {
    tof_right_ok = true;
    tof_right.setAddress(TOF_RIGHT_ADDR);
    tof_right.setTimeout(TOF_TIMEOUT_MS);
    tof_right.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    tof_right.startContinuous(TOF_CONTINUOUS_PERIOD_MS);
    Serial.println("OK (0x52)");
  }
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

void read_tof_sensors() {
  int16_t left = tof_left_ok ? tof_left.readRangeContinuousMillimeters() : -1;
  bool left_timeout = tof_left_ok && tof_left.timeoutOccurred();
  int16_t center = tof_center_ok ? tof_center.readRangeContinuousMillimeters() : -1;
  bool center_timeout = tof_center_ok && tof_center.timeoutOccurred();
  int16_t right = tof_right_ok ? tof_right.readRangeContinuousMillimeters() : -1;
  bool right_timeout = tof_right_ok && tof_right.timeoutOccurred();

  if (left_timeout) tof_timeout_count++;
  if (center_timeout) tof_timeout_count++;
  if (right_timeout) tof_timeout_count++;

  bool left_fault = !tof_left_ok || left_timeout || left <= 0;
  bool center_fault = !tof_center_ok || center_timeout || center <= 0;
  bool right_fault = !tof_right_ok || right_timeout || right <= 0;
  float current_side_max = side_max_mm;

  bool left_valid = !left_fault && left < TOF_INVALID_READING_MM;
  bool center_valid = !center_fault && center < TOF_INVALID_READING_MM;
  bool right_valid = !right_fault && right < TOF_INVALID_READING_MM;
  left_valid = left_valid && left <= current_side_max;
  center_valid = center_valid && center <= current_side_max;
  right_valid = right_valid && right <= current_side_max;

  tof_left_valid = left_valid;
  tof_center_valid = center_valid;
  tof_right_valid = right_valid;
  tof_left_fault = left_fault;
  tof_center_fault = center_fault;
  tof_right_fault = right_fault;

  tof_left_mm = left_fault ? -1 : left;
  tof_center_mm = center_fault ? -1 : center;
  tof_right_mm = right_fault ? -1 : right;
  last_sensor_update_ms = millis();
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
    // Read the three ToF sensors
    read_tof_sensors();

    if (tof_left_fault || tof_center_fault || tof_right_fault) {
      set_motor_speed(0, 0);
      telemetry_error = 0;
      telemetry_pid_output = 0;
      telemetry_left_pwm = 0;
      telemetry_right_pwm = 0;
      telemetry_status = "TOF FAULT";
      loop_count++;
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_LOOP_MS));
      continue;
    }
    
    // ===== TURN DETECTION (front ToF) =====
    int turn_signal = 0;
    float speed_reduction = 1.0;
    float turn_threshold = turn_threshold_mm;
    float collision_threshold = collision_threshold_mm;
    int side_max = (int)side_max_mm;
    bool front_turn_zone = tof_center_valid && tof_center_mm < turn_threshold;

    if (active_turn_direction != 0) {
      uint32_t turn_elapsed = millis() - active_turn_started_ms;
      bool front_clear = !tof_center_valid || tof_center_mm > turn_threshold + TURN_CLEAR_MARGIN_MM;
      if ((turn_elapsed >= TURN_MIN_DURATION_MS && front_clear) || turn_elapsed >= TURN_MAX_DURATION_MS) {
        active_turn_direction = 0;
      }
    }

    if (active_turn_direction == 0 && front_turn_zone) {
      int left_compare = tof_left_valid ? tof_left_mm : side_max;
      int right_compare = tof_right_valid ? tof_right_mm : side_max;
      if (right_compare > left_compare + 20) {
        active_turn_direction = 1;
        active_turn_started_ms = millis();
      } else if (left_compare > right_compare + 20) {
        active_turn_direction = -1;
        active_turn_started_ms = millis();
      }
    }

    if (active_turn_direction != 0) {
      turn_signal = active_turn_direction;
      speed_reduction = 0.65;
    }

    bool search_mode = (!tof_left_valid || !tof_right_valid) && turn_signal == 0 && !front_turn_zone;
    
    if (tof_center_valid && tof_center_mm < collision_threshold && turn_signal == 0) {
      set_motor_speed(0, 0);
      telemetry_error = 0;
      telemetry_pid_output = 0;
      telemetry_left_pwm = 0;
      telemetry_right_pwm = 0;
      telemetry_status = "FRONT STOP";
      loop_count++;
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_LOOP_MS));
      continue;
    }
    
    // ===== CENTERING PID (left/right ToF error) =====
    // Positive error means the right side is more open, so steer right.
    float error = (!search_mode && turn_signal == 0 && tof_left_valid && tof_right_valid)
                    ? (tof_right_mm - tof_left_mm) / 10.0
                    : 0.0;

    if (search_mode || turn_signal != 0) {
      pid_integral = 0.0;
      pid_prev_error = 0.0;
    }
    
    pid_integral += error * (CONTROL_LOOP_MS / 1000.0);
    pid_integral = constrain(pid_integral, -50, 50);
    
    float pid_derivative = (error - pid_prev_error) / (CONTROL_LOOP_MS / 1000.0);
    pid_prev_error = error;
    
    float pid_output = Kp * error + Ki * pid_integral + Kd * pid_derivative;
    
    float steering = constrain(pid_output, -100, 100);
    
    float current_base_speed = base_speed;
    float current_search_speed = search_speed;
    float active_speed = current_base_speed;
    if (search_mode && active_speed > current_search_speed) {
      active_speed = current_search_speed;
    }
    float left_base = active_speed * speed_reduction;
    float right_base = active_speed * speed_reduction;
    
    float turn_adjustment = turn_signal * turn_strength;
    float left_motor = left_base + steering + turn_adjustment;
    float right_motor = right_base - steering - turn_adjustment;
    
    left_motor = constrain(left_motor, 0, 255);
    right_motor = constrain(right_motor, 0, 255);
    
    set_motor_speed((int)left_motor, (int)right_motor);
    
    // Store telemetry
    telemetry_error = error;
    telemetry_pid_output = pid_output;
    telemetry_left_pwm = left_motor;
    telemetry_right_pwm = right_motor;
    if (turn_signal > 0) telemetry_status = "TURN RIGHT";
    else if (turn_signal < 0) telemetry_status = "TURN LEFT";
    else telemetry_status = search_mode ? "SEARCH FWD" : "RUN";
    loop_count++;
    
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_LOOP_MS));
  }
}

// ============= WiFi & WEB SERVER (Core 1) =============

String build_telemetry_json() {
  String json;
  json.reserve(700);
  json = "{\"error\":" + String(telemetry_error, 2) +
         ",\"pid_out\":" + String(telemetry_pid_output, 1) +
         ",\"left_pwm\":" + String((int)telemetry_left_pwm) +
         ",\"right_pwm\":" + String((int)telemetry_right_pwm) +
         ",\"kp\":" + String(Kp, 2) +
         ",\"ki\":" + String(Ki, 2) +
         ",\"kd\":" + String(Kd, 2) +
         ",\"speed\":" + String((int)base_speed) +
         ",\"search\":" + String((int)search_speed) +
         ",\"turn_strength\":" + String((int)turn_strength) +
         ",\"turn\":" + String((int)turn_threshold_mm) +
         ",\"collision\":" + String((int)collision_threshold_mm) +
         ",\"side_max\":" + String((int)side_max_mm) +
         ",\"tof_left\":" + String((int)tof_left_mm) +
         ",\"tof_center\":" + String((int)tof_center_mm) +
         ",\"tof_right\":" + String((int)tof_right_mm) +
         ",\"side_target\":" + String(EXPECTED_CENTER_SIDE_READING_MM) +
         ",\"tof_left_valid\":" + String(tof_left_valid ? "true" : "false") +
         ",\"tof_center_valid\":" + String(tof_center_valid ? "true" : "false") +
         ",\"tof_right_valid\":" + String(tof_right_valid ? "true" : "false") +
         ",\"tof_left_fault\":" + String(tof_left_fault ? "true" : "false") +
         ",\"tof_center_fault\":" + String(tof_center_fault ? "true" : "false") +
         ",\"tof_right_fault\":" + String(tof_right_fault ? "true" : "false") +
         ",\"status\":\"" + String((const char*)telemetry_status) + "\"" +
         ",\"loop_count\":" + String(loop_count) +
         ",\"uptime_s\":" + String(millis() / 1000UL) +
         ",\"sensor_age_ms\":" + String(millis() - last_sensor_update_ms) +
         ",\"tof_timeouts\":" + String(tof_timeout_count) +
         ",\"wifi_clients\":" + String(WiFi.softAPgetStationNum()) + "}";
  return json;
}

bool update_tuning_value(const String& key, float value) {
  if (key == "kp") {
    Kp = constrain(value, 0.0f, 5.0f);
    prefs.putFloat("kp", Kp);
  } else if (key == "ki") {
    Ki = constrain(value, 0.0f, 1.0f);
    prefs.putFloat("ki", Ki);
  } else if (key == "kd") {
    Kd = constrain(value, 0.0f, 10.0f);
    prefs.putFloat("kd", Kd);
  } else if (key == "speed") {
    base_speed = constrain(value, 0.0f, 255.0f);
    prefs.putFloat("speed", base_speed);
  } else if (key == "search") {
    search_speed = constrain(value, 0.0f, 255.0f);
    prefs.putFloat("search", search_speed);
  } else if (key == "turn_strength") {
    turn_strength = constrain(value, 30.0f, 140.0f);
    prefs.putFloat("turn_power", turn_strength);
  } else if (key == "turn") {
    turn_threshold_mm = constrain(value, 30.0f, 600.0f);
    prefs.putFloat("turn", turn_threshold_mm);
  } else if (key == "collision") {
    collision_threshold_mm = constrain(value, 30.0f, 600.0f);
    prefs.putFloat("collide", collision_threshold_mm);
  } else if (key == "side_max") {
    side_max_mm = constrain(value, 100.0f, 2000.0f);
    prefs.putFloat("side_max", side_max_mm);
  } else {
    return false;
  }

  Serial.printf("[TUNE] %s=%.2f\n", key.c_str(), value);
  return true;
}

String get_html_dashboard() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Maze Robot PID Tuner</title>
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
          <input type="range" id="kp" min="0" max="5" step="0.1" value="1.2">
          <span class="value-display" id="kp-val">1.20</span>
        </div>
        
        <div class="control-group">
          <label>Integral (Ki)</label>
          <input type="range" id="ki" min="0" max="1" step="0.01" value="0.00">
          <span class="value-display" id="ki-val">0.00</span>
        </div>
        
        <div class="control-group">
          <label>Derivative (Kd)</label>
          <input type="range" id="kd" min="0" max="10" step="0.1" value="0.8">
          <span class="value-display" id="kd-val">0.80</span>
        </div>
        
        <div class="control-group">
          <label>Base Speed (0-255)</label>
          <input type="range" id="speed" min="0" max="255" step="5" value="180">
          <span class="value-display" id="speed-val">180</span>
        </div>

        <div class="control-group">
          <label>Search Speed (0-255)</label>
          <input type="range" id="search" min="0" max="255" step="5" value="125">
          <span class="value-display" id="search-val">125</span>
        </div>

        <div class="control-group">
          <label>Turn Strength (PWM)</label>
          <input type="range" id="turn_strength" min="30" max="140" step="5" value="90">
          <span class="value-display" id="turn_strength-val">90</span>
        </div>

        <div class="control-group">
          <label>Turn Threshold (mm)</label>
          <input type="range" id="turn" min="30" max="600" step="5" value="260">
          <span class="value-display" id="turn-val">260</span>
        </div>

        <div class="control-group">
          <label>Collision Threshold (mm)</label>
          <input type="range" id="collision" min="30" max="600" step="5" value="140">
          <span class="value-display" id="collision-val">140</span>
        </div>

        <div class="control-group">
          <label>Out-of-Range Limit (mm)</label>
          <input type="range" id="side_max" min="100" max="2000" step="25" value="450">
          <span class="value-display" id="side_max-val">450</span>
        </div>
        
        <div id="status" class="status disconnected">⚠️ Disconnected</div>
      </div>
      
      <div class="panel">
        <h2>Live Telemetry</h2>
        
        <div class="telemetry">
          <div class="telemetry-item">
            <span>Robot Status</span>
            <strong id="robot-status">BOOT</strong>
          </div>
          <div class="telemetry-item">
            <span>Left ToF (mm)</span>
            <strong id="tof-left">--</strong>
          </div>
          <div class="telemetry-item">
            <span>Center ToF (mm)</span>
            <strong id="tof-center">--</strong>
          </div>
          <div class="telemetry-item">
            <span>Right ToF (mm)</span>
            <strong id="tof-right">--</strong>
          </div>
          <div class="telemetry-item">
            <span>ToF Error (cm)</span>
            <strong id="error">0.00</strong>
          </div>
          <div class="telemetry-item">
            <span>Centered Side Target</span>
            <strong id="side-target">130</strong>
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
          <div class="telemetry-item">
            <span>Control Loop</span>
            <strong id="control-hz">-- Hz</strong>
          </div>
          <div class="telemetry-item">
            <span>ESP Uptime</span>
            <strong id="uptime">0 s</strong>
          </div>
          <div class="telemetry-item">
            <span>WiFi Clients</span>
            <strong id="wifi-clients">0</strong>
          </div>
          <div class="telemetry-item">
            <span>Sensor Update Age</span>
            <strong id="sensor-age">-- ms</strong>
          </div>
          <div class="telemetry-item">
            <span>ToF Timeouts</span>
            <strong id="tof-timeouts">0</strong>
          </div>
        </div>
        
      </div>
    </div>
  </div>

  <script>
    const status = document.getElementById('status');
    const tuneTimers = {};
    let telemetryRequestActive = false;
    let previousLoopCount = null;
    let previousLoopTime = null;

    function applyTelemetry(data) {
      try {
        const formatTof = (value, valid, fault) => {
          if (fault) return 'FAULT';
          if (!valid) return 'OUT';
          return Number(value).toFixed(0);
        };
        if (data.status !== undefined) document.getElementById('robot-status').textContent = data.status;
        if (data.tof_left !== undefined) document.getElementById('tof-left').textContent = formatTof(data.tof_left, data.tof_left_valid, data.tof_left_fault);
        if (data.tof_center !== undefined) document.getElementById('tof-center').textContent = formatTof(data.tof_center, data.tof_center_valid, data.tof_center_fault);
        if (data.tof_right !== undefined) document.getElementById('tof-right').textContent = formatTof(data.tof_right, data.tof_right_valid, data.tof_right_fault);
        if (data.side_target !== undefined) document.getElementById('side-target').textContent = data.side_target.toFixed(0);
        if (data.error !== undefined) document.getElementById('error').textContent = data.error.toFixed(2);
        if (data.pid_out !== undefined) document.getElementById('pid-out').textContent = data.pid_out.toFixed(1);
        if (data.left_pwm !== undefined) document.getElementById('left-pwm').textContent = data.left_pwm.toFixed(0);
        if (data.right_pwm !== undefined) document.getElementById('right-pwm').textContent = data.right_pwm.toFixed(0);
        if (data.uptime_s !== undefined) document.getElementById('uptime').textContent = data.uptime_s + ' s';
        if (data.wifi_clients !== undefined) document.getElementById('wifi-clients').textContent = data.wifi_clients;
        if (data.sensor_age_ms !== undefined) document.getElementById('sensor-age').textContent = data.sensor_age_ms + ' ms';
        if (data.tof_timeouts !== undefined) document.getElementById('tof-timeouts').textContent = data.tof_timeouts;
        if (data.loop_count !== undefined) {
          const now = Date.now();
          if (previousLoopCount !== null && data.loop_count >= previousLoopCount) {
            const hz = (data.loop_count - previousLoopCount) * 1000 / (now - previousLoopTime);
            document.getElementById('control-hz').textContent = hz.toFixed(1) + ' Hz';
          }
          previousLoopCount = data.loop_count;
          previousLoopTime = now;
        }
        ['kp', 'ki', 'kd', 'speed', 'search', 'turn_strength', 'turn', 'collision', 'side_max'].forEach(id => {
          if (data[id] !== undefined) {
            const slider = document.getElementById(id);
            const display = document.getElementById(id + '-val');
            const value = Number(data[id]);
            if (slider && document.activeElement !== slider) slider.value = value;
            if (display) display.textContent = value.toFixed((id === 'kp' || id === 'ki' || id === 'kd') ? 2 : 0);
          }
        });
        
      } catch (e) {
        console.log('Telemetry update failed', e);
      }
    }

    async function pollTelemetry() {
      if (telemetryRequestActive) return;
      telemetryRequestActive = true;
      try {
        const response = await fetch('/telemetry?t=' + Date.now(), { cache: 'no-store' });
        if (!response.ok) throw new Error('Telemetry request failed');
        applyTelemetry(await response.json());
        status.textContent = 'Connected directly';
        status.className = 'status connected';
      } catch (e) {
        status.textContent = 'Robot disconnected';
        status.className = 'status disconnected';
      } finally {
        telemetryRequestActive = false;
      }
    }

    pollTelemetry();
    setInterval(pollTelemetry, 500);
    
    ['kp', 'ki', 'kd', 'speed', 'search', 'turn_strength', 'turn', 'collision', 'side_max'].forEach(id => {
      const slider = document.getElementById(id);
      const display = document.getElementById(id + '-val');
      
      slider.addEventListener('input', (e) => {
        const value = parseFloat(e.target.value);
        const decimalPlaces = (id === 'kp' || id === 'ki' || id === 'kd') ? 2 : 0;
        display.textContent = value.toFixed(decimalPlaces);
        
        clearTimeout(tuneTimers[id]);
        tuneTimers[id] = setTimeout(async () => {
          const url = '/tune?key=' + encodeURIComponent(id) + '&value=' + encodeURIComponent(value);
          try { await fetch(url, { cache: 'no-store' }); }
          catch (e) { status.textContent = 'Tune update failed'; }
        }, 150);
      });
    });
  </script>
</body>
</html>
  )rawliteral";
}

void core1_web_server(void* param) {
  Serial.println("\n[Core 1] Direct dashboard access point starting...");
  
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  IPAddress ap_ip(192, 168, 4, 1);
  IPAddress ap_subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ap_ip, ap_ip, ap_subnet);
  bool ap_started = WiFi.softAP(ap_ssid, ap_password, 6, false, 4);
  Serial.printf("  Network: %s\n", ap_ssid);
  Serial.printf("  Password: %s\n", ap_password);
  Serial.print("  Dashboard: http://");
  Serial.println(WiFi.softAPIP());
  if (!ap_started) Serial.println("  WARNING: access point failed to start");
  dns_server.start(53, "*", WiFi.softAPIP());
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(200, "text/html", get_html_dashboard());
    response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    request->send(response);
  });
  
  server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", build_telemetry_json());
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });

  server.on("/tune", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!request->hasParam("key") || !request->hasParam("value")) {
      request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing key or value\"}");
      return;
    }

    String key = request->getParam("key")->value();
    float value = request->getParam("value")->value().toFloat();
    if (!update_tuning_value(key, value)) {
      request->send(400, "application/json", "{\"ok\":false,\"error\":\"unknown key\"}");
      return;
    }
    request->send(200, "application/json", "{\"ok\":true}");
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    request->redirect("http://192.168.4.1/");
  });
  
  server.begin();
  Serial.println("[Web] Direct dashboard server started");

  while (1) {
    dns_server.processNextRequest();
    vTaskDelay(pdMS_TO_TICKS(10));
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
  Kp = prefs.getFloat("kp", 1.2);
  Ki = prefs.getFloat("ki", 0.0);
  Kd = prefs.getFloat("kd", 0.8);
  base_speed = prefs.getFloat("speed", 180);
  search_speed = prefs.getFloat("search", DEFAULT_SEARCH_SPEED);
  turn_strength = prefs.getFloat("turn_power", DEFAULT_TURN_STRENGTH);
  turn_threshold_mm = prefs.getFloat("turn", DEFAULT_TOF_TURN_THRESHOLD_MM);
  collision_threshold_mm = prefs.getFloat("collide", DEFAULT_TOF_COLLISION_THRESHOLD_MM);
  side_max_mm = prefs.getFloat("side_max", DEFAULT_TOF_SIDE_MAX_MM);
  
  Serial.println("\n[Flash] Loaded tuning values:");
  Serial.printf("  Kp=%.2f  Ki=%.2f  Kd=%.2f  Speed=%.0f  Search=%.0f  TurnStrength=%.0f\n", Kp, Ki, Kd, base_speed, search_speed, turn_strength);
  Serial.printf("  Turn=%.0f mm  Collision=%.0f mm  OutLimit=%.0f mm\n", turn_threshold_mm, collision_threshold_mm, side_max_mm);
  
  // Initialize hardware
  Serial.println("\n[Hardware] Initializing...");
  init_i2c();
  init_tof_sensors();
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
