#include <WiFi.h>
#include <WebServer.h>

// Define Motor Pins (Verified Layout)
const int IN1 = 27;   // Left Motor Forward
const int IN2 = 26;  // Left Motor Backward
const int IN3 = 33;  // Right Motor Forward
const int IN4 = 25;   // Right Motor Backward

// PWM Settings
const int freq = 5000;      
const int resolution = 8;   
int currentDriveSpeed = 150; 

// State tracking for back-EMF protection
int lastSpeedA = 0;
int lastSpeedB = 0;

// Set up Wi-Fi Access Point Credentials
const char* ssid = "ESP32-RC-Car";
const char* password = "12345678Password";

WebServer server(80);

// --- Function Prototypes ---
void stopAll();
void setMotors(int speedA, int speedB);
void handleRoot();
void handleAction();
void handleSpeed();

void setup() {
  Serial.begin(115200);

  ledcAttach(IN1, freq, resolution);
  ledcAttach(IN2, freq, resolution);
  ledcAttach(IN3, freq, resolution);
  ledcAttach(IN4, freq, resolution);

  stopAll();

  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/action", handleAction);
  server.on("/setspeed", handleSpeed);
  
  server.begin();
  Serial.println("Fail-Safe Live Dashboard Online!");
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no\">";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; text-align: center; background-color: #1e1e24; color: white; margin:0; padding-top: 20px; user-select: none; -webkit-user-select: none; }";
  html += ".container { display: flex; flex-direction: column; align-items: center; justify-content: center; }";
  html += ".grid { display: grid; grid-template-columns: repeat(3, 90px); grid-gap: 15px; margin-top: 20px; justify-content: center; }";
  html += ".btn { width: 90px; height: 90px; background-color: #00adb5; color: white; border-radius: 50%; border: none; font-size: 28px; font-weight: bold; cursor: pointer; display: flex; align-items: center; justify-content: center; touch-action: none; box-shadow: 0 4px #00686c; }";
  html += ".btn:active { background-color: #393e46; transform: translateY(4px); box-shadow: none; }";
  html += ".slider-container { margin: 30px 0; width: 80%; max-width: 300px; }";
  html += ".slider { width: 100%; height: 15px; border-radius: 5px; background: #393e46; outline: none; -webkit-appearance: none; }";
  html += ".slider::-webkit-slider-thumb { -webkit-appearance: none; width: 25px; height: 25px; border-radius: 50%; background: #ff2e63; cursor: pointer; }";
  html += "</style></head>";
  html += "<body>";
  html += "<div class=\"container\">";
  html += "<h2>Live Control Deck</h2>";
  
  html += "<div class=\"slider-container\">";
  html += "<p>Motor Power: <span id=\"speedVal\">" + String(currentDriveSpeed) + "</span></p>";
  html += "<input type=\"range\" min=\"90\" max=\"255\" value=\"" + String(currentDriveSpeed) + "\" class=\"slider\" id=\"speedSlider\" onchange=\"updateSpeed(this.value)\">";
  html += "</div>";

  html += "<div class=\"grid\">";
  html += "<div></div><button class=\"btn\" id=\"btn-up\">▲</button><div></div>";
  html += "<button class=\"btn\" id=\"btn-left\">◀</button><div></div><button class=\"btn\" id=\"btn-right\">▶</button>";
  html += "<div></div><button class=\"btn\" id=\"btn-down\">▼</button><div></div>";
  html += "</div>";
  html += "</div>";

  html += "<script>";
  html += "function sendCmd(direction) {";
  html += "  fetch('/action?dir=' + direction).catch(err => console.log(err));";
  html += "}";
  html += "function updateSpeed(val) {";
  html += "  document.getElementById('speedVal').innerText = val;";
  html += "  fetch('/setspeed?val=' + val);";
  html += "}";
  
  // High-compatibility touch interface wrapper
  html += "function setupMomentary(elementId, dirName) {";
  html += "  const el = document.getElementById(elementId);";
  
  // Mobile specific touch triggers
  html += "  el.addEventListener('touchstart', (e) => { e.preventDefault(); sendCmd(dirName); }, {passive: false});";
  html += "  el.addEventListener('touchend', (e) => { e.preventDefault(); sendCmd('stop'); }, {passive: false});";
  
  // Desktop backup triggers
  html += "  el.addEventListener('mousedown', () => sendCmd(dirName));";
  html += "  el.addEventListener('mouseup', () => sendCmd('stop'));";
  html += "  el.addEventListener('mouseleave', () => sendCmd('stop'));";
  html += "}";
  
  html += "setupMomentary('btn-up', 'forward');";
  html += "setupMomentary('btn-down', 'backward');";
  html += "setupMomentary('btn-left', 'left');";
  html += "setupMomentary('btn-right', 'right');";
  html += "</script>";
  
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleAction() {
  if (server.hasArg("dir")) {
    String direction = server.arg("dir");
    Serial.println("Executed: " + direction); // Prints to monitor so you can see live tracking

    if (direction == "forward") {
      setMotors(currentDriveSpeed, currentDriveSpeed);
    } else if (direction == "backward") {
      setMotors(-currentDriveSpeed, -currentDriveSpeed);
    } else if (direction == "left") {
      setMotors(-currentDriveSpeed, currentDriveSpeed); 
    } else if (direction == "right") {
      setMotors(currentDriveSpeed, -currentDriveSpeed); 
    } else if (direction == "stop") {
      stopAll();
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("val")) {
    currentDriveSpeed = server.arg("val").toInt();
    Serial.print("Base Speed Configured to: ");
    Serial.println(currentDriveSpeed);
  }
  server.send(200, "text/plain", "OK");
}

void setMotors(int speedA, int speedB) {
  speedA = constrain(speedA, -255, 255);
  speedB = constrain(speedB, -255, 255);

  bool flipA = ((speedA > 0 && lastSpeedA < 0) || (speedA < 0 && lastSpeedA > 0) || (speedA == 0 && lastSpeedA != 0));
  bool flipB = ((speedB > 0 && lastSpeedB < 0) || (speedB < 0 && lastSpeedB > 0) || (speedB == 0 && lastSpeedB != 0));

  if (flipA || flipB) {
    stopAll();
    delay(60); 
  }

  lastSpeedA = speedA;
  lastSpeedB = speedB;

  // Left Motor (A)
  if (speedA > 0) {
    ledcWrite(IN1, speedA);
    ledcWrite(IN2, 0);
  } else if (speedA < 0) {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, abs(speedA));
  } else {
    ledcWrite(IN1, 0);
    ledcWrite(IN2, 0);
  }

  // Right Motor (B)
  if (speedB > 0) {
    ledcWrite(IN3, speedB);
    ledcWrite(IN4, 0);
  } else if (speedB < 0) {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, abs(speedB));
  } else {
    ledcWrite(IN3, 0);
    ledcWrite(IN4, 0);
  }
}

void stopAll() {
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
  ledcWrite(IN3, 0);
  ledcWrite(IN4, 0);
}
