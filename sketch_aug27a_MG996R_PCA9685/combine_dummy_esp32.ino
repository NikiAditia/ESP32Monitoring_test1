#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

const char* ssid = "Purworejo Pride Bawah";
const char* password = "polisi27";
const char* websocket_server = "192.168.0.106";
const int websocket_port = 8000;

const String uid = "66ba2ab6efd2eeeeb7df85e8";

String url;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

WebSocketsClient webSocket;

int servoMin = 140;
int servoMax = 680;
int currentAngle = 0;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;

// Status switch
bool switch1 = false; // Initial state
bool switch2 = false; // Initial state
bool switch3 = false; // Initial state
bool switch4 = false; // Initial state
bool switch5 = false; // Initial state
bool switch6 = false; // Initial state

// Previous status
bool prevSwitch1 = false;
bool prevSwitch2 = false;
bool prevSwitch3 = false;
bool prevSwitch4 = false;
bool prevSwitch5 = false;
bool prevSwitch6 = false;

void setup() {
  pwm.begin();
  pwm.setPWMFreq(60);

  Serial.begin(115200);
  Serial.println("Servo Controller siap. Masukkan sudut antara 0 hingga 180.");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  url = "/ws/control-esp/" + uid;

  webSocket.begin(websocket_server, websocket_port, url);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();

  // Membaca nilai sudut dari Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int angle = input.toInt();

    if (angle >= 0 && angle <= 180) {
      int pulseLength = map(angle, 0, 180, servoMin, servoMax);

      // Kontrol semua 16 channel dengan panjang pulsa yang sama
      for (int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, pulseLength);
      }

      currentAngle = angle;
      updateSwitchStatus(angle > 0); 

      // Kirim status hanya jika ada perubahan
      if (hasStatusChanged()) {
        sendSwitchStatus();
        updatePreviousStatus(); // Update previous status
      }
      
      Serial.print("Servo bergerak ke ");
      Serial.print(angle);
      Serial.println(" derajat pada semua channel");
    } else {
      Serial.println("Nilai tidak valid. Masukkan sudut antara 0 hingga 180.");
    }
  }

  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();
    if (hasStatusChanged()) {
      sendSwitchStatus();
      updatePreviousStatus(); // Update previous status
    }
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Disconnected from WebSocket server");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected to WebSocket server");
      break;
    case WStype_TEXT:
      Serial.printf("Message from server: %s\n", payload);
      // Menangani perintah dari server jika diperlukan
      handleIncomingMessage(payload, length);
      break;
    default:
      break;
  }
}

void handleIncomingMessage(uint8_t *payload, size_t length) {
  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("angle")) {
    int angle = doc["angle"];
    Serial.print("Received angle: ");
    Serial.println(angle);

  // Cek apakah ada kunci boolean untuk kontrol
  if (doc.containsKey("control")) {
    bool control = doc["control"];
    Serial.print("Received control: ");
    Serial.println(control ? "true" : "false");

    if (control) {
      // Jika true, gerakkan servo ke sudut 90 derajat
      moveServoToAngle(90);
    } else {
      // Jika false, kembalikan servo ke posisi 0 derajat
      moveServoToAngle(0);
    }
  }
}

void moveServoToAngle(int angle) {
  if (angle >= 0 && angle <= 180) {
    int pulseLength = map(angle, 0, 180, servoMin, servoMax);

    // Kontrol semua 16 channel dengan panjang pulsa yang sama
    for (int channel = 0; channel < 16; channel++) {
      pwm.setPWM(channel, 0, pulseLength);
    }

    currentAngle = angle;

    Serial.print("Servo bergerak ke ");
    Serial.print(angle);
    Serial.println(" derajat pada semua channel");
  } else {
    Serial.println("Nilai tidak valid. Masukkan sudut antara 0 hingga 180.");
  }
}

void sendSwitchStatus() {
  if (webSocket.isConnected()) {
    DynamicJsonDocument doc(256);
    
    // Set switch states
    doc["uid"] = uid; 
    doc["switch1"] = switch1;
    doc["switch2"] = switch2;
    doc["switch3"] = switch3;
    doc["switch4"] = switch4;
    doc["switch5"] = switch5;
    doc["switch6"] = switch6;
    
    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(json);
    
    // Log status terkirim untuk debugging
    Serial.println("Status switch terkirim: " + json);
  }
}

void updateSwitchStatus(bool state) {
  // Update switch1 based on state (example logic)
  switch1 = state;
  // Add logic for other switches if needed
}

bool hasStatusChanged() {
  return (switch1 != prevSwitch1 || switch2 != prevSwitch2 ||
          switch3 != prevSwitch3 || switch4 != prevSwitch4 ||
          switch5 != prevSwitch5 || switch6 != prevSwitch6);
}

void updatePreviousStatus() {
  prevSwitch1 = switch1;
  prevSwitch2 = switch2;
  prevSwitch3 = switch3;
  prevSwitch4 = switch4;
  prevSwitch5 = switch5;
  prevSwitch6 = switch6;
}