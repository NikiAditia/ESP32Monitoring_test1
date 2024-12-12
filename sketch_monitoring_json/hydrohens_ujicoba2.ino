#include <TDS_Gravity.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include "DS18B20.h"
#include <FlowSensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include "OTACallbacks.h"
#include "freertos/semphr.h"
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#define DEBUG_ESP_PORT Serial
#define DEBUG_ESP_HTTP_SERVER

Adafruit_ADS1115 ads;
const uint8_t ADS_ADDR = 0x4A;

#define PH_SENSOR_PIN 0

#define TDS_SENSOR_PIN 2
TDS_Gravity tds;

#define TURB_SENSOR_PIN 1

#define ONE_WIRE_BUS 13
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 temperatureSensor(&oneWire);

const int triggerPin = 32;
const int echoPin = 18;
const int triggerSelectorPins[4] = {25, 26, 27, 33};
const int echoSelectorPins[4] = {4, 16, 17, 5};
const int totalDistanceSensor = 5;
float distanceValues[totalDistanceSensor] = {0, 0, 0, 0, 0};

#define FLOW_SENSOR_PIN 36
FlowSensor sensorYFB5(YFB5, FLOW_SENSOR_PIN);
void IRAM_ATTR count() {
    sensorYFB5.count();
}

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x69);
int servoMin = 120;  
int servoMax = 600;

const int servoMinPulse = 120;
const int servoMaxPulse = 600;

// Maximum number of sensors and servos
const int ACTIVE_SERVO = 4; // Current number of active sensors/servos

// Arrays for margins and degrees
int offDegrees[ACTIVE_SERVO + 1] = {85, 85, 95, 95, 5};   // Degrees for "on" position
int onDegrees[ACTIVE_SERVO + 1] = {5, 5, 5, 15, 90};        // Degrees for "off" position

const float BOTTOM_MARGIN_TANK = 20.0; // Bottom margin for distance in cm
const float UPPER_MARGIN_TANK = 7.0;   // Upper margin for distance in cm
const float BOTTOM_MARGIN_DRINKER = 9.5; // Bottom margin for drinker in cm
const float UPPER_MARGIN_DRINKER = 6.0;   // Upper margin for drinker in cm


const char* ssid = "paku";
const char* password = "duapuluhdua";

// WebSocket server details
const char* websocket_server = "192.168.137.219";
const int websocket_port = 8000;

const String uid = "66ba2ab6efd2eeeeb7df85e8";

String url;

WebSocketsClient webSocket;

AsyncWebServer server(80);

LiquidCrystal_I2C lcdDisplay(0x27, 20, 4);

#define RELAY_POWER_PIN 21
#define RELAY_MODE_PIN 19
#define RELAY_PUMP_PIN 15
#define SWITCH_GPIO_PIN 34

SemaphoreHandle_t modeMutex;
SemaphoreHandle_t sensorDataMutex;

const char* currentModeString = "Manual Mode";  // Default to Manual mode
// Variabel global untuk menyimpan status mode yang akan dikirim
String currentModeStringSend;
String previousMode = "";  // Menyimpan mode sebelumnya, awalnya kosong

// Define a global array to store the status of each servo (0 = off, 1 = on)
int servoStatus[ACTIVE_SERVO] = {0}; // Initialize all to 0 (off)

void setupWiFi() {
    Serial.println("[WiFi] Connecting to WiFi...\n");
    WiFi.begin(ssid, password);
    unsigned long startAttemptTime = millis();
    // while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("[WiFi] Connecting...\n");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WiFi] Connected successfully\n");
        Serial.printf("[WiFi] IP Address: %s\n", WiFi.localIP().toString().c_str());

        // Initialize WebSerial and attach to the server
        WebSerial.begin(&server);
        WebSerial.onMessage(webSerialCallback);
        Serial.println("[WebSerial] Initialized and ready.");

        // Initialize ElegantOTA
        OTACallbacks::begin(server);
        Serial.println("[Setup] ElegantOTA initialized successfully.");
        server.begin();
    } else {
        Serial.println("[WiFi] Connection failed.\n");
    }
    delay(1000);
}

// Variables to store sensor values
float Temperature = 0, measuredPH = 0, measuredTDS = 0, measuredTurb = 0, volumeL = 0, flowRate = 0;
// Variabel untuk menyimpan status servo sebelumnya
bool previousServoStatus[ACTIVE_SERVO] = {false};

void taskReadSensors(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const float ADC_CONVERSION_FACTOR = 0.125 / 1000;
  for (;;) {
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
      unsigned long startTime = millis(); // Debug logging start time
      temperatureSensor.requestTemperatures();
      Temperature = temperatureSensor.getTempC();

      float phVoltage = ads.readADC_SingleEnded(PH_SENSOR_PIN) * ADC_CONVERSION_FACTOR;
      measuredPH = -0.0007 * ads.readADC_SingleEnded(PH_SENSOR_PIN) + 15.506;

      float tdsVoltage = ads.readADC_SingleEnded(TDS_SENSOR_PIN) * ADC_CONVERSION_FACTOR;
      measuredTDS = 0.0356 * ads.readADC_SingleEnded(TDS_SENSOR_PIN) + 9.4055;

      float turbVoltage = ads.readADC_SingleEnded(TURB_SENSOR_PIN) * ADC_CONVERSION_FACTOR;
      measuredTurb = -0.0318 * ads.readADC_SingleEnded(TURB_SENSOR_PIN) + 700.97;

      sensorYFB5.read();
      volumeL = sensorYFB5.getVolume();
      flowRate = sensorYFB5.getFlowRate_m();

      xSemaphoreGive(sensorDataMutex);    
    }
    vTaskDelayUntil(&xLastWakeTime, 4000 / portTICK_PERIOD_MS); // Delay 4 seconds
  }
}

void taskReadDistance(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
      unsigned long startTime = millis(); // Debug logging start time
      for (int sensorIndex = 0; sensorIndex < totalDistanceSensor; sensorIndex++) {
        selectSensor(sensorIndex);
        sendTrigPulse();

        long duration = pulseIn(echoPin, HIGH, 300000);  // Adjusted timeout
        if (duration == 0) {
            Serial.printf("[ReadDistanceTask] Sensor %d: Timeout. Retrying...", sensorIndex + 1);
            sendTrigPulse();
            duration = pulseIn(echoPin, HIGH, 200000);  // Retry
        }

        if (duration == 0) {
            distanceValues[sensorIndex] = -1;  // Timeout after retry
            Serial.printf("[ReadDistanceTask] Sensor %d: Timeout", sensorIndex + 1);
        } else {
            distanceValues[sensorIndex] = (duration / 2.0) * 0.0343;
        }

        Serial.printf("Sensor %d Distance: %.2f cm", sensorIndex + 1, distanceValues[sensorIndex]);
      }
      xSemaphoreGive(sensorDataMutex);

      // Debug logging for execution time
      unsigned long endTime = millis();
      Serial.printf("[ReadDistanceTask] Execution time: %lu ms", endTime - startTime);
    }
    vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_PERIOD_MS); // Delay 2 seconds
  }
}

void selectSensor(int sensorIndex) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(triggerSelectorPins[i], (sensorIndex >> i) & 0x01);
        digitalWrite(echoSelectorPins[i], (sensorIndex >> i) & 0x01);
    }
    Serial.printf("[ReadDistanceTask] Selected Sensor: %d", sensorIndex + 1);
}

void sendTrigPulse() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);  // Increased from 2 to 5 µs for more reliable triggering
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10); // Increased from 10 to 15 µs for better compatibility
  digitalWrite(triggerPin, LOW);
}

void updateServoStatus(String jsonData) {
  // Parse JSON data and update servoStatus
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, jsonData);

  if (error) {
    Serial.printf("Failed to parse JSON: %s\n", error.c_str());
    return;
  }

  // Access the "data" array in JSON
  JsonArray data = doc["data"].as<JsonArray>();

  // Take modeMutex to ensure safe update to servoStatus
  if (xSemaphoreTake(modeMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < data.size(); i++) {
      String servoId = data[i]["id"].as<String>();
      bool status = data[i]["status"];

      // Find the corresponding servo ID and update the servoStatus
      int servoIndex = -1;
      if (servoId == "servo1") {
        servoIndex = 0;
      } else if (servoId == "servo2") {
        servoIndex = 1;
      } else if (servoId == "servo3") {
        servoIndex = 2;
      } else if (servoId == "servo4") {
        servoIndex = 3;
      }

      if (servoIndex >= 0 && servoIndex < ACTIVE_SERVO) {
        if (servoStatus[servoIndex] != status) {  // Only update if different
          servoStatus[servoIndex] = status ? 1 : 0;
          Serial.printf("Servo %d status updated to %s\n", servoIndex, status ? "ON" : "OFF");
        }
      }
    }
    // Release the mutex after updating the servoStatus array
    xSemaphoreGive(modeMutex);
  }
}


void taskWaterControl(void* pvParameters) { 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    // Take the modeMutex to ensure safe access to currentModeString
    if (xSemaphoreTake(modeMutex, portMAX_DELAY) == pdTRUE) {
      // Check the currentModeString directly (this holds "Automatic" or "Manual")
      if (strcmp(currentModeString, "Automatic Mode") == 0) { 
        // Automatic mode logic
        xSemaphoreTake(sensorDataMutex, portMAX_DELAY); // Take sensor data for condition check
        Serial.println("[WaterControl] Running in automatic mode.");        
        
        for (int channel = 0; channel < ACTIVE_SERVO; channel++) {
          if (distanceValues[channel] > BOTTOM_MARGIN_DRINKER) {
              pwm.setPWM(channel, 0, map(onDegrees[channel], 0, 180, servoMin, servoMax));
              servoStatus[channel] = 1; // Set the status to "on"
              Serial.printf("[WaterControl] Servo %d ON: %.2f cm (distance > %d cm)\n",
                            channel, distanceValues[channel], BOTTOM_MARGIN_DRINKER);
          } else if (distanceValues[channel] < UPPER_MARGIN_DRINKER) {
              pwm.setPWM(channel, 0, map(offDegrees[channel], 0, 180, servoMin, servoMax));
              servoStatus[channel] = 0; // Set the status to "off"
              Serial.printf("[WaterControl] Servo %d OFF: %.2f cm (distance < %d cm)\n",
                            channel, distanceValues[channel], UPPER_MARGIN_DRINKER);
          } else {
              Serial.printf("[WaterControl] Servo %d unchanged: %.2f cm\n", channel, distanceValues[channel]);
          }
        }

        // Water level control
        float WaterLevelTank = distanceValues[4]; // Distance sensor 5 is at index 4 (zero-based indexing)
        if (WaterLevelTank > BOTTOM_MARGIN_TANK) {
          digitalWrite(RELAY_PUMP_PIN, LOW); // Turn relay ON
          Serial.println("[WaterControl] Relay ON (distance > 20 cm)");
        } else if (WaterLevelTank < UPPER_MARGIN_TANK) {
          digitalWrite(RELAY_PUMP_PIN, HIGH); // Turn relay OFF
          Serial.println("[WaterControl] Relay OFF (distance < 7 cm)");
        }

        xSemaphoreGive(sensorDataMutex);
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Allow other tasks to run smoothly
      } else {
          // Manual mode logic
          Serial.println("[WaterControl] Running in manual mode.");
          
          // Control the servos based on the servoStatus array
          for (int channel = 0; channel < ACTIVE_SERVO; channel++) {
            if (servoStatus[channel] == 1) {
              // If servoStatus[channel] is 1, move to 'onDegrees' position
              pwm.setPWM(channel, 0, map(onDegrees[channel], 0, 180, servoMin, servoMax));
              Serial.printf("[WaterControl] Manual Mode: Servo %d ON\n", channel);
            } else {
              // If servoStatus[channel] is 0, move to 'offDegrees' position
              pwm.setPWM(channel, 0, map(offDegrees[channel], 0, 180, servoMin, servoMax));
              Serial.printf("[WaterControl] Manual Mode: Servo %d OFF\n", channel);
            }

            // Check if the servo is turned on and the distance is greater than a threshold, then turn it off automatically
            if (servoStatus[channel] == 1 && distanceValues[channel] < UPPER_MARGIN_DRINKER) {
              // Automatically turn the servo off if the distance exceeds the threshold
              servoStatus[channel] = 0;  // Set status to off
              pwm.setPWM(channel, 0, map(offDegrees[channel], 0, 180, servoMin, servoMax));  // Move to off position
              Serial.printf("[WaterControl] Manual Mode: Servo %d OFF automatically due to distance (%.2f cm < 6 cm).\n", 
                            channel, distanceValues[channel]);
            }
          }

          vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay to allow smooth manual control
      }

      // Release the modeMutex after checking the mode
      xSemaphoreGive(modeMutex);
    }

    // Main delay to allow other tasks to run smoothly, independent of mode
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Allow other tasks to run smoothly
  }
}


void sendModeToWebSocket(const String& mode) {
  // Membuat JSON untuk mengirim mode ke WebSocket
  StaticJsonDocument<128> jsonDoc;

  // Menambahkan UID dan tipe pesan
  jsonDoc["uid"] = uid;  // Ganti dengan UID yang sesuai
  jsonDoc["type"] = "selector";  // Menambahkan tipe pesan
  jsonDoc["selector"] = mode;  // Menambahkan mode (automatic/manual)

  // Serialize JSON ke String
  String message;
  serializeJson(jsonDoc, message);

  // Periksa apakah WebSocket terhubung sebelum mengirim data
  if (webSocket.isConnected()) {
    webSocket.sendTXT(message);  // Mengirim pesan melalui WebSocket
    Serial.println("[WebSocket] Mode data sent: " + message);
  } else {
    Serial.println("[WebSocket] Not connected. Unable to send mode data.");
  }
}

void taskChangeMode(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Read the switch state
    bool switchState = digitalRead(SWITCH_GPIO_PIN); // Read the state of the switch (ON or OFF)

    // Take the modeMutex to ensure safe access to systemMode
    if (xSemaphoreTake(modeMutex, portMAX_DELAY) == pdTRUE) {
      int systemMode = switchState ? HIGH : LOW;  // Set systemMode to HIGH if switch is ON, LOW if OFF

      // Update the currentModeString based on the systemMode
      currentModeString = (systemMode == HIGH) ? "Automatic Mode" : "Manual Mode";
      currentModeStringSend = (systemMode == HIGH) ? "automatic" : "manual";
      
      xSemaphoreGive(modeMutex);  // Release the mutex

      // Log the current mode
      digitalWrite(RELAY_MODE_PIN, switchState ? LOW : HIGH);  // Control mode indication relay
      Serial.printf("Mode: %s\n", currentModeString);  // Print the current mode to Serial

      // Hanya kirim mode jika ada perubahan
      if (currentModeStringSend != previousMode) {
        // Mode berubah, kirim mode baru ke WebSocket
        sendModeToWebSocket(currentModeStringSend);  // Fungsi untuk mengirim mode ke backend

        // Update previousMode untuk perbandingan di iterasi berikutnya
        previousMode = currentModeStringSend;
      }
    }

    // Delay for 2 seconds to avoid too frequent polling of the switch
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));  // Delay for 2 seconds
  }
}

void taskDisplayLCD(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const int refreshRow2Interval = 1000 / portTICK_PERIOD_MS; // 1 second
  const int refreshRow34Interval = 2000 / portTICK_PERIOD_MS; // 2 seconds

  String lastRow1 = "HydroHens";
  String lastRow2 = "", lastRow3 = "", lastRow4 = "";

  int currentSensorIndex = 0;
  int currentInfoIndex = 0;

  TickType_t lastRow2Update = xTaskGetTickCount();
  TickType_t lastRow34Update = xTaskGetTickCount();

  for (;;) {
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
      // Row 1: Static content (never changes)
      lcdDisplay.setCursor((20 - lastRow1.length()) / 2, 0); // Centered on row 1
      lcdDisplay.print(lastRow1);

      // Row 2: Refresh every 1 second
      if (xTaskGetTickCount() - lastRow2Update >= refreshRow2Interval) {
        String currentRow2 = String("Mode: ") + (digitalRead(SWITCH_GPIO_PIN) ? "Automatic" : "Manual   ");

        // Update main part of Row 2
        if (currentRow2 != lastRow2) {
          lcdDisplay.setCursor(0, 1);
          lcdDisplay.print(currentRow2);
          lastRow2 = currentRow2;
        }

        // Update the 20th column of Row 2
        lcdDisplay.setCursor(19, 1); // Set cursor to column 20 (0-indexed, so 19) on row 2
        if (digitalRead(RELAY_PUMP_PIN) == LOW) {
          lcdDisplay.print("P"); // Display "P" if the pump is ON
        } else {
          lcdDisplay.print(" "); // Clear the position if the pump is OFF
        }

        lastRow2Update = xTaskGetTickCount();
      }

      // Rows 3 and 4: Refresh every 2 seconds
      if (xTaskGetTickCount() - lastRow34Update >= refreshRow34Interval) {
        // Row 3 content based on sensor readings
        String currentRow3;
        switch (currentSensorIndex) {
          case 0:
            currentRow3 = String("Temp: ") + String(Temperature, 1) + " C   ";
            break;
          case 1:
            currentRow3 = String("pH: ") + String(measuredPH, 1) + "       ";
            break;
          case 2:
            currentRow3 = String("TDS: ") + String(measuredTDS, 1) + " ppm   ";
            break;
          case 3:
            currentRow3 = String("Turb: ") + String(measuredTurb, 1) + " NTU   ";
            break;
          case 4:
            currentRow3 = String("Flow: ") + String(flowRate, 1) + " L/m  ";
            break;
        }
        currentSensorIndex = (currentSensorIndex + 1) % 5;

        if (currentRow3 != lastRow3) {
          lcdDisplay.setCursor(0, 2);
          lcdDisplay.print(currentRow3);
          lastRow3 = currentRow3;
        }

        // Row 4 content based on percentage, IP, or water volume
        String currentRow4;

        if (currentInfoIndex == 0) {
          // Display IP Address
          currentRow4 = String("IP: ") + WiFi.localIP().toString();
        } else if (currentInfoIndex == 5) {
          // Display Water Volume Tank
          float WaterLevelTank = distanceValues[4]; // Sensor 5 corresponds to index 4
          float WaterVolumeTank = (-1.59 * WaterLevelTank) + 36.39; // Convert distance to volume
          currentRow4 = String("Volume: ") + String(WaterVolumeTank, 2) + " L      ";
        } else {
          // Display percentage for sensors 1 to 5 (indices 0 to 4)
          float distanceValue = distanceValues[currentInfoIndex - 1];
          float percentage = (10.0 - distanceValue) / (10.0 - 6.0) * 100.0;

          // Clamp percentage between 0 and 100 to handle out-of-range values
          if (percentage < 0) percentage = 0;
          if (percentage > 100) percentage = 100;

          currentRow4 = String("Dist ") + String(currentInfoIndex) + ": " +
                        String(percentage, 1) + "%         ";
          // currentRow4 = String("Dist ") + String(currentInfoIndex) + ": " +
          //               String(distanceValue, 2) + "cm         ";
        }

        currentInfoIndex = (currentInfoIndex + 1) % 6;

        if (currentRow4 != lastRow4) {
          lcdDisplay.setCursor(0, 3);
          lcdDisplay.print(currentRow4);
          lastRow4 = currentRow4;
        }
        lastRow34Update = xTaskGetTickCount();
      }

      xSemaphoreGive(sensorDataMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to allow other tasks to run
  }
}

void webSerialCallback(uint8_t* data, size_t len) {
    if (data == nullptr) {
        Serial.println("[WebSerial] Received null data, aborting.");
        return;
    }
    String message;
    for (size_t i = 0; i < len; i++) {
        message += (char)data[i];
    }
    Serial.printf("Received: %s", message.c_str());
}

// Perbaikan bagian WebSocket
void setupWebSocket() {
    url = "/ws/" + uid + "?source=esp32";

    webSocket.begin(websocket_server, websocket_port, url);

    // Atur event handler untuk WebSocket
    webSocket.onEvent(webSocketEvent);

    // Interval reconnect jika koneksi terputus
    webSocket.setReconnectInterval(5000);

    Serial.println("[WebSocket] Initialized successfully.");
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    String jsonData = "";  // Deklarasi jsonData di luar switch, di awal fungsi
  // String JsonData;
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("[WebSocket] Disconnected.");
            break;
        case WStype_CONNECTED:
            Serial.printf("[WebSocket] Connected to server at %s:%d\n", websocket_server, websocket_port);
            break;
        case WStype_TEXT:
            // Menerima pesan berupa JSON dan mengirimkannya ke fungsi updateServoData
            jsonData = String((char*)payload, length);  // Convert payload menjadi String
            Serial.println("Data received via WebSocket:");
            Serial.println("Pesan WebSocket diterima: " + jsonData);  // Debugging pesan
            updateServoStatus(jsonData);  // Panggil fungsi untuk memperbarui status servo
            break;
        case WStype_BIN:
            Serial.println("[WebSocket] Binary message received.");
            break;
        case WStype_ERROR:
            Serial.println("[WebSocket] Error occurred.");
            break;
        default:
            Serial.println("[WebSocket] Unhandled event.");
            break;
    }
}

void taskWebSocketHandler(void* pvParameters) {
    for (;;) {
        webSocket.loop(); // Menjaga koneksi tetap aktif
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100 ms untuk menjaga efisiensi
    }
}

void sendSensorDataToWebSocket() {
    // Membuat JSON menggunakan ArduinoJson
    StaticJsonDocument<512> jsonDoc;

    // Mendapatkan nilai sensor (gunakan mutex untuk konsistensi data)
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
        // Add message type
        jsonDoc["type"] = "monitoring";

        JsonObject main_sensors = jsonDoc.createNestedObject("main_sensors");
        main_sensors["pH"] = measuredPH;
        main_sensors["temperature"] = Temperature;
        main_sensors["TDS"] = measuredTDS;
        main_sensors["turbidity"] = measuredTurb;
        main_sensors["flow_sensor"] = flowRate;
        main_sensors["Volume"] = volumeL;

        JsonObject auxiliary_sensors = jsonDoc.createNestedObject("auxiliary_sensors");
        for (int i = 0; i < totalDistanceSensor; i++) {
            auxiliary_sensors[String("Sensor") + (i + 1)] = distanceValues[i];
        }

        xSemaphoreGive(sensorDataMutex); // Lepaskan mutex setelah data selesai digunakan
    }

    // Serialize JSON ke String
    String message;
    serializeJson(jsonDoc, message);

    // Periksa apakah WebSocket terhubung sebelum mengirim data
    if (webSocket.isConnected()) {
        webSocket.sendTXT(message);
        Serial.println("[WebSocket] Message sent: " + message);
    } else {
        Serial.println("[WebSocket] Not connected. Unable to send data.");
    }
}

void sendControlDataToWebSocket() {
    // Membuat JSON untuk data kontrol (misalnya status servo)
    StaticJsonDocument<512> jsonDoc;

    // Menambahkan UID dan tipe pesan
    jsonDoc["uid"] = uid;  // Ganti dengan UID yang sesuai
    jsonDoc["type"] = "control";  // Menambahkan tipe pesan

    // Membuat array untuk switches
    JsonArray switches = jsonDoc.createNestedArray("switches");

    // Variabel untuk menandai apakah ada perubahan
    bool hasChange = false;

    // Menambahkan status setiap servo ke dalam array switches
    for (int i = 0; i < ACTIVE_SERVO; i++) {
        JsonObject switchStatus = switches.createNestedObject();
        switchStatus["id"] = "servo" + String(i + 1); // Servo1, Servo2, ...
        
        // Mengkonversi nilai 0 menjadi false dan 1 menjadi true
        switchStatus["status"] = (servoStatus[i] == 1);  // 1 menjadi true, 0 menjadi false

        // Cek apakah status servo berubah
        if (servoStatus[i] != previousServoStatus[i]) {
            hasChange = true;  // Menandakan ada perubahan
            previousServoStatus[i] = servoStatus[i]; // Update status servo sebelumnya
        }
    }

    // Jika ada perubahan, kirim data
    if (hasChange) {
        // Serialize JSON ke String
        String message;
        serializeJson(jsonDoc, message);

        // Periksa apakah WebSocket terhubung sebelum mengirim data
        if (webSocket.isConnected()) {
            webSocket.sendTXT(message);
            Serial.println("[WebSocket] Control data sent: " + message);
        } else {
            Serial.println("[WebSocket] Not connected. Unable to send control data.");
        }
    }
}


void setup(void) {
    Serial.begin(115200);
    Serial.println("[Setup] Starting setup...");

    // Initialize relay pins
    Serial.println("[Setup] Setting RELAY Pin Mode");
    pinMode(RELAY_MODE_PIN, OUTPUT);
    pinMode(RELAY_POWER_PIN, OUTPUT);
    pinMode(RELAY_PUMP_PIN, OUTPUT);
    digitalWrite(RELAY_POWER_PIN, LOW);
    digitalWrite(RELAY_PUMP_PIN, HIGH);

    // Initialize I2C
    Wire.begin(23, 22);
    Serial.println("[Setup] I2C initialized.");

    // Initialize ADS1115
    if (!ads.begin(0x48)) {
        Serial.println("[Setup] Failed to initialize ADS1115. Please check wiring and address.");
        while (true) {
            delay(1000); // Halt the program if ADS1115 initialization fails
        }
    } else {
        Serial.println("[Setup] ADS1115 initialized successfully.");
    }
    ads.setGain(GAIN_ONE);

    // Connect to WiFi
    setupWiFi();
    Serial.println("[Setup] WiFi setup complete.");

    // Initialize LCD Display
    lcdDisplay.init();
    lcdDisplay.backlight();
    lcdDisplay.begin(20, 4);
    Serial.println("[Setup] LCD display initialized successfully.");

    // Initialize Temperature Sensor
    temperatureSensor.begin();
    temperatureSensor.setResolution(12);
    Serial.println("[Setup] Temperature Sensor initialized successfully.");

    // Initialize distance sensor pins
    for (int i = 0; i < 4; i++) {
        pinMode(triggerSelectorPins[i], OUTPUT);
        pinMode(echoSelectorPins[i], OUTPUT);
    }
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println("[Setup] Distance sensor pins set up successfully.");

    // Initialize Flow Sensor
    sensorYFB5.begin(count);
    Serial.println("[Setup] Flow sensor initialized successfully.");

    // Initialize PWM for Servos
    pwm.begin();
    pwm.setPWMFreq(60);
    Serial.println("[Setup] PWM for servos initialized successfully.");

    // Initialize SWITCH GPIO Pin
    pinMode(SWITCH_GPIO_PIN, INPUT);
    Serial.println("[Setup] SWITCH_GPIO_PIN setup successfully.");

    // **Create Semaphores (Critical Fix)**
    sensorDataMutex = xSemaphoreCreateMutex();
    modeMutex = xSemaphoreCreateMutex();
    // Validate semaphore creation
    if (sensorDataMutex == NULL || modeMutex == NULL) {
        Serial.println("[Setup] Failed to create semaphores. Check memory allocation.");
        while (true) {
            delay(1000); // Halt execution if semaphores fail to initialize
        }
    }
    Serial.println("[Setup] Semaphores created successfully.");

    Serial.printf("[Debug] Free Heap Memory: %d bytes\n", ESP.getFreeHeap());

    // Create tasks after semaphores are initialized
    Serial.println("[Setup] Creating tasks...");
    if (xTaskCreatePinnedToCore(taskReadSensors, "ReadSensorsTask", 8192, NULL, 3, NULL, 1) != pdPASS) {
        Serial.println("[Setup] Failed to create ReadSensorsTask.");
    } else {
        Serial.println("[Setup] ReadSensorsTask created successfully.");
    }

    if (xTaskCreatePinnedToCore(taskReadDistance, "ReadDistanceTask", 8192, NULL, 3, NULL, 1) != pdPASS) {
        Serial.println("[Setup] Failed to create ReadDistanceTask.");
    } else {
        Serial.println("[Setup] ReadDistanceTask created successfully.");
    }

    if (xTaskCreatePinnedToCore(taskWaterControl, "WaterControlTask", 8192, NULL, 2, NULL, 1) != pdPASS) {
        Serial.println("[Setup] Failed to create WaterControlTask.");
    } else {
        Serial.println("[Setup] WaterControlTask created successfully.");
    }

    if (xTaskCreatePinnedToCore(taskChangeMode, "ChangeModeTask", 8192, NULL, 1, NULL, 1) != pdPASS) {
        Serial.println("[Setup] Failed to create ChangeModeTask.");
    } else {
        Serial.println("[Setup] ChangeModeTask created successfully.");
    }

    if (xTaskCreatePinnedToCore(taskDisplayLCD, "DisplayLCDTask", 8192, NULL, 4, NULL, 1) != pdPASS) {
        Serial.println("[Setup] Failed to create DisplayLCDTask.");
    } else {
        Serial.println("[Setup] DisplayLCDTask created successfully.");
    }
    Serial.println("[Setup] All setup steps complete.");


    // Inisialisasi WebSocket
    setupWebSocket();

    // Buat task untuk menangani WebSocket secara asinkron
    if (xTaskCreatePinnedToCore(taskWebSocketHandler, "WebSocketHandlerTask", 4096, NULL, 1, NULL, 1) != pdPASS) {
        Serial.println("[Setup] Failed to create WebSocketHandlerTask.");
    } else {
        Serial.println("[Setup] WebSocketHandlerTask created successfully.");
    }
}

void loop() {
  // ElegantOTA loop for handling OTA updates
  ElegantOTA.loop();

  // Pengiriman data WebSocket setiap 10 detik
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= 10000) { // Setiap 10 detik
      lastSendTime = millis();
      sendSensorDataToWebSocket(); // Kirim data sensor melalui WebSocket
  }

  sendControlDataToWebSocket();

  // Check if there's incoming data from Serial
  // if (Serial.available() > 0) {
  //   String jsonData = Serial.readString();  // Read the incoming data as a string
  //   updateServoStatus(jsonData);  // Call the updateServoStatus function
  // }

  // Batch WebSerial output at regular intervals
  static unsigned long lastPrintTime = 0;
  const unsigned long printInterval = 2000; // Print every 2 seconds

  if (millis() - lastPrintTime >= printInterval) {
    lastPrintTime = millis();

    // Acquire mutex for thread-safe access to sensor data
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
      // Prepare the sensor data in a vertical format
      String sensorData = "[Sensors]\n";
      sensorData += "Temperature: " + String(Temperature, 2) + " C\n";
      sensorData += "Measured pH: " + String(measuredPH, 2) + "\n";
      sensorData += "TDS: " + String(measuredTDS, 2) + " ppm\n";
      sensorData += "Turbidity: " + String(measuredTurb, 2) + " NTU\n";
      sensorData += "Flow Rate: " + String(flowRate, 2) + " L/min\n";
      sensorData += "Volume: " + String(volumeL, 2) + " L\n";
      sensorData += "Distances:\n";
      for (int i = 0; i < totalDistanceSensor; i++) {
        sensorData += "  Sensor " + String(i + 1) + ": " + String(distanceValues[i], 2) + " cm\n";
      }

      // Print the sensor data
      WebSerial.println(sensorData);
      WebSerial.println("-----------------------------");

      // Print servoStatus array
      String servoData = "Servo Status:\n";
      for (int i = 0; i < ACTIVE_SERVO; i++) {
        servoData += "  Servo " + String(i + 1) + ": " + String(servoStatus[i]) + "\n";
      }
      WebSerial.println(servoData);
      WebSerial.println("-----------------------------");

      // Release the mutex after accessing the shared data
      xSemaphoreGive(sensorDataMutex);
    }
  }
}
