#include "DFRobot_ESP_PH.h"
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

#define DEBUG_ESP_PORT Serial
#define DEBUG_ESP_HTTP_SERVER

Adafruit_ADS1115 ads;
const uint8_t ADS_ADDR = 0x4A;

#define PH_SENSOR_PIN 0
DFRobot_ESP_PH ph;

#define TDS_SENSOR_PIN 1
TDS_Gravity tds;

#define TURB_SENSOR_PIN 2

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
const int NUM_SENSORS = 16; // Maximum number of sensors/servos
const int ACTIVE_DEVICES = 4; // Current number of active sensors/servos

// Arrays for margins and degrees
int bottomMargin[NUM_SENSORS] = {9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9};    // Bottom margin for each sensor
int upperMargin[NUM_SENSORS] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};     // Upper margin for each sensor
int onDegrees[NUM_SENSORS] = {85, 85, 95, 95, 90, 85, 85, 95, 95, 90, 85, 85, 95, 95, 90, 85};   // Degrees for "on" position
int offDegrees[NUM_SENSORS] = {5, 5, 5, 15, 5, 5, 5, 5, 15, 5, 5, 5, 5, 15, 5, 5};        // Degrees for "off" position

const char* ssid = "Galaxy A50E3F7";
const char* password = "lalala123";

AsyncWebServer server(80);

LiquidCrystal_I2C lcdDisplay(0x27, 20, 4);

#define RELAY_MODE_PIN 19
#define RELAY_POWER_PIN 21
#define SWITCH_GPIO_PIN 34

SemaphoreHandle_t modeMutex;
SemaphoreHandle_t sensorDataMutex;

bool isAutomaticMode = true;

void printToSerialAndWeb(const char* format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[256];
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0) {
        Serial.println("[Error] Message formatting failed.");
        return;
    }
    if (length >= sizeof(buffer)) {
        Serial.println("[Warning] Message truncated.");
    }

    // Always print to Serial for debugging
    Serial.println(buffer);

    // Send to WebSerial without the client connection check
    WebSerial.println(buffer);
    WebSerial.flush();  // Ensure immediate transmission
    Serial.println("[Debug] Message sent to WebSerial.");
}

void setupWiFi() {
    printToSerialAndWeb("[WiFi] Connecting to WiFi...\n");
    WiFi.begin(ssid, password);
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(1000);
        printToSerialAndWeb("[WiFi] Connecting...\n");
    }
    if (WiFi.status() == WL_CONNECTED) {
        printToSerialAndWeb("[WiFi] Connected successfully\n");
        printToSerialAndWeb("[WiFi] IP Address: %s\n", WiFi.localIP().toString().c_str());

        // Initialize WebSerial and attach to the server
        WebSerial.begin(&server);
        WebSerial.onMessage(webSerialCallback);
        server.begin(); // Ensure the server is started after WebSerial
        WebSerial.println("[WebSerial] Initialized and ready.");
    } else {
        printToSerialAndWeb("[WiFi] Connection failed.\n");
    }
}

// Variables to store sensor values
float ambientTemperature = 0, measuredPH = 0, measuredTDS = 0, measuredTurb = 0, volumeL = 0, flowRate = 0;

void taskReadSensors(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const float ADC_CONVERSION_FACTOR = 0.125 / 1000;
  for (;;) {
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
      unsigned long startTime = millis(); // Debug logging start time
      ambientTemperature = temperatureSensor.getTempC();

      float phVoltage = ads.readADC_SingleEnded(PH_SENSOR_PIN) * ADC_CONVERSION_FACTOR;
      measuredPH = ph.readPH(phVoltage * 1000, ambientTemperature);

      float tdsVoltage = ads.readADC_SingleEnded(TDS_SENSOR_PIN) * ADC_CONVERSION_FACTOR;
      measuredTDS = tds.readTDSValue(tdsVoltage);

      float turbVoltage = ads.readADC_SingleEnded(TURB_SENSOR_PIN) * ADC_CONVERSION_FACTOR;
      measuredTurb = -0.03 * turbVoltage + 668.07;

      sensorYFB5.read();
      volumeL = sensorYFB5.getVolume();
      flowRate = sensorYFB5.getFlowRate_m();

      xSemaphoreGive(sensorDataMutex);

      // Debug logging for execution time
      unsigned long endTime = millis();
      printToSerialAndWeb("[ReadSensorsTask] Execution time: %lu ms", endTime - startTime);
    }

    printToSerialAndWeb("Temperature: %.2f C", ambientTemperature);
    printToSerialAndWeb("pH: %.2f", measuredPH);
    printToSerialAndWeb("TDS: %.2f ppm", measuredTDS);
    printToSerialAndWeb("Turbidity: %.2f NTU", measuredTurb);
    printToSerialAndWeb("Volume: %.2f L", volumeL);
    printToSerialAndWeb("Flow rate: %.2f L/min", flowRate);
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

          long duration = pulseIn(echoPin, HIGH, 200000);  // Adjusted timeout
          if (duration == 0) {
              printToSerialAndWeb("[ReadDistanceTask] Sensor %d: Timeout. Retrying...", sensorIndex + 1);
              sendTrigPulse();
              duration = pulseIn(echoPin, HIGH, 200000);  // Retry
          }

          if (duration == 0) {
              distanceValues[sensorIndex] = -1;  // Timeout after retry
              printToSerialAndWeb("[ReadDistanceTask] Sensor %d: Timeout", sensorIndex + 1);
          } else {
              distanceValues[sensorIndex] = (duration / 2.0) * 0.0343;
          }

          printToSerialAndWeb("Sensor %d Distance: %.2f cm", sensorIndex + 1, distanceValues[sensorIndex]);
      }
      xSemaphoreGive(sensorDataMutex);

      // Debug logging for execution time
      unsigned long endTime = millis();
      printToSerialAndWeb("[ReadDistanceTask] Execution time: %lu ms", endTime - startTime);
    }
    vTaskDelayUntil(&xLastWakeTime, 2000 / portTICK_PERIOD_MS); // Delay 2 seconds
  }
}

void selectSensor(int sensorIndex) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(triggerSelectorPins[i], (sensorIndex >> i) & 0x01);
        digitalWrite(echoSelectorPins[i], (sensorIndex >> i) & 0x01);
    }
    printToSerialAndWeb("[ReadDistanceTask] Selected Sensor: %d", sensorIndex + 1);
}

void sendTrigPulse() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(5);  // Increased from 2 to 5 µs for more reliable triggering
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(15); // Increased from 10 to 15 µs for better compatibility
  digitalWrite(triggerPin, LOW);
}


void taskWaterControl(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    if (xSemaphoreTake(modeMutex, portMAX_DELAY) == pdTRUE) {
      if (isAutomaticMode) {
        xSemaphoreTake(sensorDataMutex, portMAX_DELAY); // Take sensor data for condition check
        for (int channel = 0; channel < ACTIVE_DEVICES; channel++) {
          if (distanceValues[channel] < bottomMargin[channel]) {
            pwm.setPWM(channel, 0, map(onDegrees[channel], 0, 180, servoMin, servoMax));
          } else if (distanceValues[channel] > upperMargin[channel]) {
            pwm.setPWM(channel, 0, map(offDegrees[channel], 0, 180, servoMin, servoMax));
          }
        }
        xSemaphoreGive(sensorDataMutex);
      } else {
        // Manual mode operation
        printToSerialAndWeb("[WaterControl] Running in manual mode.");
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay to allow other tasks to run smoothly
      }
      xSemaphoreGive(modeMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to allow other tasks to run smoothly
  }
}

void taskChangeMode(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    bool switchState = digitalRead(SWITCH_GPIO_PIN);
    if (xSemaphoreTake(modeMutex, portMAX_DELAY) == pdTRUE) {
      isAutomaticMode = switchState; // Update global flag
      xSemaphoreGive(modeMutex);
    }
    const char* mode = switchState ? "Automatic Mode (Switch On)" : "Manual Mode (Switch Off)";
    digitalWrite(RELAY_MODE_PIN, switchState ? LOW : HIGH);
    printToSerialAndWeb("Mode: %s", mode);

    // Debug logging for task execution
    unsigned long startTime = millis();
    unsigned long endTime = millis();
    printToSerialAndWeb("[ChangeModeTask] Execution time: %lu ms", endTime - startTime);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000)); // Changed to 2 seconds
  }
}

void taskDisplayLCD(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int currentSensorIndex = 0;
  int currentInfoIndex = 0;
  const int displayInterval = 4000 / portTICK_PERIOD_MS; // 4 seconds
  String lastRow1 = "", lastRow2 = "", lastRow3 = "", lastRow4 = "";

  for (;;) {
    IPAddress localIP = WiFi.localIP();

    String currentRow1 = "HydroHens";
    String currentRow2 = String("Mode: ") + (digitalRead(SWITCH_GPIO_PIN) ? "Automatic" : "Manual");
    String currentRow3 = "";
    String currentRow4 = "";

    switch (currentSensorIndex) {
      case 0:
        currentRow3 = String("Temp: ") + String(ambientTemperature, 1) + " C   ";
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

    if (currentInfoIndex == 0) {
      currentRow4 = String("IP: ") + localIP.toString();
    } else {
      currentRow4 = String("Dist ") + String(currentInfoIndex) + ": " + String(distanceValues[currentInfoIndex - 1], 2) + " cm  ";
    }
    currentInfoIndex = (currentInfoIndex + 1) % 6;

    if (currentRow1 != lastRow1) {
      lcdDisplay.setCursor((20 - currentRow1.length()) / 2, 0);
      lcdDisplay.print(currentRow1);
      lastRow1 = currentRow1;
    }

    if (currentRow2 != lastRow2) {
      lcdDisplay.setCursor(0, 1);
      lcdDisplay.print(currentRow2);
      lastRow2 = currentRow2;
    }

    if (currentRow3 != lastRow3) {
      lcdDisplay.setCursor(0, 2);
      lcdDisplay.print(currentRow3);
      lastRow3 = currentRow3;
    }

    if (currentRow4 != lastRow4) {
      lcdDisplay.setCursor(0, 3);
      lcdDisplay.print(currentRow4);
      lastRow4 = currentRow4;
    }

    vTaskDelayUntil(&xLastWakeTime, displayInterval);
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
    printToSerialAndWeb("Received: %s", message.c_str());
}

void setup(void) {
    Serial.begin(115200);
    Serial.println("[Setup] Starting setup...");

    // Initialize I2C
    Serial.println("[Setup] Initializing I2C...");
    Wire.begin(23, 22);
    Serial.println("[Setup] I2C initialized.");

    // Initialize ADS1115
    /*
    if (!ads.begin(0x48)) {
        Serial.println("[Setup] Failed to initialize ADS1115. Please check wiring and address.");
        while (true) {
            delay(1000); // Halt the program if ADS1115 initialization fails
        }
    } else {
        Serial.println("[Setup] ADS1115 initialized successfully.");
    }
    ads.setGain(GAIN_ONE);
    */
    
    // Initialize LCD Display
    Serial.println("[Setup] Initializing LCD display...");
    lcdDisplay.init();
    lcdDisplay.backlight();
    lcdDisplay.begin(20, 4);
    Serial.println("[Setup] LCD display initialized successfully.");

    // Initialize relay pins
    Serial.println("[Setup] Setting RELAY Pin Mode");
    pinMode(RELAY_MODE_PIN, OUTPUT);
    pinMode(RELAY_POWER_PIN, OUTPUT);
    digitalWrite(RELAY_POWER_PIN, LOW);

    // Connect to WiFi
    Serial.println("[Setup] Connecting to WiFi...");
    setupWiFi();
    Serial.println("[Setup] WiFi setup complete.");

    // Initialize pH Sensor
    Serial.println("[Setup] Initializing pH sensor...");
    ph.begin();
    Serial.println("[Setup] pH sensor initialized successfully.");

    // Initialize Temperature Sensor
    Serial.println("[Setup] Initializing Temperature Sensor...");
    temperatureSensor.begin();
    temperatureSensor.setResolution(12);
    Serial.println("[Setup] Temperature Sensor initialized successfully.");

    // Initialize distance sensor pins
    Serial.println("[Setup] Setting up distance sensor pins...");
    for (int i = 0; i < 4; i++) {
        pinMode(triggerSelectorPins[i], OUTPUT);
        pinMode(echoSelectorPins[i], OUTPUT);
    }
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println("[Setup] Distance sensor pins set up successfully.");

    // Initialize Flow Sensor
    Serial.println("[Setup] Initializing flow sensor...");
    sensorYFB5.begin(count);
    Serial.println("[Setup] Flow sensor initialized successfully.");

    // Initialize PWM for Servos
    Serial.println("[Setup] Initializing PWM for servos...");
    pwm.begin();
    pwm.setPWMFreq(60);
    Serial.println("[Setup] PWM for servos initialized successfully.");

    // Initialize SWITCH GPIO Pin
    Serial.println("[Setup] Setting up SWITCH_GPIO_PIN...");
    pinMode(SWITCH_GPIO_PIN, INPUT);
    Serial.println("[Setup] SWITCH_GPIO_PIN setup successfully.");

    // Initialize WebSerial
    Serial.println("[Setup] Initializing WebSerial...");
    WebSerial.begin(&server);
    WebSerial.onMessage(webSerialCallback);
    Serial.println("[Setup] WebSerial initialized successfully.");

    // Initialize ElegantOTA
    Serial.println("[Setup] Initializing ElegantOTA...");
    ElegantOTA.setAutoReboot(true);
    ElegantOTA.begin(&server);
    Serial.println("[Setup] ElegantOTA initialized successfully.");

    // Initialize OTA Callbacks
    Serial.println("[Setup] Initializing OTA Callbacks...");
    OTACallbacks::begin(server);
    Serial.println("[Setup] OTA Callbacks initialized successfully.");

    // Start Web Server
    Serial.println("[Setup] Starting Web Server...");
    Serial.println("[Setup] Web Server started successfully.");
    WebSerial.println("[WebSerial Test] WebSerial initialized successfully.");
    printToSerialAndWeb("This is a very long test message. This message is intentionally long to test buffer truncation. Ensure the buffer is large enough.");

    // **Create Semaphores (Critical Fix)**
    Serial.println("[Setup] Creating semaphores...");
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
    /*
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
    */
    Serial.println("[Setup] All setup steps complete.");
}

void loop() {
  ElegantOTA.loop();
  // Empty loop, as tasks are managed by FreeRTOS
}
