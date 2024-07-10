#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Firebase.h>
#include "EEPROM.h"
#include "DFRobot_ESP_PH.h"
#include "DS18B20.h"
#include "GravityTDS.h"
#include "FlowSensor.h"

// Replace with your network credentials
#define WIFI_SSID "Purworejo Pride Bawah"
#define WIFI_PASSWORD "polisi27"

// Replace with your Firebase project credentials
//#define FIREBASE_HOST "https://coba-react-b54a0-default-rtdb.asia-southeast1.firebasedatabase.app/"
//#define FIREBASE_AUTH "AIzaSyAzP_kMpvDUAOL7hWmSak0zGUjdTqnenJg"
#define FIREBASE_HOST "https://selasa4proj-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "AIzaSyDzWmbs9p1ZjLjI8HVPbGw8Gvr6HyLkGTs"


// Constants
#define ADC 4096.0
#define RESOLUTION 12
#define VOLTAGE 3.3
#define FLOW_RESET_INTERVAL 60000

// GPIO Pins
#define PH_PIN 1
#define TEMP_PIN 2
#define TURB_PIN 5
#define TDS_PIN 6
#define FLOW_PIN 7

// Specifications
#define FLOW_TYPE YFB5

// Sensor Objects
DFRobot_ESP_PH pH;
GravityTDS TDS;
FlowSensor flowSensor;
OneWire oneWire(TEMP_PIN);
DS18B20 temperatureSensor(&oneWire);
Firebase firebase(FIREBASE_HOST);

// Variables
unsigned long flowLastTime = 0;
float lastTemperature = 0;
unsigned long tempTimepoint = 0;
unsigned long phTimepoint = 0;
unsigned long turbTimepoint = 0;
unsigned long tdsTimepoint = 0;
unsigned long flowTimepoint = 0;

// Function Prototypes
void IRAM_ATTR count() {
  flowSensor.count();
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(32);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi");

  pH.begin();
  temperatureSensor.begin();
  temperatureSensor.setResolution(RESOLUTION);

  TDS.setAref(VOLTAGE);
  TDS.setAdcRange(ADC);
  TDS.begin();
  TDS.setKvalue(1.0);

  flowSensor.build(FLOW_TYPE, FLOW_PIN);
  flowSensor.begin(count);

  // Initialize Firebase instance with your Firebase project URL
  Firebase firebase(FIREBASE_HOST);
  firebase.json(true); // Enable JSON mode
}

void loop() {
  readSensors();
}

void readSensors() {
    static unsigned long lastUpdateTime = 0;
    const unsigned long UPDATE_INTERVAL = 1000; // Update every second

    if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = millis();

        // Update all sensor readings
        float temperature = readTemperature();
        float phValue = readPH(temperature);
        float turbidity = readTurbidity();
        float tdsValue = readTDS(temperature);
        float flowRate = readFlow();
        float volume = flowSensor.getVolume();

        // Push sensor data to Firebase
        firebase.pushFloat("/sensorData/temperature", temperature);
        firebase.pushFloat("/sensorData/ph", phValue);
        firebase.pushFloat("/sensorData/turbidity", turbidity);
        firebase.pushFloat("/sensorData/tds", tdsValue);
        firebase.pushFloat("/sensorData/flowRate", flowRate);
        firebase.pushFloat("/sensorData/volume", volume);

        // Print the formatted string to Serial
        char buffer[200];
        sprintf(buffer, "Temperature (°C): %.2f\tpH: %.2f\tTurbidity (NTU): %.2f\tTDS (ppm): %.0f\tFlow rate (L/min): %.2f\tVolume (L): %.2f",
                temperature, phValue, turbidity, tdsValue, flowRate, volume);
        Serial.println(buffer);
    }
}

float readPH(float temperature) {
  const unsigned long PH_INTERVAL = 1000;

  if (millis() - phTimepoint >= PH_INTERVAL) {
    phTimepoint = millis();

    float voltage = analogRead(PH_PIN) * ((VOLTAGE * 1000) / ADC);
    float phValue = pH.readPH(voltage, temperature);

    pH.calibration(voltage, temperature);
    return phValue;
  }
  return NAN;
}

float readTemperature() {
  const unsigned long TEMP_INTERVAL = 750;

  if (millis() - tempTimepoint >= TEMP_INTERVAL) {
    tempTimepoint = millis();
    temperatureSensor.requestTemperatures();
    lastTemperature = temperatureSensor.getTempC();
    return lastTemperature;
  }
  return NAN;
}

float readTurbidity() {
  const unsigned long TURB_INTERVAL = 1000;

  if (millis() - turbTimepoint >= TURB_INTERVAL) {
    turbTimepoint = millis();

    float voltage = 0.0;
    for (int i = 0; i < 100; i++) {
      voltage += analogRead(TURB_PIN) * (VOLTAGE / 4095.0);
    }
    voltage /= 100.0;
    voltage = (voltage - 0.0) * (4.2 - 4.11) / (3.3 - 0.0) + 4.11;
    float ntu = constrain(-1120.4 * pow(voltage, 2) + 5742.3 * voltage - 4352.9, 0.0, 300.0);
    return ntu;
  }
  return NAN;
}

float readTDS(float temperature) {
  const unsigned long TDS_INTERVAL = 1000;

  if (millis() - tdsTimepoint >= TDS_INTERVAL) {
    tdsTimepoint = millis();

    TDS.setTemperature(temperature);
    TDS.update();
    float tdsValue = TDS.getTdsValue();
    return tdsValue;
  }
  return NAN;
}

float readFlow() {
  unsigned long flowCurrentTime = millis();

  if (flowCurrentTime - flowTimepoint >= 1000) {
    flowTimepoint = flowCurrentTime;
    flowSensor.read();

    if (flowCurrentTime - flowLastTime >= FLOW_RESET_INTERVAL) {
      flowSensor.resetVolume();
      flowLastTime = flowCurrentTime;
    }
    return flowSensor.getFlowRate_m();
  }
  return NAN;
}
