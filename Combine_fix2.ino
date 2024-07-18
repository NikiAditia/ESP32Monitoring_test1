#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <EEPROM.h>
#include "DFRobot_ESP_PH.h"
#include "GravityTDS.h"
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// Replace with your network credentials
#define WIFI_SSID "cukimay"
#define WIFI_PASSWORD "tobrutgajahat"

// Replace with your Firebase project credentials
#define API_KEY "AIzaSyAzP_kMpvDUAOL7hWmSak0zGUjdTqnenJg"
#define DATABASE_URL "https://coba-react-b54a0-default-rtdb.asia-southeast1.firebasedatabase.app/"

#define USER_EMAIL "zaenul@gmail.com"
#define USER_PASSWORD "mazzen22"


// Constants
#define ADC 4096.0
#define VOLTAGE 3.3

// GPIO Pins
#define PH_PIN 1
#define TDS_PIN 6

// Specifications
#define PH_INTERVAL 1000  // Interval in milliseconds for pH sensor reading
#define TDS_INTERVAL 1000 // Interval in milliseconds for TDS sensor reading

// Sensor Objects
DFRobot_ESP_PH pH;
GravityTDS TDS;
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variables
unsigned long phTimepoint = 0;
unsigned long tdsTimepoint = 0;
String uid;
// String databasePath = "/sensorData";
String databasePath;
// String databasePath = "/UsersData/" + uid + "/readings";
String parentPath;

void IRAM_ATTR count() {
  // Not used for pH and TDS sensors
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

  // Setup Firebase
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
  config.max_token_generation_retry = 5;
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Initialize pH sensor
  pH.begin();

  // Initialize TDS sensor
  TDS.setAref(VOLTAGE);
  TDS.setAdcRange(ADC);
  TDS.setKvalue(1.0);
  TDS.begin();

    // Update database path
  databasePath = "/UsersData/" + uid + "/readings";
}

void loop() {
  readSensors();
}

void readSensors() {
  static unsigned long lastUpdateTime = 0;
  const unsigned long UPDATE_INTERVAL = 5000; // Update every second

  if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = millis();

    // Update pH and TDS readings
    float phValue = readPH();
    float tdsValue = readTDS();

    // Push sensor data to Firebase
    String parentPath = databasePath + "/" + String(millis());

    FirebaseJson json;
    json.set("/ph", phValue);
    json.set("/tds", tdsValue);

    if (Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json)) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Failed to send data: " + fbdo.errorReason());
    }

    // Print the formatted string to Serial
    char buffer[200];
    sprintf(buffer, "pH: %.2f\tTDS (ppm): %.0f", phValue, tdsValue);
    Serial.println(buffer);
  }
}

float readPH() {
  if (millis() - phTimepoint >= PH_INTERVAL) {
    phTimepoint = millis();

    float voltage = analogRead(PH_PIN) * ((VOLTAGE * 1000) / ADC);
    float temperature = 25.0; // Assuming constant temperature for pH reading
    float phValue = pH.readPH(voltage, temperature);

    pH.calibration(voltage, temperature);
    return phValue;
  }
  return NAN;
}

float readTDS() {
  if (millis() - tdsTimepoint >= TDS_INTERVAL) {
    tdsTimepoint = millis();

    float temperature = 25.0; // Assuming constant temperature for TDS reading
    TDS.setTemperature(temperature);
    TDS.update();
    float tdsValue = TDS.getTdsValue();
    return tdsValue;
  }
  return NAN;
}
