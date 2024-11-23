#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h> // Tambahkan library JSON

// WiFi credentials
const char* ssid = "paku";
const char* password = "duapuluhdua";

// WebSocket server details
const char* websocket_server = "192.168.67.219";
const int websocket_port = 8000;

const String uid = "66ba2ab6efd2eeeeb7df85e8";

String url;

WebSocketsClient webSocket;

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");

    url = "/ws/" + uid;

    webSocket.begin(websocket_server, websocket_port, url);

    // Set WebSocket event handler
    webSocket.onEvent(webSocketEvent);

    webSocket.setReconnectInterval(5000);
}

void loop() {
    webSocket.loop(); // Keep WebSocket connection alive and process messages

    // Example of sending sensor data every 20 seconds
    static unsigned long lastSendTime = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= 10000) { // 20 seconds
        lastSendTime = currentMillis;

        // Generate random sensor data
        float tds = random(0, 1000); // TDS value between 0 and 1000
        float ph = random(0, 140) / 10.0; // pH value between 0.0 and 14.0
        float turbidity = random(0, 1000) / 100.0; // Turbidity value between 0.0 and 10.0
        float temperature = random(200, 300) / 10.0; // Temperature value between 20.0 and 30.0
        float flow_sensor = random(0, 1000) / 10.0; // Flow sensor value between 0.0 and 100.0

        // Generate random auxiliary sensor data
        float sensorA = random(0, 100); // Auxiliary sensor A value between 0 and 100
        float sensorB = random(0, 100); // Auxiliary sensor B value between 0 and 100
        float sensorC = random(0, 100); // Auxiliary sensor C value between 0 and 100
        float sensorD = random(0, 100); // Auxiliary sensor D value between 0 and 100
        float sensorE = random(0, 100); // Auxiliary sensor E value between 0 and 100
        float sensorF = random(0, 100); // Auxiliary sensor D value between 0 and 100
        float sensorG = random(0, 100); // Auxiliary sensor E value between 0 and 100

        // Membuat JSON menggunakan ArduinoJson
        StaticJsonDocument<512> jsonDoc;
        JsonObject main_sensors = jsonDoc.createNestedObject("main_sensors");
        main_sensors["pH"] = ph;
        main_sensors["temperature"] = temperature;
        main_sensors["TDS"] = tds;
        main_sensors["turbidity"] = turbidity;
        main_sensors["flow_sensor"] = flow_sensor;

        JsonObject auxiliary_sensors = jsonDoc.createNestedObject("auxiliary_sensors");
        auxiliary_sensors["A"] = sensorA;
        auxiliary_sensors["B"] = sensorB;
        auxiliary_sensors["C"] = sensorC;
        auxiliary_sensors["D"] = sensorD;
        auxiliary_sensors["E"] = sensorE;
        auxiliary_sensors["F"] = sensorF;
        auxiliary_sensors["G"] = sensorG;

        // Serialize JSON to String
        String message;
        serializeJson(jsonDoc, message);

        // Send the message
        webSocket.sendTXT(message);
        Serial.println("Message sent: " + message);
    }
}

// Event handler for WebSocket
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("Disconnected!");
            break;
        case WStype_CONNECTED:
            Serial.println("Connected to server");
            break;
        case WStype_TEXT:
            Serial.print("Message received: ");
            Serial.println((char*)payload);
            break;
        case WStype_BIN:
            Serial.println("Binary message received");
            break;
        case WStype_ERROR:
            Serial.println("Error occurred");
            break;
        default:
            break;
    }
}
