#include <WiFi.h>
#include <WebSocketsClient.h>

// WiFi credentials
const char* ssid = "Purworejo Pride Bawah";
const char* password = "polisi27";

// WebSocket server details
const char* host = "192.168.0.103"; // IP address of your server
const int port = 8000; // Port of your WebSocket server
const char* url = "/ws/66ba2ab6efd2eeeeb7df85e8"; // WebSocket URL endpoint, replace 'your_uid_here' with actual UID

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

    // Initialize WebSocket connection
    webSocket.begin(host, port, url);

    // Set WebSocket event handler
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    webSocket.loop(); // Keep WebSocket connection alive and process messages

    // Example of sending sensor data every 20 seconds
    static unsigned long lastSendTime = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= 5000) { // 20 seconds
        lastSendTime = currentMillis;

        // Generate random sensor data
        float tds = random(0, 1000); // TDS value between 0 and 1000
        float ph = random(0, 140) / 10.0; // pH value between 0.0 and 14.0
        float turbidity = random(0, 1000) / 100.0; // Turbidity value between 0.0 and 10.0
        float temperature = random(200, 300) / 10.0; // Temperature value between 20.0 and 30.0

        // Create JSON-like string with sensor data
        String message = String("{\"TDS\":") + String(tds) +
                         String(",\"pH\":") + String(ph) +
                         String(",\"turbidity\":") + String(turbidity) +
                         String(",\"temperature\":") + String(temperature) +
                         String("}");

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
