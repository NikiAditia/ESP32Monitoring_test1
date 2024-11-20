#include "OTACallbacks.h"
#include <WebSerial.h>

unsigned long OTACallbacks::ota_progress_millis = 0;

void OTACallbacks::begin(AsyncWebServer &server) {
    ElegantOTA.onStart(OTACallbacks::onOTAStart);
    ElegantOTA.onProgress(OTACallbacks::onOTAProgress);
    ElegantOTA.onEnd(OTACallbacks::onOTAEnd); 
    ElegantOTA.begin(&server);
    String otaReadyMessage = "OTA Ready. Update at: http://" + WiFi.localIP().toString() + "/update";
    Serial.println(otaReadyMessage);
    WebSerial.println(otaReadyMessage);
}

void OTACallbacks::onOTAStart() {
    String startMessage = "OTA update started!";
    Serial.println(startMessage);
    WebSerial.println(startMessage);
}

void OTACallbacks::onOTAProgress(size_t current, size_t final) {
    if (millis() - ota_progress_millis > 1000) {
        ota_progress_millis = millis();
        String progressMessage = String("OTA Progress: ") + String(current) + " / " + String(final) + " bytes";
        Serial.printf("%s\n", progressMessage.c_str());
        WebSerial.printf("%s\n", progressMessage.c_str());
    }
}

void OTACallbacks::onOTAEnd(bool success) {
    String endMessage = success ? "OTA update finished successfully!" : "OTA update failed!";
    Serial.println(endMessage);
    WebSerial.println(endMessage);
}
