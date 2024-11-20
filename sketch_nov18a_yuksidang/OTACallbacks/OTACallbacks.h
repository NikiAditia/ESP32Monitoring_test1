#ifndef OTACallbacks_h
#define OTACallbacks_h

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

class OTACallbacks {
public:
    static void begin(AsyncWebServer &server);
private:
    static unsigned long ota_progress_millis;
    static void onOTAStart();
    static void onOTAProgress(size_t current, size_t final);
    static void onOTAEnd(bool success);
};

#endif
