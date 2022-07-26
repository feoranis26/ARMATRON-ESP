#define FDEBUG
#define FSTORAGE_DEBUG_VERBOSE

#include <fWiFiManager.h>
void setup() {
    Serial.begin(115200);
    fWiFiManager::Start();
}

void loop() {
    if (Serial.available()) {
        if (Serial.readStringUntil('\n') == "config wifi")
            fWiFiManager::Configure();
    }
}
