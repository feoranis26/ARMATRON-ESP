#pragma once
#ifndef fStorage_h
#define fStorage_h
#include <SPIFFS.h>
#include "fDebugUtils.h"
#include "fSerialParser.h"

#ifdef USE_FGUI
#include "fGUI.h"
#endif

class fStorageUtils {
public:
    static bool SaveData(String name, String data) {
        if (!spiffsMounted)
            beginSPIFFS();

        File file = SPIFFS.open("/" + name, "w", true);

        if (!file) {
            fDebugUtils::ThrowException("[fStorage] Can't open file! Name: " + name + ", data: " + data);
            return false;
        }

        int written = file.print(data);
        if (written < data.length())
            fDebugUtils::ThrowException("[fStorage] Can't save file! Name: " + name + ", data: " + data);

        file.close();

#ifdef FSTORAGE_DEBUG_VERBOSE
        fDebugUtils::Log("[fStorage] Saved file " + name);
#endif
        return true;
    }

    static bool ReadData(String name, String* read) {
        if (!spiffsMounted)
            beginSPIFFS();

        if (!SPIFFS.exists(name)) {
            fDebugUtils::Log("[fStorage] File doesn't exist! Name: " + name);
            //return false;
        }

        File file = SPIFFS.open("/" + name, "r");

        if (!file) {
            fDebugUtils::Log("[fStorage] Can't open file! Name: " + name);
            return false;
        }

        *read = file.readString();

#ifdef FSTORAGE_DEBUG_VERBOSE
        fDebugUtils::Log("[fStorage] Read data: \"" + *read + "\" from file: \"" + name + "\"");
#endif
        if (*read == "")
            return false;

        return true;
    }



private:
    static bool spiffsMounted;

    static void beginSPIFFS() {
        bool success = SPIFFS.begin();

        ledcSetup(0, 50000, 8);
        ledcAttachPin(2, 0);

        if (!success) {
            while (true) {
                fDebugUtils::Log("[fStorage] Mount SPIFFS failed!");
                fDebugUtils::Log("[fStorage] Type \"FLASH\" to try reflashing the FS.");
                ledcAttachPin(2, 0);
                while (!Serial.available()) {
                    ledcWrite(0, 64);
                    delay(125);
                    ledcWrite(0, 0);
                    delay(375);
                    ledcWrite(0, 64);
                    delay(125);
                    ledcWrite(0, 0);
                    delay(375);
                    fDebugUtils::alert_beep();
                }

                if (Serial.readStringUntil('\n') == "FLASH")
                    break;
            }
            fDebugUtils::Log("[fStorage] Reflashing FS. This may take some time...");
            fDebugUtils::beep();
            SPIFFS.format();

            fDebugUtils::success_beep();
            fDebugUtils::Log("[fStorage] Flash done.");
            success = SPIFFS.begin();
        }

        if (!success) {
            fDebugUtils::Log("[fStorage] Flash SPIFFS failed!");

            ledcAttachPin(2, 0);
            while (true) {
                ledcWrite(0, 64);
                delay(125);
                ledcWrite(0, 0);
                delay(375);
                ledcWrite(0, 64);
                delay(125);
                ledcWrite(0, 0);
                delay(375);
                fDebugUtils::alert_beep();
            }
        }

        spiffsMounted = true;

        if (fSerialParser::available)
            fSerialParser::AddCommand("fstorage_flash", &FlashSPIFFS);
    }

    static void FlashSPIFFS() {
        fDebugUtils::Log("[fStorage] Type \"FLASH\" to reflash the FS.");
        ledcAttachPin(2, 0);
        while (!Serial.available()) {
            ledcWrite(0, 64);
            delay(125);
            ledcWrite(0, 0);
            delay(375);
            ledcWrite(0, 64);
            delay(125);
            ledcWrite(0, 0);
            delay(375);
            fDebugUtils::alert_beep();
        }

        if (Serial.readStringUntil('\n') != "FLASH") {
            fDebugUtils::Log("[fStorage] Operation cancelled.");
            fDebugUtils::beep();
            return;
        }

        fDebugUtils::Log("[fStorage] Reflashing FS. This may take some time...");
        fDebugUtils::beep();
        SPIFFS.format();
        fDebugUtils::success_beep();
        fDebugUtils::Log("[fStorage] Flash done.");
    }
};

#endif