#pragma once
#ifndef fWiFiManager_h
#define fWiFiManager_h
#include <WiFi.h>
#include "fStorageUtils.h"
#include "fSerialParser.h"

#include <ESPAsyncWebServer.h>

#ifdef USE_FGUI
#include "fGUI.h"
#endif

class fWiFiManager {
public:

    static void StartAsTask() {
        xTaskCreate(start, "fWifi_Startup", 10000, NULL, 1, &StartTask);
    }

    static void start(void* parameter) {
        Start();
    }

    static void Start() {
        fDebugUtils::Log("[fWifi] Starting...");
        fSerialParser::AddCommand("fwifi_config", &Configure);

        bool dataLoaded = loadData();

        if (!dataLoaded) {
            setupFalloverAP("ESP_ACCESSPOINT", "123456");

            status = "DEFAULT AP";
            isdone = true;
            return;
        }

        if (!isAP)
            connect();
        else
            setupAP(ssid, pass);

        startServer();
        isdone = true;

        vTaskDelete(StartTask);

        while (true)
            delay(100);
    }

    static void Configure() {
        fDebugUtils::Log("[fWifi] Starting configuration");
        while (true) {
            Serial.println("Enter Hostname:");

            while (!Serial.available())
                delay(1);

            hostname = Serial.readStringUntil('\n');
            Serial.println("Hostname: \"" + hostname + "\"");


            Serial.println("Enter SSID:");

            while (!Serial.available())
                delay(1);

            ssid = Serial.readStringUntil('\n');
            Serial.println("SSID: \"" + ssid + "\"");

            Serial.println("Enter Password:");

            while (!Serial.available())
                delay(1);

            pass = Serial.readStringUntil('\n');
            Serial.println("Password: \"" + pass + "\"");

            Serial.println("Setup as access point? (y/n):");

            while (!Serial.available())
                delay(1);

            isAP = Serial.readStringUntil('\n') == "y";
            Serial.println("Is acces point: " + (String)(isAP ? "Yes" : "No"));

            Serial.println("Accept config? (y/n):");

            while (!Serial.available())
                delay(1);

            if (Serial.readStringUntil('\n') == "y")
                break;
        }

        fDebugUtils::Log("[fWifi] Configuration done.");

        saveData();
    }

    static String GetStatus() {
        return status;
    }

    static bool isDone() {
        return isdone;
    }

private:
    static TaskHandle_t StartTask;

    static void startServer() {
        server.on("/wifi_config", HTTP_GET, [](AsyncWebServerRequest* request) {
            if (!request->hasParam("ssid") || !request->hasParam("pass")) {
                request->send(400);
                return;
            }

            ssid = request->getParam("ssid")->value();
            pass = request->getParam("pass")->value();

            saveData();
            request->send(200);
            });
    }

    static void saveData() {
        fDebugUtils::Log("[fWifi] Saving data...");
        fStorageUtils::SaveData("/FWIFI_HOST", hostname);
        fStorageUtils::SaveData("/FWIFI_SSID", ssid);
        fStorageUtils::SaveData("/FWIFI_PASS", pass);
        fStorageUtils::SaveData("/FWIFI_MODE", isAP ? "a" : "s");
        fDebugUtils::Log("[fWifi] Saved.");
    }

    static bool loadData() {
        String modeData;

        bool hostFound = fStorageUtils::ReadData("/FWIFI_HOST", &hostname);
        bool ssidFound = fStorageUtils::ReadData("/FWIFI_SSID", &ssid);
        bool passFound = fStorageUtils::ReadData("/FWIFI_PASS", &pass);
        bool modeFound = fStorageUtils::ReadData("/FWIFI_MODE", &modeData);

        if (!hostFound || !ssidFound || !passFound || !modeFound) {
            fDebugUtils::Log("[fWifi] No saved data found!");
            return false;
        }

        if (modeData == "a")
            isAP = true;

        return true;
    }

    static void setupFalloverAP(String apSSID, String apPassword) {
        fDebugUtils::Log("[fWifi] Setting up fallover AP!");
        setupAP(apSSID, apPassword);
    }

    static void setupAP(String apSSID, String apPassword) {
        fDebugUtils::Log("[fWifi] AP SSID: " + apSSID + ", Password: " + apPassword);

        WiFi.disconnect();
        WiFi.mode(WIFI_AP);

        char buffer[64];
        char buffer2[64];

        apSSID.toCharArray(buffer, 64);
        apPassword.toCharArray(buffer2, 64);

        WiFi.softAP(buffer, buffer2);
        WiFi.enableAP(true);
    }

    static void connect() {
        char buffer[64];
        char buffer2[64];
        char buffer3[64];
        hostname.toCharArray(buffer, 64);

        WiFi.mode(WIFI_STA);
        WiFi.setHostname(buffer);
        WiFi.hostname(hostname);

        ssid.toCharArray(buffer2, 64);
        pass.toCharArray(buffer3, 64);
        WiFi.begin(buffer2, buffer3);

        fDebugUtils::Log("[fWifi] Connecting to " + ssid);
        status = "SSID: " + ssid;

        long startMS = millis();


        ledcSetup(0, 50000, 8);
        ledcAttachPin(2, 0);
        int dutyCycle = 0;
        while (WiFi.status() != WL_CONNECTED && millis() - startMS < 10000) {
            //fDebugUtils::beep();
            for (dutyCycle = 0; dutyCycle <= 64 && WiFi.status() != WL_CONNECTED && millis() - startMS < 10000; dutyCycle++) {
                // changing the LED brightness with PWM
                ledcWrite(0, dutyCycle);
                delay(20);
            }
            //fDebugUtils::beep();
            // decrease the LED brightness
            for (; dutyCycle >= 0 && WiFi.status() != WL_CONNECTED && millis() - startMS < 10000; dutyCycle--) {
                // changing the LED brightness with PWM
                ledcWrite(0, dutyCycle);
                delay(20);
            }
        }
        if (!WiFi.isConnected()) {
            fDebugUtils::Log("[fWifi] Failed to connect to " + ssid);
            setupFalloverAP(hostname + "_AP", pass);
            status = "FAILOVER AP";
            return;
        }
        fDebugUtils::Log("[fWifi] Connected to " + ssid + ".");
        fDebugUtils::Log("[fWifi] IP is " + WiFi.localIP().toString() + ".");
        status = "IP: " + WiFi.localIP().toString();

        isdone = true;


        if (WiFi.isConnected()) {
            //fDebugUtils::success_beep();
            for (int blinks = 0; blinks < 2; blinks++) {
                for (; dutyCycle <= 64; dutyCycle++) {
                    // changing the LED brightness with PWM
                    ledcWrite(0, dutyCycle);
                    delay(5);
                }

                // decrease the LED brightness
                for (; dutyCycle >= 0; dutyCycle--) {
                    // changing the LED brightness with PWM
                    ledcWrite(0, dutyCycle);
                    delay(5);
                }
            }

            for (; dutyCycle <= 16; dutyCycle++) {
                // changing the LED brightness with PWM
                ledcWrite(0, dutyCycle);
                delay(50);
            }
        }
        else {
            ledcWrite(2, 64);
            delay(125);
            ledcWrite(2, 0);
            delay(375);
            ledcWrite(2, 64);
            delay(125);
            ledcWrite(2, 0);
            delay(375);
        }
    }

    static String hostname, ssid, pass, status;
    static bool isAP, isdone;
    static AsyncWebServer server;
};

#endif