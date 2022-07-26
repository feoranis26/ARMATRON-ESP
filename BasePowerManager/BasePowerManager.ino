#define USE_FGUI
#define BUZZER_PIN 19
#define FDEBUG
#define FSTORAGE_DEBUG_VERBOSE
//#define WIFI_ENABLE

#include "flib.h"

#include <ESPAsyncWebServer.h>

//AsyncWebServer srv(80);

void setup() {
    analogSetAttenuation(ADC_11db);
    adcAttachPin(32);
    adcAttachPin(35);
    adcAttachPin(34);

    pinMode(18, INPUT);
    digitalWrite(18, HIGH);

    pinMode(23, OUTPUT);
    digitalWrite(23, HIGH);

    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

    fWiFiActions::AddCommand("enable_power", &enablePower);
    fWiFiActions::AddCommand("disable_power", &disablePower);
    fWiFiActions::AddCommand("disable_vcheck", &disableVoltageChk);
    fWiFiActions::AddCommand("shutdown", &Shutdown);

    fGPIOActionSystem::AddAction(18, &shutdownBtn, true);
    fSerialParser::AddCommand("enable_power", &enablePower);
    fSerialParser::AddCommand("disable_power", &disablePower);
    fSerialParser::AddCommand("disable_vcheck", &disableVoltageChk);
    fSerialParser::AddCommand("shutdown", &Shutdown);

    flib_Startup();
    //srv.begin();
}

bool isPowerOn = false;

void enablePower() {
    if (isPowerOn)
        return;

    fDebugUtils::beep();
    fGUI::StartMenu();

    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_10x20_tr, true);

        fGUI::PrintCentered("Power on!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(i) / 6, true, true);

        fGUI::Flush(true);

        delay(200);
    }
    digitalWrite(25, HIGH);

    long startms = millis();
    float voltage12v = (analogReadMilliVolts(35) * 11.0 / 1000.0) - 0.3;

    fDebugUtils::beep();

    while (voltage12v < 9) {
        voltage12v = (analogReadMilliVolts(35) * 11.0 / 1000.0) - 0.3;

        fGUI::SetFont(u8g2_font_6x10_tr, true);
        fGUI::PrintCentered("Checking voltage", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
        fGUI::Flush(true);
        

        if (millis() - startms > 1000) {
            digitalWrite(25, LOW);

            fDebugUtils::alert_beep();

            fGUI::SetFont(u8g2_font_8x13B_tr, true);
            fGUI::PrintCentered("NO 12V SUPPLY!", 64, 16, true);
            fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
            fGUI::Flush(true);

            while (millis() - startms < 10000 && !digitalRead(18)) {
                fDebugUtils::error_tone();
            }

            while (!digitalRead(18)) {
                fDebugUtils::alert_beep();
                for (int i = 0; i < 38; i++) {
                    delay(250);
                    if (digitalRead(18))
                        break;
                }
            }

            return;
        }
    }

    fGUI::SetFont(u8g2_font_6x10_tr, true);
    fGUI::PrintCentered("Checking voltage", 64, 16, true);
    fGUI::ProgressBar(64, 36, 96, 16, 1, true, true);
    fGUI::Flush(true);

    delay(500);


    isPowerOn = true;
    fGUI::EndMenu();
}

void disablePower() {
    if (!isPowerOn)
        return;

    fDebugUtils::beep();
    fGUI::StartMenu();

    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_10x20_tr, true);

        fGUI::PrintCentered("Power off!", 64, 16 , true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(5 - i) / 5, false, true);

        fGUI::Flush(true);

        delay(200);
    }

    digitalWrite(25, LOW);

    isPowerOn = false;

    fGUI::EndMenu();
}

bool voltageCheck = true;
void disableVoltageChk() {
    voltageCheck = false;
}

void shutdownBtn() {
    fGUI::StartMenu();

    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_10x20_tr, true);

        fGUI::PrintCentered("HARD RESET!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(5 - i) / 5, true, true);

        fGUI::SetFont(u8g2_font_5x7_tr, true);
        fGUI::PrintCentered("SHUTDOWN IN " + String(5 - i) + " s!", 64, 60, true);

        fGUI::Flush(true);

        fDebugUtils::beep();

        if (!digitalRead(18))
            return;

        delay(750);

        if (!digitalRead(18))
            return;
    }

    Shutdown();
    fGUI::EndMenu();
}

void Shutdown() {
    fGUI::StartMenu();
    fDebugUtils::alert_beep();

    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_10x20_tr, true);

        fGUI::PrintCentered("Shutting down!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(i) / 5, true, true);

        fGUI::SetFont(u8g2_font_5x7_tr, true);
        fGUI::Flush(true);

        delay(200);
    }

    digitalWrite(23, LOW);

    fGUI::SetFont(u8g2_font_10x20_tr, true);
    fGUI::PrintCentered("BYE", 64, 32, true);
    fGUI::Flush(true);

    ESP.restart();
}

// the loop function runs over and over again until power down or reset
void loop() {
    double voltage5v  = (analogReadMilliVolts(32) * 11.0 / 1000.0) - 0.3;
    double voltage12v = (analogReadMilliVolts(35) * 11.0 / 1000.0) - 0.3;
    double voltage24v = (analogReadMilliVolts(34) * 11.0 / 1000.0) - 0.3;

    double secs = (double)millis() / 1000;  

    if (digitalRead(18))
        fGUI::ResetBurnInProtectionTimeout();

    if(fmod(secs, 2) < 1)
        fGUI::ProgressBar(124, 62, 8, 4, fmod(secs, 2), true);
    else
        fGUI::ProgressBar(124, 62, 8, 4, 2 - fmod(secs, 2), false);

    if (voltageCheck) {
        if (isPowerOn && voltage12v < 9) {
            digitalWrite(25, LOW);

            fDebugUtils::alert_beep();

            fGUI::SetFont(u8g2_font_10x20_tr);
            fGUI::PrintCentered("12V TOO LOW!", 64, 37);
            fGUI::Flush();
            fDebugUtils::error_tone();

            long startms = millis();

            while (millis() - startms < 10000 && !digitalRead(18)) {
                fDebugUtils::error_tone();
            }

            while (!digitalRead(18)) {
                fDebugUtils::alert_beep();
                for (int i = 0; i < 38; i++) {
                    delay(250);
                    if (digitalRead(18))
                        break;
                }
            }

            isPowerOn = false;
            return;
        }
        
        if (false && voltage24v < 18) {
            digitalWrite(25, LOW);

            fDebugUtils::alert_beep();

            fGUI::SetFont(u8g2_font_10x20_tr);
            fGUI::PrintCentered("24V TOO LOW!", 64, 37);
            fGUI::Flush();
            fDebugUtils::error_tone();

            long startms = millis();

            while (millis() - startms < 10000 && !digitalRead(18)) {
                fDebugUtils::error_tone();
            }

            while (!digitalRead(18)) {
                fDebugUtils::alert_beep();
                for (int i = 0; i < 38; i++) {
                    delay(250);
                    if (digitalRead(18))
                        break;
                }
            }

            return;
        }
    }

    fGUI::SetFont(u8g2_font_8x13_tr);

    fGUI::PrintCentered("5V  : " + String(voltage5v) + "v ON", 64, 22);
    fGUI::PrintCentered("12V : " + String(isPowerOn ? (String(voltage12v) + "v ON") : "OFF"), 64, 37);
    fGUI::PrintCentered("24V : " + String(false ? (String(voltage24v) + "v ON") : "OFF"), 64, 52);

    fGUI::SetFont(u8g2_font_3x5im_tr);

    if(fWiFiManager::GetStatus() != "Not started")
        fGUI::Print(fWiFiManager::GetStatus(), 2, 62);

    fGUI::Flush();
    delay(50);
}
