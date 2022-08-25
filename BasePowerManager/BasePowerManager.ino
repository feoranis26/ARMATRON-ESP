#define USE_FGUI
#define BUZZER_PIN 19
#define FDEBUG
//#define WIFI_ENABLE

#include "flib.h"
#include "fComms.h"

#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 adc;
//AsyncWebServer srv(80);


bool isPowerOn = false;
bool isCharging = false;
bool canCharge = false;

void setup() {
    analogSetAttenuation(ADC_11db);
    adcAttachPin(32);
    adcAttachPin(35);
    adcAttachPin(34);
    adcAttachPin(39);

    pinMode(18, INPUT);
    digitalWrite(18, HIGH);

    pinMode(23, OUTPUT);
    digitalWrite(23, HIGH);

    pinMode(25, OUTPUT);
    digitalWrite(25, LOW);

    pinMode(26, OUTPUT);
    digitalWrite(26, LOW);

    fREST::AddCommand("enable_power", &enablePower);
    fREST::AddCommand("disable_power", &disablePower);
    fREST::AddCommand("disable_vcheck", &disableVoltageChk);
    fREST::AddCommand("shutdown", &Shutdown);

    fGPIOActionSystem::AddAction(18, &shutdownBtn, true);
    fSerialParser::AddCommand("enable_power", &enablePower);
    fSerialParser::AddCommand("disable_power", &disablePower);
    fSerialParser::AddCommand("disable_vcheck", &disableVoltageChk);
    fSerialParser::AddCommand("shutdown", &Shutdown);
    fSerialParser::AddCommand("shutdown_10s", &ShutdownDelayed);

    fComms::AddCommand("enable_power", [](String args) {
        enablePower();
        });

    fComms::AddCommand("disable_power", [](String args) {
        disablePower();
        });

    fComms::AddCommand("start_chg", [](String args) {
        startCharging();
        });

    fComms::AddCommand("stop_chg", [](String args) {
        stopCharging();
        });

    fComms::AddCommand("disable_vcheck", [](String args) {
        disableVoltageChk();
        });

    fComms::AddCommand("get_voltage", [](String args) {
        double voltage5v = (analogReadMilliVolts(32) * 11.0 / 1000.0) - 0.3;
        double voltage12v = (analogReadMilliVolts(35) * 11.0 / 1000.0) - 0.3;
        double voltage24v = (analogReadMilliVolts(34) * 11.0 / 1000.0) - 0.3;

        fComms::TCPSend("5v=" + String(voltage5v) + ",12v=" + String(voltage12v) + ",24v=" + String(voltage24v) + "\r");
        });

    fComms::AddCommand("shutdown", [](String args) {
        Shutdown();
        });

    fComms::AddCommand("shutdown_delay", [](String args) {
        ShutdownDelayed();
        });

    flib_Startup();

    if (!adc.begin()) {
        fDebugUtils::Log("Can't init ADC!");

        fGUI::SetFont(u8g2_font_10x20_tr);
        fGUI::PrintCentered("ADC FAIL", 64, 16);
        fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true);
        fGUI::Flush();

        fDebugUtils::alert_beep();

        while (true)
            fDebugUtils::error_tone();
    }

    adc.setGain(GAIN_TWOTHIRDS);

    fComms::StartAsTask("POWERMGR");

    ArduinoOTA.setHostname("POWERMGR");
    ArduinoOTA.begin();
    //srv.begin();
}

void enablePower() {
    if (isPowerOn || isCharging)
        return;

    fDebugUtils::beep();
    fGUI::StartMenu();
    fDebugUtils::success_background();
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
    float voltage24v = 24;// (analogReadMilliVolts(34) * 11.0 / 1000.0) - 0.3;

    fDebugUtils::beep();

    while (voltage12v < 9 || voltage24v < 18) {
        voltage12v = (analogReadMilliVolts(35) * 11.0 / 1000.0) - 0.3;
        voltage24v = 24;// (analogReadMilliVolts(34) * 11.0 / 1000.0) - 0.3;

        fGUI::SetFont(u8g2_font_6x10_tr, true);
        fGUI::PrintCentered("Checking voltage", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
        fGUI::Flush(true);


        if (millis() - startms > 1000) {
            digitalWrite(25, LOW);

            fGUI::SetFont(u8g2_font_8x13B_tr, true);
            fGUI::PrintCentered(voltage12v < 9 ? (voltage24v > 18 ? "NO 12V SUPPLY!" : "NO POWER!") : "NO 24V SUPPLY!", 64, 16, true);
            fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
            fGUI::Flush(true);

            fDebugUtils::alert_beep();

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

void startCharging() {
    if (isCharging)
        return;

    if (!canCharge) {
        fDebugUtils::shutdown_background();
        return;
    }

    if (isPowerOn) {
        disablePower();
        delay(500);
    }

    fDebugUtils::alert_beep2();

    fGUI::StartMenu();
    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_8x13_tr, true);

        fGUI::PrintCentered("START CHARGE!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(i) / 6, true, true);

        fGUI::Flush(true);

        delay(200);
    }

    fGUI::SetFont(u8g2_font_6x10_tr, true);
    fGUI::PrintCentered("Checking voltage", 64, 16, true);
    fGUI::ProgressBar(64, 36, 96, 16, 1, true, true);
    fGUI::Flush(true);

    delay(500);

    float voltage12v = adc.readADC_SingleEnded(0) * 11;
    float voltage24v = adc.readADC_SingleEnded(1) * 11;
    if (voltage12v < 8)
    {
        digitalWrite(26, LOW);

        fGUI::SetFont(u8g2_font_10x20_tr, true);
        fGUI::PrintCentered("12V N/C!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
        fGUI::Flush(true);

        fDebugUtils::error_beep();
        delay(2500);

        fGUI::EndMenu();
        return;
    }

    if (voltage24v < 16)
    {
        digitalWrite(26, LOW);

        fGUI::SetFont(u8g2_font_10x20_tr, true);
        fGUI::PrintCentered("24V N/C!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
        fGUI::Flush(true);

        fDebugUtils::error_beep();
        delay(2500);

        fGUI::EndMenu();
        return;
    }

    digitalWrite(26, HIGH);

    long startms = millis();
    float current12v = (adc.computeVolts(adc.readADC_SingleEnded(2)) - 2.5) * (1 / 0.185);
    float current24v = (adc.computeVolts(adc.readADC_SingleEnded(3)) - 2.5) * (1 / 0.185);
    fDebugUtils::beep();

    /*
    while (current12v < 0.1 || current24v < 0.1) {
        current12v = (adc.readADC_SingleEnded(2) * 0.000125 - 2.5) * (1 / 0.185);
        current24v = (adc.readADC_SingleEnded(3) * 0.000125 - 2.5) * (1 / 0.185);

        fGUI::SetFont(u8g2_font_6x10_tr, true);
        fGUI::PrintCentered("Checking current", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
        fGUI::Flush(true);


        if (millis() - startms > 250) {
            digitalWrite(26, LOW);

            fGUI::SetFont(u8g2_font_10x20_tr, true);
            fGUI::PrintCentered("CNCT ERR!", 64, 16, true);
            fGUI::ProgressBar(64, 36, 96, 16, 5.0 / 6.0, true, true);
            fGUI::Flush(true);

            fDebugUtils::error_beep();
            delay(2500);

            fGUI::EndMenu();
            return;
        }
    }

    fGUI::SetFont(u8g2_font_6x10_tr, true);
    fGUI::PrintCentered("Checking current", 64, 16, true);
    fGUI::ProgressBar(64, 36, 96, 16, 1, true, true);
    fGUI::Flush(true);

    delay(500);

    */


    isCharging = true;
    fGUI::EndMenu();
    fDebugUtils::success_background();
}

void disablePower() {
    if (!isPowerOn)
        return;

    fDebugUtils::beep();
    fGUI::StartMenu();

    fDebugUtils::shutdown_background();
    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_10x20_tr, true);

        fGUI::PrintCentered("Power off!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(5 - i) / 5, false, true);

        fGUI::Flush(true);

        delay(200);
    }

    digitalWrite(25, LOW);

    isPowerOn = false;

    fGUI::EndMenu();
}

void stopCharging() {
    if (!isCharging)
        return;

    fDebugUtils::beep();
    fGUI::StartMenu();

    fDebugUtils::shutdown_background();
    for (int i = 0; i < 5; i++) {
        fGUI::SetFont(u8g2_font_8x13_tr, true);

        fGUI::PrintCentered("Stop charging!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(5 - i) / 5, false, true);

        fGUI::Flush(true);

        delay(200);
    }

    digitalWrite(26, LOW);

    isCharging = false;

    fGUI::EndMenu();
}

bool voltageCheck = true;
void disableVoltageChk() {
    voltageCheck = false;
}

void shutdownBtn() {
    fGUI::EndMenu();

    for (int i = 0; i < 20; i++) {
        if (!digitalRead(18))
            return;
        delay(100);
    }

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
    fDebugUtils::shutdown_background();

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

void ShutdownDelayed() {
    fGUI::StartMenu();
    fDebugUtils::alert_background();
    for (int i = 0; i < 40; i++) {
        fGUI::SetFont(u8g2_font_10x20_tr, true);

        fGUI::PrintCentered("Shutting down!", 64, 16, true);
        fGUI::ProgressBar(64, 36, 96, 16, (double)(i) / 40, true, true);

        fGUI::SetFont(u8g2_font_5x7_tr, true);
        fGUI::Flush(true);
        delay(250);

        if (i % 4 == 0)
            fDebugUtils::beep_background();

        if (digitalRead(18))
            return;
    }

    fDebugUtils::shutdown_beep();

    digitalWrite(23, LOW);

    fGUI::SetFont(u8g2_font_10x20_tr, true);
    fGUI::PrintCentered("BYE", 64, 32, true);
    fGUI::Flush(true);

    ESP.restart();
}

// the loop function runs over and over again until power down or reset
void loop() {
    ArduinoOTA.handle();

    double voltage5v = (analogReadMilliVolts(32) * 11.0 / 1000.0) - 0.3;
    double voltage12v = (analogReadMilliVolts(35) * 11.0 / 1000.0) - 0.3;
    double voltage24v = (analogReadMilliVolts(34) * 11.0 / 1000.0) - 0.3;
    double voltage5vRegIn = (analogReadMilliVolts(33) * 11.0 / 1000.0) - 0.3;

    float chargeCurrent12v = (adc.computeVolts(adc.readADC_SingleEnded(2)) - 2.43) / 0.37;
    float chargeCurrent24v = (adc.computeVolts(adc.readADC_SingleEnded(1)) - 2.43) / 0.37;

    float chargeVoltage12v = adc.computeVolts(adc.readADC_SingleEnded(0)) * 11.015;
    float chargeVoltage24v = (analogReadMilliVolts(39) * 10.527 / 1000.0);

    double secs = (double)millis() / 1000;

    if (digitalRead(18))
        fGUI::ResetBurnInProtectionTimeout();

    if (fmod(secs, 2) < 1)
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

        if (voltage5vRegIn < 9) {
            fDebugUtils::alert_beep();

            fGUI::SetFont(u8g2_font_10x20_tr);
            fGUI::PrintCentered("5V SP LOW!", 64, 37);
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

                if (voltage5vRegIn < 8)
                    Shutdown();
            }
        }

        /*
        if (isCharging && chargeCurrent12v < 0.1 && chargeCurrent24v < 0.1) {
            stopCharging();
        }
        */


        if (isCharging && (chargeVoltage12v < 9 || chargeVoltage24v < 18)) {
            digitalWrite(26, LOW);

            fDebugUtils::alert_beep();

            fGUI::SetFont(u8g2_font_10x20_tr);
            fGUI::PrintCentered("CHG DSCNCT!", 64, 37);
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

    if (!canCharge && chargeVoltage12v > 9 && chargeVoltage24v > 18) {
        fDebugUtils::success_background();
        canCharge = true;
    }

    else if (!(chargeVoltage12v > 9 && chargeVoltage24v > 18)) {
        if (canCharge && !isCharging)
            fDebugUtils::shutdown_background();

        canCharge = false;
    }

    fGUI::SetFont(u8g2_font_8x13_tr);

    fGUI::Print("5V : " + String(voltage5v) + "v", 0, 17);
    fGUI::Print("12V: " + String(isPowerOn ? (String(voltage12v) + "v") : (isCharging ? ("CHG " + String(floor(chargeCurrent12v * 10) / 10) + "A") : "OFF")), 0, 32);
    fGUI::Print("24V: " + String(isPowerOn ? (String(voltage24v) + "v") : (isCharging ? ("CHG " + String(floor(chargeCurrent24v * 10) / 10) + "A") : "OFF")), 0, 47);

    if (!isCharging) {
        fGUI::Print("5v SPLY", 72, 32);
        fGUI::Print(String(voltage5vRegIn) + "v", 88, 45);
    }

    fGUI::SetFont(u8g2_font_3x5im_tr);

    if (fWiFiManager::GetStatus() != "Not started")
        fGUI::Print(fWiFiManager::GetStatus(), 2, 64);

    fGUI::Print(fComms::GetStatus(), 2, 56);

    fGUI::Flush();

    fComms::TCPSend("5v: " + String(voltage5v));
    fComms::TCPSend("12v: " + String(voltage12v));
    fComms::TCPSend("24v: " + String(voltage24v));

    fComms::TCPSend("in_12v: " + String(chargeVoltage12v));
    fComms::TCPSend("in_24v: " + String(chargeVoltage24v));

    fComms::TCPSend("in_12v_c: " + String(chargeCurrent12v));
    fComms::TCPSend("in_24v_c: " + String(chargeCurrent24v));

    delay(50);
}
