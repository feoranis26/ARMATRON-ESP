#pragma once
#ifndef fDebug_h
#define fDebug_h
#include "Arduino.h"
#include "Esp.h"

#ifdef USE_FGUI
#include "fGUI.h"
#endif

class fDebugUtils
{
public:
    static void ThrowException(String message) {
#ifdef FDEBUG
        Serial.println(message);
        delay(1000);
#endif
        abort();
    }

    static void Log(String message) {
#ifdef FDEBUG
        Serial.println(message);
#endif
#ifdef USE_FGUI_DEBUG
        fGUI::SetFont(u8g2_font_3x5im_tr);
        fGUI::Print(message, 0, 59);
        fGUI::Flush();
#endif
        //lastLogMesage = message;
    }

    static void beep() {
#ifdef BUZZER_PIN
        ledcAttachPin(BUZZER_PIN, 1);
        ledcWriteTone(1, 500);
        delay(125);
        ledcWriteTone(1, 1000);
        delay(125);
        ledcWrite(1, 0);
        ledcDetachPin(BUZZER_PIN);
#endif
    }
    static void alert_beep() {
#ifdef BUZZER_PIN
        ledcAttachPin(BUZZER_PIN, 1);
        ledcWriteTone(1, 500);
        delay(125);
        ledcWriteTone(1, 1000);
        delay(125);
        ledcWrite(1, 0);
        delay(375);
        ledcWriteTone(1, 500);
        delay(125);
        ledcWriteTone(1, 1000);
        delay(125);
        ledcWrite(1, 0);
        delay(375);
        ledcDetachPin(BUZZER_PIN);
#endif
    }

    static void success_beep() {
#ifdef BUZZER_PIN
        ledcAttachPin(BUZZER_PIN, 1);
        ledcWriteTone(1, 500);
        delay(250);
        ledcWriteTone(1, 750);
        delay(250);
        ledcWriteTone(1, 1000);
        delay(375);
        ledcDetachPin(BUZZER_PIN);
#endif
    }

    static void startup_beep() {
#ifdef BUZZER_PIN
        ledcAttachPin(BUZZER_PIN, 1);
        ledcWriteTone(1, 600);
        delay(250);
        ledcWriteTone(1, 900);
        delay(125);
        ledcWrite(1, 0);
        delay(125);
        ledcWriteTone(1, 900);
        delay(125);
        ledcWrite(1, 0);
        delay(125);
        ledcDetachPin(BUZZER_PIN);
#endif
    }

    static void shutdown_beep() {
#ifdef BUZZER_PIN
        ledcAttachPin(BUZZER_PIN, 1);
        ledcWriteTone(1, 1000);
        delay(250);
        ledcWriteTone(1, 750);
        delay(250);
        ledcWriteTone(1, 500);
        delay(375);
        ledcDetachPin(BUZZER_PIN);
#endif
    }

    static void error_tone() {
#ifdef BUZZER_PIN
        ledcAttachPin(BUZZER_PIN, 1);
        ledcWriteTone(1, 800);
        delay(500);
        ledcWriteTone(1, 1600);
        delay(500);
        ledcDetachPin(BUZZER_PIN);
#endif
    }

    static void beep_background() {
        xTaskCreate(beep_bgtask, "beep", 1000, NULL, 1, NULL);
    }

    static void beep_bgtask(void* arg) {
        beep();

        vTaskDelete(NULL);
        delay(100);
    }

    static void alert_background() {
        xTaskCreate(alert_bgtask, "beep", 1000, NULL, 1, NULL);
    }

    static void alert_bgtask(void* arg) {
        alert_beep();

        vTaskDelete(NULL);
        delay(100);
    }

    static void success_background() {
        xTaskCreate(success_bgtask, "beep", 1000, NULL, 1, NULL);
    }

    static void success_bgtask(void* arg) {
        success_beep();

        vTaskDelete(NULL);
        delay(100);
    }

    static void shutdown_background() {
        xTaskCreate(shutdown_bgtask, "beep", 1000, NULL, 1, NULL);
    }

    static void shutdown_bgtask(void* arg) {
        shutdown_beep();

        vTaskDelete(NULL);
        delay(100);
    }

    TaskHandle_t runningTask;
    //static String lastLogMesage;
};

//String fDebugUtils::lastLogMesage = "";

#endif

