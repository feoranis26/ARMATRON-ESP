#pragma once
#ifndef fGUI_h
#define fGUI_h

//#include "fDebugUtils.h"
#include <U8x8lib.h>
#include <U8g2lib.h>
#include <MUIU8g2.h>

class fLabel {
public:
    String text;
    int x;
    int y;

};

class fGUI {
public:
    static void begin() {
        //fDebugUtils::Log("[fGUI] Connecting to display...");

        display.begin();

        //fDebugUtils::beep();
        //fDebugUtils::Log("[fGUI] Connected!");

        // Display Text
        display.clearBuffer();
        display.setFont(u8g2_font_10x20_tr);

        PrintCentered("Starting", 64, 20);
        display.sendBuffer();
        delay(2000);
    }

    static void SetFont(const uint8_t *font, bool menu  = false) {
        if (inMenu && !menu)
            return;

        display.setFont(font);
    }

    static void Print(String text, int x, int y, bool menu = false) {
        if (inMenu && !menu)
            return;

        char buf[64];
        text.toCharArray(buf, 64);
        display.drawStr(x, y, buf);
    }

    static void PrintCentered(String text, int x, int y, bool menu = false) {
        if (inMenu && !menu)
            return;

        char buf[64];
        text.toCharArray(buf, 64);

        int length = display.getStrWidth(buf);
        display.drawStr(x - length / 2, y, buf);
    }

    static void DrawBoxCentered(int x, int y, int w, int h, bool menu = false) {
        if (inMenu && !menu)
            return;

        display.drawBox(x - w / 2, y - h / 2, w, h);

    }
    static void DrawFrameCentered(int x, int y, int w, int h, bool menu = false) {
        if (inMenu && !menu)
            return;

        display.drawFrame(x - w / 2, y - h / 2, w, h);
    }

    static void Flush(bool menu = false) {
        if (inMenu && !menu)
            return;

        if (millis() - lastUserInput < 60000) {
            display.sendBuffer();
            display.clearBuffer();
        }
        else {
            display.clearBuffer();
            display.sendBuffer();
        }
    }

    static void Clear(bool menu = false) {
        if (inMenu && !menu)
            return;

        display.clearBuffer();
    }

    static void ProgressBar(int x, int y, int w, int h, double progress, bool white = true, bool menu = false) {
        if (inMenu && !menu)
            return;

        DrawFrameCentered(x, y, w, h, menu);
        if(white)
            display.drawBox(x - w / 2, y - h / 2,w * progress, h);
        else
            display.drawBox((x + w / 2) - w * progress, y - h / 2, w * progress, h);
    }

    static void ResetBurnInProtectionTimeout() {
        lastUserInput = millis();
    }

    static void StartMenu() {
        ResetBurnInProtectionTimeout();
        inMenu = true;
    }

    static void EndMenu() {
        inMenu = false;
    }

private:
    static U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
    static long lastUserInput;
    static bool inMenu;
};
 
#endif