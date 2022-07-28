#pragma once
#ifndef fSerialParser_h
#define fSerialParser_h
#include <Arduino.h>
#include "fDebugUtils.h"

#ifdef USE_FGUI
#include "fGUI.h"
#endif

class SerialCommand {
public:
    SerialCommand(String n, void (*run)()) {
        name = n;
        execute = run;
    }
    SerialCommand() {
        name = "";
        execute = nullptr;
    }
    String name;
    void (*execute)();
};
class fSerialParser {
public:
    static void Begin(int baud) {
        Serial.begin(baud);
        Serial.setTimeout(10);

        available = true;
    }

    static void BeginAsTask(int baud) {
        Serial.begin(baud);
        Serial.setTimeout(10);

        available = true;

        xTaskCreate(runTask, "fSerialParser", 20000, NULL, 0, &taskHandle);
    }

    static void AddCommand(String name, void (*run)()) {
        fDebugUtils::Log("add cmd name=" + name);
        commands[numCommands] = SerialCommand(name, run);
        numCommands++;
    }

    static void parseSerial() {
        if(!Serial.available())
            return;

        String read = Serial.readStringUntil(' ');
        if (read.endsWith("\n"))
            read.substring(0, read.length() - 2);

        fDebugUtils::Log(read);

        for (int i = 0; i < numCommands; i++) {
            if (commands[i].name == read) {

#ifdef USE_FGUI
                fGUI::StartMenu();
                fGUI::Clear(true);
                fGUI::SetFont(u8g2_font_5x7_tr, true);
                fGUI::PrintCentered("Running " + commands[i].name, 64, 32, true);
                fGUI::Flush(true);
#endif
                commands[i].execute();
#ifdef USE_FGUI
                fGUI::EndMenu();
#endif
                break;
            }
        }
    }

    static bool available;
private:
    static SerialCommand commands[64];
    static int numCommands;
    static TaskHandle_t taskHandle;

    static void runTask(void *param) {
        while (true) {
            parseSerial();
            delay(200);
        }
    }
};

#endif