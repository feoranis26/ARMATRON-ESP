#pragma once
#ifndef fGPIOActionSystem_h
#define fGPIOActionSystem_h
#include <Arduino.h>
#include "fDebugUtils.h"

#ifdef USE_FGUI
#include "fGUI.h"
#endif

class GPIOAction {
public:
    GPIOAction(int p, void (*run)(), bool triggerWhenHigh = false) {
        pin = p;
        execute = run;
        trigger = triggerWhenHigh;
    }
    GPIOAction() {
        pin = -1;
        execute = nullptr;
        trigger = false;
    }
    int pin;
    void (*execute)();
    bool trigger;
};
class fGPIOActionSystem {
public:
    static void Begin() {
        available = true;
    }

    static void BeginAsTask() {
        xTaskCreate(runTask, "fGPIOActions", 20000, NULL, 0, &taskHandle);
    }

    static void AddAction(int pin, void (*run)(), bool trigger) {
        actions[numActions] = GPIOAction(pin, run, trigger);
        numActions++;
    }

    static void run() {
        for (int i = 0; i < numActions; i++) {
            if (digitalRead(actions[i].pin) == actions[i].trigger) {
#ifdef USE_FGUI
                fGUI::StartMenu();
                fGUI::Clear(true);
                fGUI::SetFont(u8g2_font_5x7_tr, true);
                fGUI::PrintCentered("Running GPIOCMD pin" + String(actions[i].pin), 64, 32, true);
                fGUI::Flush(true);
#endif
                actions[i].execute();
#ifdef USE_FGUI
                fGUI::EndMenu();
#endif
                break;
            }
        }
    }

    static bool available;
private:
    static GPIOAction actions[64];
    static int numActions;
    static TaskHandle_t taskHandle;

    static void runTask(void* parameter) {
        while (true) {
            delay(100);
            run();
        }
    }
};

#endif