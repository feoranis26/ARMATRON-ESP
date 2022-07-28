#pragma once
#ifndef fWiFiServerActionSystem_h
#define fWiFiServerActionSystem_h
#include <Arduino.h>
#include "fDebugUtils.h"
#include "fWiFiManager.h"

#include <ESPAsyncWebServer.h>

#ifdef USE_FGUI
#include "fGUI.h"
#endif

class WiFiCommand {
public:
    WiFiCommand(String n, void (*run)()) {
        name = n;
        execute = run;
    }
    WiFiCommand() {
        name = "";
        execute = nullptr;
    }
    String name;
    void (*execute)();
    TaskHandle_t taskHandle;
};
class fREST {
public:
    static void Begin() {
        server.on("/run", HTTP_GET, [](AsyncWebServerRequest* request) {
            if (!request->hasParam("action")) {
                request->send(400, "text/plain", "No action specified.");
                return;
            }

            for (int i = 0; i < numCommands; i++) {
                if (commands[i].name == request->getParam("action")->value()) {
                    currentCommand = &(commands[i]);

                    xTaskCreate(runCommand, "fRESTCommand", 20000, NULL, 1, &handle);
                    request->send(200);
                }
            }
            });
        server.begin();
    }

    static void runCommand(void *param) {
#ifdef USE_FGUI
        fGUI::StartMenu();
        fGUI::Clear(true);
        fGUI::SetFont(u8g2_font_5x7_tr, true);
        fGUI::PrintCentered("Running " + currentCommand->name, 64, 32, true);
        fGUI::Flush(true);
#endif
        currentCommand->execute();
#ifdef USE_FGUI
        fGUI::EndMenu();
#endif

        vTaskDelete(handle);
        while (true)
            delay(100);
    }


    static void AddCommand(String name, void (*run)()) {
        fDebugUtils::Log("add cmd name=" + name);
        commands[numCommands] = WiFiCommand(name, run);
        numCommands++;
    }

    static void parseSerial() {
        if (!Serial.available())
            return;

        String read = Serial.readStringUntil(' ');
        if (read.endsWith("\n"))
            read.substring(0, read.length() - 2);

        fDebugUtils::Log(read);

        for (int i = 0; i < numCommands; i++) {
            if (commands[i].name == read) {
#ifdef USE_FGUI
                fGUI::Clear();
                fGUI::SetFont(u8g2_font_5x7_tr);
                fGUI::PrintCentered("Running " + commands[i].name, 64, 32);
                fGUI::Flush();
#endif
                commands[i].execute();
                break;
            }
        }
    }

    static bool available;
private:
    static WiFiCommand commands[64];
    static WiFiCommand* currentCommand;
    static AsyncWebServer server;
    static int numCommands;
    static TaskHandle_t handle;
};

#endif