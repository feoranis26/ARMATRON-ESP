#pragma once
#ifndef flib_h
#define flib_h

#include "fGUI.h"
#include "fDebugUtils.h"
#include "fSerialParser.h"
#include "fWiFiManager.h"
#include "fGPIOActionSystem.h"
#include "fWiFiServerActionSystem.h"

void flib_Startup() {
    fDebugUtils::startup_beep();

    fGUI::begin();

    fGUI::PrintCentered("Starting", 64, 16);
    fGUI::ProgressBar(64, 36, 96, 16, 0);
    fGUI::Flush();


    fGPIOActionSystem::BeginAsTask();
    fSerialParser::BeginAsTask(115200);

    fDebugUtils::beep();
    fGUI::PrintCentered("Starting WiFi", 64, 16);
    fGUI::ProgressBar(64, 36, 96, 16, 0.5);
    fGUI::Flush();

    fWiFiManager::StartAsTask();

    while (!fWiFiManager::isDone()) {
        fGUI::PrintCentered("Starting WiFi", 64, 16);
        fGUI::ProgressBar(64, 36, 96, 16, 0.5);
        fGUI::SetFont(u8g2_font_5x7_tr);

        fGUI::PrintCentered(fWiFiManager::GetStatus(), 64, 54);
        fGUI::Flush();

        delay(750);
        fDebugUtils::beep();
    }

    fDebugUtils::Log("fWifi finished");


    if (fWiFiManager::GetStatus() == "DEFAULT AP") {
        fGUI::Clear();
        fGUI::SetFont(u8g2_font_10x20_tr);
        fGUI::PrintCentered("NO WIFI DATA!", 64, 20);

        fGUI::SetFont(u8g2_font_5x7_tr);
        fGUI::PrintCentered("Setting up AP", 64, 32);
        fGUI::PrintCentered("SSID: ESP_ACCESSPOINT", 64, 48);
        fGUI::PrintCentered("PASS: 123456", 64, 56);
        fGUI::Flush();

        fDebugUtils::error_tone();
        fDebugUtils::error_tone();
        fDebugUtils::error_tone();

        delay(5000);
    }

    if (fWiFiManager::GetStatus() == "FAILOVER AP") {
        fGUI::Clear();
        fGUI::SetFont(u8g2_font_10x20_tr);
        fGUI::PrintCentered("CNCTION FAIL", 64, 20);

        fGUI::SetFont(u8g2_font_5x7_tr);
        fGUI::PrintCentered("Setting up AP", 64, 32);
        //fGUI::PrintCentered("SSID: ESP_ACCESSPOINT", 64, 48);
        //fGUI::PrintCentered("PASS: 123456", 64, 56);
        fGUI::Flush();

        fDebugUtils::error_tone();
        fDebugUtils::error_tone();
        fDebugUtils::error_tone();

        delay(5000);
    }

    fWiFiActions::Begin();

    fGUI::SetFont(u8g2_font_10x20_tr);
    fGUI::PrintCentered("Init done!", 64, 16);
    fGUI::ProgressBar(64, 36, 96, 16, 1);
    fGUI::Flush();

    fGUI::SetFont(u8g2_font_5x7_tr);

    fDebugUtils::success_beep();
}

#endif