#pragma once
#ifndef fComms_h
#define fComms_h

#include <Arduino.h>
#include "AsyncUDP.h"
#include "AsyncTCP.h"
#include "fDebugUtils.h"
#include "fGUI.h"
//

class TCPCommand {
public:
    String name;
    void (*execute)(String);
    TCPCommand(String n, void (*e)(String)) {
        name = n;
        execute = e;
    }
    TCPCommand() {

    }
};

class fComms {
public:
    static void StartAsTask(String identifier) {
        fDebugUtils::Log("[fComms] Starting with identifier " + identifier + "\nListening on UDP: 11752, TCP: 11751, broadcasting on UDP: 11750");

        udp.listen(11752);

        udp.onPacket([](AsyncUDPPacket packet) {
            String data;

            for (int i = 0; i < packet.length(); i++)
                if((char)packet.data()[i] != '\r')
                    data += (char)packet.data()[i];

            fDebugUtils::Log("[fComms] UDP Packet from" + String(packet.remoteIP()) + ", :" + data);
            UDPSend("RECV " + data, packet.remoteIP());

            OnData(data);

            if (data == "CONNECT") {
                UDPSend("OK\r", packet.remoteIP());

                TryConnectTo(packet.remoteIP());
            }
            }
        );

        tcpClient.onData([](void* param, AsyncClient* client, void* dat, size_t len) {
            String data;

            for (int i = 0; i < len; i++)
                if (((char*)dat)[i] != '\r')
                    data += ((char*)dat)[i];

            fDebugUtils::Log("[fComms] TCP Packet from" + String(client->remoteIP()) + ", :" + data);
            TCPSend("RECV " + data + "\r");

            OnData(data);

            if (data == "DISCONNECT") {
                TCPSend("OK\r");
                Disconnect();
            }
            });

        tcpClient.onTimeout([](void* param, AsyncClient* client, uint32_t time) {
            TryConnectTo(connected);
            });


        tcpClient.onDisconnect([](void* param, AsyncClient* client) {
            isConnected = false;
            fGUI::StartMenu();
            fDebugUtils::shutdown_background();
            for (int i = 0; i < 15; i++) {
                fGUI::SetFont(u8g2_font_10x20_tr, true);

                fGUI::PrintCentered("TCP DSCNCT!", 64, 16, true);
                fGUI::ProgressBar(64, 36, 96, 16, (double)(i) / 15, true, true);

                fGUI::Flush(true);

                delay(50);
            }
            fGUI::EndMenu();
            });

        tcpClient.setAckTimeout(1000);

        xTaskCreate(broadcaster, "fComms", 20000, NULL, 1, &BroadcasterTask);

        id = identifier;
    }

    static String GetStatus() {
        return status;
    }

    static void AddCommand(String name, void (*execute)(String)) {
        commands[numCommands] = TCPCommand(name, execute);
        numCommands++;
    }

    static void UDPSend(String data, IPAddress addr) {
        udp.writeTo((uint8_t*)data.c_str(), data.length(), addr, 11752);
    }

    static void TCPSend(String data) {
        tcpClient.write(data.c_str(), data.length());
    }

private:

    static void broadcaster(void* param) {
        while (true) {
            if (isConnected && !tcpClient.connected())
            {
                TryConnectTo(connected);
            }

            if (!tcpClient.connected()) {
                udp.broadcastTo(("fComms ID:" + id + " AVAILABLE").c_str(), 11750);
                delay(1000);
                status = "NOT CONNECTED";
            }
            else {
                //String dat = "OK";
                //tcpClient.add(dat.c_str(), sizeof(dat.c_str()));
                status = "OK! IP: " + String(connected);
                delay(100);
            }

            isConnected = tcpClient.connected();
        }
    }

    static void TryConnectTo(IPAddress addr) {
        fDebugUtils::Log("[fComms] Connecting to " + addr.toString());

        tcpClient.connect(addr, 11751);

        fGUI::StartMenu();
        long startMillis = millis();
        while (millis() - startMillis < 5000 && !tcpClient.connecting() && !tcpClient.connected()) {
            if (fmod(millis(), 1000) < 100)
                fDebugUtils::beep_background();

            fGUI::Clear(true);
            fGUI::SetFont(u8g2_font_8x13_tr, true);
            fGUI::PrintCentered("TCP CNCTing!", 64, 16, true);

            double secs = (double)millis() / 1000;

            if (fmod(secs, 2) < 1)
                fGUI::ProgressBar(64, 36, 96, 16, fmod(secs, 2), true, true);
            else
                fGUI::ProgressBar(64, 36, 96, 16, 2 - fmod(secs, 2), false, true);

            fGUI::Flush(true);
            delay(20);
        }
        while (tcpClient.connecting()) {
            if (fmod(millis(), 1000) < 100)
                fDebugUtils::beep_background();

            fGUI::Clear(true);
            fGUI::SetFont(u8g2_font_8x13_tr, true);
            fGUI::PrintCentered("TCP CNCTing!", 64, 16, true);

            double secs = (double)millis() / 1000;

            if (fmod(secs, 2) < 1)
                fGUI::ProgressBar(64, 36, 96, 16, fmod(secs, 2), true, true);
            else
                fGUI::ProgressBar(64, 36, 96, 16, 2 - fmod(secs, 2), false, true);

            fGUI::Flush(true);
            delay(20);
        }

        if (!tcpClient.connected()) {
            fGUI::Clear(true);
            fGUI::SetFont(u8g2_font_8x13_tr, true);
            fGUI::PrintCentered("CONNECT FAIL!", 64, 32, true);

            fGUI::Flush(true);

            fDebugUtils::error_tone();
            fDebugUtils::error_tone();
            fDebugUtils::error_tone();
        }
        else {
            fDebugUtils::Log("[fComms] Connected to " + addr.toString());
            connected = addr;

            fGUI::Clear(true);
            fGUI::SetFont(u8g2_font_8x13_tr, true);
            fGUI::PrintCentered("TCP CNCTED!", 64, 16, true);

            fGUI::ProgressBar(64, 36, 96, 16, 1, true, true);
            fGUI::Flush(true);
            fDebugUtils::success_background();
        }

        fGUI::EndMenu();
    }

    static void Disconnect() {
        fDebugUtils::Log("[fComms] Disconnecting from " + connected.toString());

        isConnected = false;
        tcpClient.stop();
    }

    static void OnData(String data) {
        String command;
        String args;

        for (int i = 0; i < data.length(); i++) {
            if (data[i] == ' ') {
                args = data.substring(i + 1);
                break;
            }

            command += data[i];
        }

        fDebugUtils::Log("[fComms] Running command from TCP/UDP console: " + command);

        for (int i = 0; i < numCommands; i++) {
            if (commands[i].name == command) {
#ifdef USE_FGUI
                fGUI::Clear();
                fGUI::SetFont(u8g2_font_5x7_tr);
                fGUI::PrintCentered("Running " + commands[i].name, 64, 32);
                fGUI::Flush();
#endif
                commands[i].execute(args);
                break;
            }
        }
    }


    static IPAddress connected;
    static String id;
    static TaskHandle_t BroadcasterTask;
    static AsyncUDP udp;
    static AsyncClient tcpClient;
    static String status;
    static bool isConnected;
    static int numCommands;
    static TCPCommand commands[64];
};

#endif