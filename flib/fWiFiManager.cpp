#include "fWiFiManager.h"


String fWiFiManager::hostname = "";
String fWiFiManager::ssid = "";
String fWiFiManager::pass = "";
String fWiFiManager::status = "Not started";

bool fWiFiManager::isAP = false;
bool fWiFiManager::isdone = false;
TaskHandle_t fWiFiManager::StartTask;

AsyncWebServer fWiFiManager::server(5001);