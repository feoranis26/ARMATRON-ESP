#include "fWiFiServerActionSystem.h"

bool fWiFiActions::available = false;
WiFiCommand fWiFiActions::commands[64];
WiFiCommand *fWiFiActions::currentCommand;
int fWiFiActions::numCommands = 0;
TaskHandle_t fWiFiActions::handle;
AsyncWebServer fWiFiActions::server(80);