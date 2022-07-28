#include "fREST.h"

bool fREST::available = false;
WiFiCommand fREST::commands[64];
WiFiCommand *fREST::currentCommand;
int fREST::numCommands = 0;
TaskHandle_t fREST::handle;
AsyncWebServer fREST::server(80);