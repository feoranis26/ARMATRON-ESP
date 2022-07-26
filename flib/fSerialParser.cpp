#include "fSerialParser.h"

bool fSerialParser::available = false;
int fSerialParser::numCommands = 0;
SerialCommand fSerialParser::commands[64];

TaskHandle_t fSerialParser::taskHandle;