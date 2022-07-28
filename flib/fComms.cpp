#include "fComms.h"

AsyncUDP fComms::udp;
AsyncClient fComms::tcpClient;
String fComms::id = "";
IPAddress fComms::connected;
TaskHandle_t fComms::BroadcasterTask;
bool fComms::isConnected;
String fComms::status;
int fComms::numCommands;
TCPCommand fComms::commands[64];