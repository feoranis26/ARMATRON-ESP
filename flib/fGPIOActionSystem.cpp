#include "fGPIOActionSystem.h"

bool fGPIOActionSystem::available = false;
GPIOAction fGPIOActionSystem::actions[64];
int fGPIOActionSystem::numActions = 0;
TaskHandle_t fGPIOActionSystem::taskHandle;