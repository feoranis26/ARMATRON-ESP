#include "fGUI.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C fGUI::display(U8G2_R0, U8X8_PIN_NONE);
long fGUI::lastUserInput = 0;
bool fGUI::inMenu = false;

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C fGUI::display(U8G2_R0, U8X8_PIN_NONE);
