#ifndef SEC_BLE_BAND_H
#define SEC_BLE_BAND_H

#include <Arduino.h>

void initBleClient();
void bleClientLoop();
bool bleSendCommand(const String &command);

#endif