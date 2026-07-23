#ifndef SEC_BLE_BAND_H
#define SEC_BLE_BAND_H

#include <Arduino.h>
#include <NimBLEDevice.h>

void initBLE();
void bleLoop();
bool bleIsConnected();
bool bleSendText(const String &text);
bool bleSendFrame(const String &frame);

#endif