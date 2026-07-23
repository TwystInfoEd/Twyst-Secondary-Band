#ifndef MAIN
#define MAIN

#include <Wire.h>
#include <math.h>
#include <Arduino.h>

#include "../lib/IMU.h"

#define WIFI_REPORT

// ESP32-C3 I2C pins
#define SDA_PIN 8
#define SCL_PIN 9

#define BAUDRATE 115200

#ifdef WIFI_REPORT

#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "Orange-5FmR-2.4G";
const char* password = "wNpV8M24";

const char* serverUrl = "http://192.168.100.28:8000";

const char* twystMotionName = "twyst_live_session";
#endif

unsigned long previousMillis = 0;
const long sendInterval = 200;

#endif 