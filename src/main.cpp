#include "../include/main.h"
#include "../lib/IMU.h"
#include "../lib/TwystBackend/TwystBackend.h"
#include "../include/ble_band.h"

IMU imu;
TwystBackendClient backend;
bool backendSessionStarted = false;
String backendResponse;
String bleSerialBuffer;

namespace {

void handleBleSerialCommand(const String &command)
{
  if (command.length() == 0) {
    return;
  }

  if (command.equalsIgnoreCase("ping") ||
      command.equalsIgnoreCase("status") ||
      command.equalsIgnoreCase("hello")) {
    Serial.printf("[SEC-BLE] USB -> %s\n", command.c_str());
    bleSendCommand(command);
    return;
  }

  Serial.printf("[SEC-BLE] USB command ignored: %s\n", command.c_str());
}

void pollBleSerial()
{
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\r' || character == '\n') {
      bleSerialBuffer.trim();
      handleBleSerialCommand(bleSerialBuffer);
      bleSerialBuffer = "";
      continue;
    }

    bleSerialBuffer += character;
  }
}

} 

void setup()
{
  Serial.begin(BAUDRATE);
  // give serial monitor time to reconnect after reboot
  delay(2000);
  Serial.println("Initializing...");
  imu.init();
  Serial.println("Keep the IMU still...");
delay(3000);

imu.calibrateAccelGyro();

Serial.println("Calibration done.");
  initBleClient();
}

void loop()
{
  pollBleSerial();
  bleClientLoop();

static uint32_t last = micros();

uint32_t now = micros();

float dt = (now - last) / 1000000.0f;

last = now;

const float kMaxDt = 0.03f; // ~3x the nominal 10ms loop
if (dt > kMaxDt || dt <= 0.0f) {
    dt = 0.01f; // fall back to nominal loop period
}

imu.update(dt);

  // Raw counts
  // int16_t axr = imu.readAccRawX();
  // int16_t ayr = imu.readAccRawY();
  // int16_t azr = imu.readAccRawZ();

  // int16_t gxr = imu.readGyroRawX();
  // int16_t gyr = imu.readGyroRawY();
  // int16_t gzr = imu.readGyroRawZ();

  // int16_t mxr = 0;
  // int16_t myr = 0;
  // int16_t mzr = 0;
  // const bool mag_ok = imu.readMagRaw(mxr, myr, mzr);

  // Processed
  float ax = imu.getData().ax;
  float ay = imu.getData().ay;
  float az = imu.getData().az;

  float gx = imu.getData().gx;
  float gy = imu.getData().gy;
  float gz = imu.getData().gz;

  float roll = imu.getRoll();
  float pitch = imu.getPitch();
  float yaw = imu.getYaw();

 // float temp = imu.getTemperature();


// Single-line, bridge-parseable frame. Prefixed "SEC" so esp32_bridge.py
  // can tell this apart from a "MAIN ..." line relayed from the main band,
  // and from "LINK ..." status lines.
  Serial.printf(
"SEC ts=%lu acc_x=%.3f acc_y=%.3f acc_z=%.3f "
"gyro_x=%.3f gyro_y=%.3f gyro_z=%.3f "
"roll=%.2f pitch=%.2f yaw=%.2f\n",
now,
ax, ay, az,
gx, gy, gz,
roll, pitch, yaw
);
 
  delay(100);
}