#include "../include/main.h"
#include "../lib/IMU.h"
#include "../lib/TwystBackend/TwystBackend.h"
#include "../include/ble_band.h"
#include "../include/battery.h"

IMU imu;
TwystBackendClient backend;
bool backendSessionStarted = false;
String backendResponse;

void setup()
{
  Serial.begin(BAUDRATE);
  delay(2000);
  Serial.println("Initializing...");
  imu.init();
  Serial.println("Keep the IMU still...");
  delay(3000);

  imu.calibrateAccelGyro();
  Serial.println("Calibration done.");

  initBLE();
  Battery::begin();
}

void loop()
{
  bleLoop();

  static uint32_t last = micros();
  uint32_t now = micros();
  float dt = (now - last) / 1000000.0f;
  last = now;

  const float kMaxDt = 0.03f;
  if (dt > kMaxDt || dt <= 0.0f) {
    dt = 0.01f;
  }

  imu.update(dt);

  float ax = imu.getData().ax;
  float ay = imu.getData().ay;
  float az = imu.getData().az;
  float gx = imu.getData().gx;
  float gy = imu.getData().gy;
  float gz = imu.getData().gz;
  float roll = imu.getRoll();
  float pitch = imu.getPitch();
  float yaw = imu.getYaw();

  float battV = Battery::readVoltage();
  Serial.printf("[BATT DEBUG] enabled=%d voltage=%.3f available=%d\n",
                Battery::isEnabled(), battV, Battery::isAvailable());

  char frame[224];
  int len = snprintf(frame, sizeof(frame),
    "ts=%lu acc_x=%.3f acc_y=%.3f acc_z=%.3f "
    "gyro_x=%.3f gyro_y=%.3f gyro_z=%.3f "
    "roll=%.2f pitch=%.2f yaw=%.2f",
    now, ax, ay, az, gx, gy, gz, roll, pitch, yaw);

  if (Battery::isAvailable()) {
    float battPct = Battery::readPercent();
    len += snprintf(frame + len, sizeof(frame) - len,
      " batt_v=%.3f batt_pct=%.1f", battV, battPct);
  }

  if (bleIsConnected()) {
    bleSendFrame(String(frame));
  } else {
    Serial.printf("SEC %s\n", frame);
  }

  delay(100);
}