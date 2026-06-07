#include "../include/main.h"
#include "../lib/IMU.h"
#include "../lib/TwystBackend/TwystBackend.h"

IMU imu;
TwystBackendClient backend;
bool backendSessionStarted = false;
String backendResponse;

void setup()
{
  Serial.begin(BAUDRATE);
  // Give serial monitor time to reconnect after reboot
  delay(2000);
  Serial.println("Initializing...");
  imu.init();

// #ifdef WIFI_REPORT
//   backend.setBaseUrl(serverUrl);
//   if (backend.connectWiFi(ssid, password)) {
//     Serial.println("Wi-Fi connected!");
//     Serial.println(WiFi.localIP());

//     // if (backend.startRecord(twystMotionName, &backendResponse)) {
//     //   backendSessionStarted = true;
//     //   Serial.println("Backend recording session started.");
//     // } else {
//     //   Serial.print("Backend startRecord failed: ");
//     //   Serial.println(backendResponse);
//     // }
//   } else {
//     Serial.println("Wi-Fi connection failed.");
//   }
// #endif

  // Serial.println("Starting accel/gyro calibration: keep sensor still...");
  // delay(2000);
  // imu.calibrateAccelGyro();
  // Serial.println("Accel/Gyro calibration done.");

  // Serial.println("Starting magnetometer calibration: wave sensor in figure-eight...");
  // delay(2000);
  // imu.calibrateMag();
  // Serial.println("Mag calibration done.");
}

void loop()
{
// update (dt is unused in current implementation but kept for compatibility)
  imu.update(0.02);

  // Raw counts
  int16_t axr = imu.readAccRawX();
  int16_t ayr = imu.readAccRawY();
  int16_t azr = imu.readAccRawZ();

  int16_t gxr = imu.readGyroRawX();
  int16_t gyr = imu.readGyroRawY();
  int16_t gzr = imu.readGyroRawZ();

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

  // float mx = imu.getData().mx;
  // float my = imu.getData().my;
  // float mz = imu.getData().mz;

  float roll = imu.getRoll();
  float pitch = imu.getPitch();
  float yaw = imu.getYaw();

  float temp = imu.getTemperature();

// #ifdef WIFI_REPORT
//     unsigned long currentMillis = millis();
//     if (currentMillis - previousMillis >= sendInterval) {
//       previousMillis = currentMillis;

//       TwystBackendFrame frame;
//       frame.acc_x = ax;
//       frame.acc_y = ay;
//       frame.acc_z = az;
//       frame.gyro_x = gx;
//       frame.gyro_y = gy;
//       frame.gyro_z = gz;
//       frame.roll = roll;
//       frame.pitch = pitch;
//       frame.yaw = yaw;

//       if (!backend.sendRecordFrame(frame, &backendResponse)) {
//         Serial.print("Backend frame send failed: ");
//         Serial.println(backendResponse);
//       } else
//       {
//         Serial.println("Backend frame sent successfully.");
//       }
//     }
  
// #endif

  Serial.print("RAW Acc: "); Serial.print(axr); Serial.print(","); Serial.print(ayr); Serial.print(","); Serial.print(azr);
  Serial.print("  |  g: "); Serial.print(gxr); Serial.print(","); Serial.print(gyr); Serial.print(","); Serial.print(gzr);
  // Serial.print("  |  mag: "); Serial.print(mxr); Serial.print(","); Serial.print(myr); Serial.print(","); Serial.print(mzr);
  // Serial.print(mag_ok ? "" : "  [read failed]");
  Serial.println();

  Serial.print("Proc Acc[g]: "); Serial.print(ax, 3); Serial.print(", "); Serial.print(ay, 3); Serial.print(", "); Serial.println(az, 3);
  Serial.print("Proc Gyro[deg/s]: "); Serial.print(gx, 3); Serial.print(", "); Serial.print(gy, 3); Serial.print(", "); Serial.println(gz, 3);
  // Serial.print("Proc Mag[mG]: "); Serial.print(mx, 3); Serial.print(", "); Serial.print(my, 3); Serial.print(", "); Serial.println(mz, 3);

  Serial.print("Angles [deg] R/P/Y: "); Serial.print(roll, 2); Serial.print(", "); Serial.print(pitch, 2); Serial.print(", "); Serial.println(yaw, 2);
  Serial.print("Temp [C]: "); Serial.println(temp, 2);

  Serial.println("------------------------------------------------"); 
  delay(100);
}