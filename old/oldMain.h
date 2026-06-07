#include <main.h>

void setup()
{
  Serial.begin(BAUDRATE);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("Initializing...");
  delay(10000);

  Serial.println("Setting up I2C...");

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

#ifdef WIFI_REPORT
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    Serial.println(WiFi.status());
  }

  Serial.println("\nWi-Fi connected!");
  Serial.println(WiFi.localIP());
#endif

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  byte id = 0;
  if (Wire.available())
    id = Wire.read();
  Serial.print("MPU WHO_AM_I: 0x");
  Serial.println(id, HEX);

  if (id != 0x70)
  {
    Serial.println("Unknown MPU! Stopping for safety.");
    while (1)
      delay(1000);
  }
  Serial.println("MPU-9265 detected.");

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.endTransmission();
  Serial.println("MPU-9265 INT_PIN_CFG read.");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();
  Serial.println("MPU-9265 INT_PIN_CFG set to bypass mode.");
  delay(10);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MAG_CNTL1);
  Wire.write(0x16);
  Wire.endTransmission();
  delay(10);

  Serial.println("Initialization complete.");
}

int16_t readWord(uint8_t addr, uint8_t reg)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2);
  if (Wire.available() >= 2)
    return (Wire.read() << 8) | Wire.read();
  return 0;
}

int16_t readMagWord(uint8_t reg)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 2);
  if (Wire.available() >= 2)
  {
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    return (hi << 8) | lo;
  }
  return 0;
}

// Simple 1D Kalman filter
float kalmanUpdate(float angle, float gyroRate, float dt, float &angle_est, float &bias, float P[2][2])
{
  angle_est += dt * (gyroRate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float y = angle - angle_est;
  float S = P[0][0] + R_measure;
  float K[2] = {P[0][0] / S, P[1][0] / S};
  angle_est += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle_est;
}

// Tilt-compensated yaw
float computeYaw(float ax, float ay, float az, float mx, float my, float mz)
{
  float rollRad = atan2(ay, az);
  float pitchRad = atan(-ax / sqrt(ay * ay + az * az));

  float mx_comp = mx * cos(pitchRad) + mz * sin(pitchRad);
  float my_comp = mx * sin(rollRad) * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);

  float yawRad = atan2(-my_comp, mx_comp);
  return yawRad * 180.0 / PI;
}

#ifdef WIFI_REPORT

void reportStatus(float roll, float pitch, float yaw)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    String payload = "{";
    payload += "\"roll\":" + String(roll, 2) + ",";
    payload += "\"pitch\":" + String(pitch, 2) + ",";
    payload += "\"yaw\":" + String(yaw, 2);
    payload += "}";

    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0)
    {
      Serial.print("Data sent, response code: ");
      Serial.println(httpResponseCode);
      Serial.println(payload);
    }
    else
    {
      Serial.print("Error sending data: ");
      Serial.println(http.errorToString(httpResponseCode));
    }
    http.end();
  }
}

#endif

void loop()
{
  Serial.println(1);
  float dt = 0.01;

  int16_t ax_raw = readWord(MPU_ADDR, ACCEL_XOUT_H);
  int16_t ay_raw = readWord(MPU_ADDR, ACCEL_XOUT_H + 2);
  int16_t az_raw = readWord(MPU_ADDR, ACCEL_XOUT_H + 4);

    Serial.println(2);


  int16_t gx_raw = readWord(MPU_ADDR, GYRO_XOUT_H);
  int16_t gy_raw = readWord(MPU_ADDR, GYRO_XOUT_H + 2);
  int16_t gz_raw = readWord(MPU_ADDR, GYRO_XOUT_H + 4);

    Serial.println(3);


  float ax = ax_raw / 16384.0;
  float ay = ay_raw / 16384.0;
  float az = az_raw / 16384.0;

  float gx = gx_raw / 131.0;
  float gy = gy_raw / 131.0;
  float gz = gz_raw / 131.0;

    Serial.println(4);


  float rollAcc = atan2(ay, az) * 180.0 / PI;
  float pitchAcc = atan(-ax / sqrt(ay * ay + az * az)) * 180.0 / PI;

    Serial.println(5);


  float roll = kalmanUpdate(rollAcc, gx, dt, roll_angle, roll_bias, roll_P);
  float pitch = kalmanUpdate(pitchAcc, gy, dt, pitch_angle, pitch_bias, pitch_P);

    Serial.println(6);


  unsigned long currentMillis = millis();
    Serial.println(7);

  if (currentMillis - previousMillis >= sendInterval)
  {
    previousMillis = currentMillis;

    Serial.print("Roll: ");
    Serial.print(roll, 2);
    Serial.print(" | Pitch: ");
    Serial.print(pitch, 2);

      Serial.println(8);


#ifdef WIFI_REPORT
    reportStatus(roll, pitch, 0);
      Serial.println(9);

#endif
  }

  delay(10);
}