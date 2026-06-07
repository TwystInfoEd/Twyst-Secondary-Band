#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// MPU-9265 addresses
#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C

// MPU registers
#define WHO_AM_I 0x75
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define INT_PIN_CFG 0x37
#define MAG_WHO_AM_I 0x00
#define MAG_ST1 0x02
#define MAG_XOUT_L 0x03
#define MAG_ST2 0x09
#define MAG_CNTL1 0x0A

// Constants

// ESP32-C3 I2C pins
#define SDA_PIN 8
#define SCL_PIN 9

// Temperature register
#define TEMP_OUT_H 0x41

struct IMUSensorData {
    float ax, ay, az;  // Accelerometer (g)
    float gx, gy, gz;  // Gyroscope (deg/s)
    float mx, my, mz;  // Magnetometer (mG)
    float temperature; // degrees C
    float roll, pitch, yaw;  // Computed angles (degrees)
};

struct KalmanFilter {
    // Kalman filter parameters
    float Q_angle;
    float Q_bias;
    float R_measure;
    float roll_angle;
    float roll_bias;
    float roll_P[2][2];
    float pitch_angle;
    float pitch_bias;
    float pitch_P[2][2];
    
    KalmanFilter() {
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;
        roll_angle = 0;
        roll_bias = 0;
        pitch_angle = 0;
        pitch_bias = 0;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                roll_P[i][j] = 0;
                pitch_P[i][j] = 0;
            }
        }
    }
};

class IMU {
public:
    IMU();
    bool init();
    // Calibration API
    void calibrateAccelGyro();
    void calibrateMag();
    void update(float dt);
    const IMUSensorData& getData() const;
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    float getTemperature() const;
    // Raw reads
    int16_t readAccRawX();
    int16_t readAccRawY();
    int16_t readAccRawZ();
    int16_t readGyroRawX();
    int16_t readGyroRawY();
    int16_t readGyroRawZ();
    int16_t readMagRawX();
    int16_t readMagRawY();
    int16_t readMagRawZ();
    bool readMagRaw(int16_t &mx_raw, int16_t &my_raw, int16_t &mz_raw);
    // Bias getters
    float getAccBiasX() const;
    float getAccBiasY() const;
    float getAccBiasZ() const;
    float getGyroBiasX() const;
    float getGyroBiasY() const;
    float getGyroBiasZ() const;
    float getMagBiasX() const;
    float getMagBiasY() const;
    float getMagBiasZ() const;
    
private:
    KalmanFilter kalman;
    IMUSensorData sensorData;
    // Calibration values (in sensor native units)
    float acc_bias[3] {0.f, 0.f, 0.f};   // in LSB
    float gyro_bias[3] {0.f, 0.f, 0.f};  // in LSB
    float mag_bias[3] {0.f, 0.f, 0.f};   // in mG
    float mag_scale[3] {1.f, 1.f, 1.f};  // scale correction
    float mag_factory_adjust[3] {1.f, 1.f, 1.f}; // from fuse ROM
    int16_t last_mag_raw[3] {0, 0, 0};
    bool last_mag_valid {false};
    
    int16_t readWord(uint8_t addr, uint8_t reg);
    int16_t readMagWord(uint8_t reg);
    bool readMagBytes(uint8_t reg, uint8_t* dest, uint8_t count);
    bool readMagData(int16_t &mx_raw, int16_t &my_raw, int16_t &mz_raw);
    void kalmanUpdate(float angle, float gyroRate, float dt, float &angle_est, float &bias, float P[2][2]);
    float computeYaw(float ax, float ay, float az, float mx, float my, float mz);
    // helpers
    void initMagFactoryCalibration();
    float getMagResolution(bool m16bit) const;
};

// Implementation
IMU::IMU() {
}

bool IMU::init() {
    Serial.println("Setting up I2C...");
    Wire.begin(SDA_PIN, SCL_PIN);
    // increase I2C speed to help avoid bus timeouts
    Wire.setClock(400000);
    delay(100);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    byte id = 0;
    if (Wire.available())
        id = Wire.read();
    Serial.print("MPU WHO_AM_I: 0x");
    Serial.println(id, HEX);

    // Accept multiple known WHO_AM_I values (MPU6500/9250/9255 family)
    if ((id != 0x70) && (id != 0x71) && (id != 0x73)) {
        Serial.println("Unknown MPU! Stopping for safety.");
        return false;
    }
    Serial.print("MPU detected (WHO_AM_I=0x");
    Serial.print(id, HEX);
    Serial.println(")");

    // Disable the MPU's I2C master block so the host can talk to the AK8963 directly.
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6A); // USER_CTRL
    Wire.write(0x00);
    Wire.endTransmission();

    // Enable bypass mode: connect SDA/SCL directly to the magnetometer bus.
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(INT_PIN_CFG);
    Wire.write(0x22);
    Wire.endTransmission();
    delay(10);
    Serial.println("MPU INT_PIN_CFG set to bypass mode.");

    // // Configure AK8963 (magnetometer): 16-bit continuous measurement 100Hz (0x16)
    // Wire.beginTransmission(MAG_ADDR);
    // Wire.write(MAG_CNTL1);
    // Wire.write(0x16);
    // Wire.endTransmission();
    // delay(10);
    // Serial.println("AK8963 magnetometer configured for 16-bit continuous measurement at 100Hz.");

    // // Read factory mag adjustment values from fuse ROM
    // initMagFactoryCalibration();

    // int16_t test_mx = 0;
    // int16_t test_my = 0;
    // int16_t test_mz = 0;
    // if (readMagData(test_mx, test_my, test_mz)) {
    //     last_mag_raw[0] = test_mx;
    //     last_mag_raw[1] = test_my;
    //     last_mag_raw[2] = test_mz;
    //     last_mag_valid = true;
    //     Serial.println("AK8963 read check passed.");
    // } else {
    //     last_mag_valid = false;
    //     Serial.println("AK8963 read check failed. Verify bypass mode, address 0x0C, and sensor wiring.");
    // }

    Serial.println("IMU initialization complete.");
    return true;
}

int16_t IMU::readWord(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(); // send stop then request
    Wire.requestFrom((uint8_t)addr, (uint8_t)2);
    uint32_t start = millis();
    while (Wire.available() < 2 && (millis() - start) < 20) {
        delay(1);
    }
    if (Wire.available() >= 2) {
        return (int16_t)((Wire.read() << 8) | Wire.read());
    }
    return 0;
}

int16_t IMU::readMagWord(uint8_t reg) {
    uint8_t data[2] = {0, 0};
    if (readMagBytes(reg, data, 2)) {
        return (int16_t)((data[1] << 8) | data[0]);
    }
    return 0;
}

bool IMU::readMagBytes(uint8_t reg, uint8_t* dest, uint8_t count) {
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
        Wire.beginTransmission(MAG_ADDR);
        Wire.write(reg);
        if (attempt == 0) {
            Wire.endTransmission(false);
        } else {
            Wire.endTransmission();
        }

        uint8_t received = Wire.requestFrom((uint8_t)MAG_ADDR, count);
        if (received == count) {
            uint8_t index = 0;
            while (Wire.available() && index < count) {
                dest[index++] = Wire.read();
            }
            if (index == count) {
                return true;
            }
        }
        delay(2);
    }
    return false;
}

bool IMU::readMagData(int16_t &mx_raw, int16_t &my_raw, int16_t &mz_raw) {
    uint8_t st1_data[1] = {0};
    if (!readMagBytes(MAG_ST1, st1_data, 1)) {
        return false;
    }

    const uint8_t st1 = st1_data[0];
    if ((st1 & 0x01) == 0) {
        return false;
    }

    uint8_t raw_data[7] = {0};
    if (!readMagBytes(MAG_XOUT_L, raw_data, 7)) {
        return false;
    }

    const uint8_t st2 = raw_data[6];
    if (st2 & 0x08) {
        return false;
    }

    mx_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    my_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    mz_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    return true;
}

void IMU::initMagFactoryCalibration() {
    // enter fuse ROM access mode
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x0A); // CNTL1
    Wire.write(0x00); // power down
    Wire.endTransmission();
    Serial.println("AK8963 magnetometer powered down for factory calibration read.");
    delay(10);
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x0A);
    Wire.write(0x0F); // fuse ROM access
    Wire.endTransmission();
    Serial.println("AK8963 magnetometer set to fuse ROM access mode for factory calibration read.");
    delay(10);
    // ASAX, ASAY, ASAZ at 0x10..0x12
    uint8_t factory_data[3] = {0, 0, 0};
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (readMagBytes(0x10, factory_data, 3)) {
            mag_factory_adjust[0] = ((float)factory_data[0] - 128.0f) / 256.0f + 1.0f;
            mag_factory_adjust[1] = ((float)factory_data[1] - 128.0f) / 256.0f + 1.0f;
            mag_factory_adjust[2] = ((float)factory_data[2] - 128.0f) / 256.0f + 1.0f;
            break;
        }
        delay(5);
    }
    Serial.print("Mag factory adjustment values: ");
    Serial.print(mag_factory_adjust[0], 4);
    Serial.print(", ");
    Serial.print(mag_factory_adjust[1], 4);
    Serial.print(", ");
    Serial.println(mag_factory_adjust[2], 4);
    // power down and set to continuous mode again
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x0A);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x0A);
    Wire.write(0x16); // 16-bit continuous 100Hz
    Wire.endTransmission();
    delay(10);
}

float IMU::getMagResolution(bool m16bit) const {
    if (m16bit) return 10.0f * 4912.0f / 32760.0f; // mG per LSB
    return 10.0f * 4912.0f / 8190.0f;
}

void IMU::kalmanUpdate(float angle, float gyroRate, float dt, float &angle_est, float &bias, float P[2][2]) {
    angle_est += dt * (gyroRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + kalman.Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += kalman.Q_bias * dt;

    float y = angle - angle_est;
    float S = P[0][0] + kalman.R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};
    angle_est += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}

float IMU::computeYaw(float ax, float ay, float az, float mx, float my, float mz) {
    float rollRad = atan2(ay, az);
    float pitchRad = atan(-ax / sqrt(ay * ay + az * az));

    float mx_comp = mx * cos(pitchRad) + mz * sin(pitchRad);
    float my_comp = mx * sin(rollRad) * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);

    float yawRad = atan2(-my_comp, mx_comp);
    return yawRad * 180.0 / PI;
}

void IMU::update(float dt) {
    int16_t ax_raw = readWord(MPU_ADDR, ACCEL_XOUT_H);
    int16_t ay_raw = readWord(MPU_ADDR, ACCEL_XOUT_H + 2);
    int16_t az_raw = readWord(MPU_ADDR, ACCEL_XOUT_H + 4);

    int16_t gx_raw = readWord(MPU_ADDR, GYRO_XOUT_H);
    int16_t gy_raw = readWord(MPU_ADDR, GYRO_XOUT_H + 2);
    int16_t gz_raw = readWord(MPU_ADDR, GYRO_XOUT_H + 4);

    // apply accel/gyro biases (stored in LSB counts)
    int16_t ax_cal = ax_raw - (int16_t)acc_bias[0];
    int16_t ay_cal = ay_raw - (int16_t)acc_bias[1];
    int16_t az_cal = az_raw - (int16_t)acc_bias[2];

    int16_t gx_cal = gx_raw - (int16_t)gyro_bias[0];
    int16_t gy_cal = gy_raw - (int16_t)gyro_bias[1];
    int16_t gz_cal = gz_raw - (int16_t)gyro_bias[2];

    sensorData.ax = (float)ax_cal / 16384.0f; // g
    sensorData.ay = (float)ay_cal / 16384.0f;
    sensorData.az = (float)az_cal / 16384.0f;

    sensorData.gx = (float)gx_cal / 131.0f; // deg/s
    sensorData.gy = (float)gy_cal / 131.0f;
    sensorData.gz = (float)gz_cal / 131.0f;

    // // Magnetometer: read raw, apply factory adjust, subtract bias and apply scale
    // int16_t mx_raw = 0;
    // int16_t my_raw = 0;
    // int16_t mz_raw = 0;
    // if (readMagData(mx_raw, my_raw, mz_raw)) {
    //     float mres = getMagResolution(true); // assume 16-bit
    //     sensorData.mx = ((float)mx_raw * mres * mag_factory_adjust[0] - mag_bias[0]) * mag_scale[0];
    //     sensorData.my = ((float)my_raw * mres * mag_factory_adjust[1] - mag_bias[1]) * mag_scale[1];
    //     sensorData.mz = ((float)mz_raw * mres * mag_factory_adjust[2] - mag_bias[2]) * mag_scale[2];
    // }

    // temperature
    int16_t temp_count = readWord(MPU_ADDR, TEMP_OUT_H);
    sensorData.temperature = ((float)temp_count) / 333.87f + 21.0f;

    float rollAcc = atan2(sensorData.ay, sensorData.az) * 180.0 / PI;
    float pitchAcc = atan(-sensorData.ax / sqrt(sensorData.ay * sensorData.ay + sensorData.az * sensorData.az)) * 180.0 / PI;

    kalmanUpdate(rollAcc, sensorData.gx, dt, kalman.roll_angle, kalman.roll_bias, kalman.roll_P);
    kalmanUpdate(pitchAcc, sensorData.gy, dt, kalman.pitch_angle, kalman.pitch_bias, kalman.pitch_P);
    
    sensorData.roll = kalman.roll_angle;
    sensorData.pitch = kalman.pitch_angle;

    sensorData.yaw = computeYaw(sensorData.ax, sensorData.ay, sensorData.az, sensorData.mx, sensorData.my, sensorData.mz);
}

// Raw read helpers
int16_t IMU::readAccRawX() { return readWord(MPU_ADDR, ACCEL_XOUT_H); }
int16_t IMU::readAccRawY() { return readWord(MPU_ADDR, ACCEL_XOUT_H + 2); }
int16_t IMU::readAccRawZ() { return readWord(MPU_ADDR, ACCEL_XOUT_H + 4); }

int16_t IMU::readGyroRawX() { return readWord(MPU_ADDR, GYRO_XOUT_H); }
int16_t IMU::readGyroRawY() { return readWord(MPU_ADDR, GYRO_XOUT_H + 2); }
int16_t IMU::readGyroRawZ() { return readWord(MPU_ADDR, GYRO_XOUT_H + 4); }

int16_t IMU::readMagRawX() {
    int16_t mx_raw = 0;
    int16_t my_raw = 0;
    int16_t mz_raw = 0;
    if (!last_mag_valid) {
        readMagRaw(mx_raw, my_raw, mz_raw);
    }
    return last_mag_valid ? last_mag_raw[0] : 0;
}

int16_t IMU::readMagRawY() {
    int16_t mx_raw = 0;
    int16_t my_raw = 0;
    int16_t mz_raw = 0;
    if (!last_mag_valid) {
        readMagRaw(mx_raw, my_raw, mz_raw);
    }
    return last_mag_valid ? last_mag_raw[1] : 0;
}

int16_t IMU::readMagRawZ() {
    int16_t mx_raw = 0;
    int16_t my_raw = 0;
    int16_t mz_raw = 0;
    if (!last_mag_valid) {
        readMagRaw(mx_raw, my_raw, mz_raw);
    }
    return last_mag_valid ? last_mag_raw[2] : 0;
}

bool IMU::readMagRaw(int16_t &mx_raw, int16_t &my_raw, int16_t &mz_raw) {
    if (readMagData(mx_raw, my_raw, mz_raw)) {
        last_mag_raw[0] = mx_raw;
        last_mag_raw[1] = my_raw;
        last_mag_raw[2] = mz_raw;
        last_mag_valid = true;
        return true;
    }
    return false;
}

// Bias getters (physical units)
float IMU::getAccBiasX() const { return acc_bias[0] / 16384.0f; }
float IMU::getAccBiasY() const { return acc_bias[1] / 16384.0f; }
float IMU::getAccBiasZ() const { return acc_bias[2] / 16384.0f; }

float IMU::getGyroBiasX() const { return gyro_bias[0] / 131.0f; }
float IMU::getGyroBiasY() const { return gyro_bias[1] / 131.0f; }
float IMU::getGyroBiasZ() const { return gyro_bias[2] / 131.0f; }

float IMU::getMagBiasX() const { return mag_bias[0]; }
float IMU::getMagBiasY() const { return mag_bias[1]; }
float IMU::getMagBiasZ() const { return mag_bias[2]; }

// Calibration implementations
void IMU::calibrateAccelGyro() {
    const uint16_t sample_count = 200;
    int64_t acc_sum[3] = {0, 0, 0};
    int64_t gyro_sum[3] = {0, 0, 0};
    for (uint16_t i = 0; i < sample_count; ++i) {
        int16_t ax = readAccRawX();
        int16_t ay = readAccRawY();
        int16_t az = readAccRawZ();
        int16_t gx = readGyroRawX();
        int16_t gy = readGyroRawY();
        int16_t gz = readGyroRawZ();
        acc_sum[0] += ax; acc_sum[1] += ay; acc_sum[2] += az;
        gyro_sum[0] += gx; gyro_sum[1] += gy; gyro_sum[2] += gz;
        delay(5);
    }
    float acc_avg[3];
    float gyro_avg[3];
    for (int i = 0; i < 3; ++i) {
        acc_avg[i] = (float)acc_sum[i] / (float)sample_count;
        gyro_avg[i] = (float)gyro_sum[i] / (float)sample_count;
    }
    // Remove gravity from Z accel average
    if (acc_avg[2] > 0) acc_avg[2] -= 16384.0f; else acc_avg[2] += 16384.0f;

    // store biases in LSB counts
    acc_bias[0] = acc_avg[0]; acc_bias[1] = acc_avg[1]; acc_bias[2] = acc_avg[2];
    gyro_bias[0] = gyro_avg[0]; gyro_bias[1] = gyro_avg[1]; gyro_bias[2] = gyro_avg[2];
}

void IMU::calibrateMag() {
    // collect samples while user waves device in figure-eight
    const uint16_t sample_count = 500;
    int16_t mag_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
    int16_t mag_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
    uint16_t collected = 0;
    for (uint16_t attempts = 0; collected < sample_count && attempts < sample_count * 3; ++attempts) {
        int16_t mx = 0;
        int16_t my = 0;
        int16_t mz = 0;
        if (!readMagData(mx, my, mz)) {
            delay(5);
            continue;
        }
        if (mx > mag_max[0]) mag_max[0] = mx;
        if (my > mag_max[1]) mag_max[1] = my;
        if (mz > mag_max[2]) mag_max[2] = mz;
        if (mx < mag_min[0]) mag_min[0] = mx;
        if (my < mag_min[1]) mag_min[1] = my;
        if (mz < mag_min[2]) mag_min[2] = mz;
        ++collected;
        delay(12);
    }
    // hard iron bias (in mG)
    float mres = getMagResolution(true);
    mag_bias[0] = ((float)(mag_max[0] + mag_min[0]) / 2.0f) * mres * mag_factory_adjust[0];
    mag_bias[1] = ((float)(mag_max[1] + mag_min[1]) / 2.0f) * mres * mag_factory_adjust[1];
    mag_bias[2] = ((float)(mag_max[2] + mag_min[2]) / 2.0f) * mres * mag_factory_adjust[2];
    // soft iron scale
    float sx = ((float)(mag_max[0] - mag_min[0]) * mag_factory_adjust[0]) / 2.0f;
    float sy = ((float)(mag_max[1] - mag_min[1]) * mag_factory_adjust[1]) / 2.0f;
    float sz = ((float)(mag_max[2] - mag_min[2]) * mag_factory_adjust[2]) / 2.0f;
    float avg = (sx + sy + sz) / 3.0f;
    mag_scale[0] = avg / sx;
    mag_scale[1] = avg / sy;
    mag_scale[2] = avg / sz;
}

const IMUSensorData& IMU::getData() const {
    return sensorData;
}

float IMU::getRoll() const {
    return sensorData.roll;
}

float IMU::getPitch() const {
    return sensorData.pitch;
}

float IMU::getYaw() const {
    return sensorData.yaw;
}

float IMU::getTemperature() const {
    return sensorData.temperature;
}

#endif // IMU_H