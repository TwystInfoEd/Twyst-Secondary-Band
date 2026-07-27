#ifndef IMU_MAIN_H
#define IMU_MAIN_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#define PIN_SPI_MOSI   11
#define PIN_SPI_MISO   13
#define PIN_SPI_CLK    12
#define PIN_IMU_CS     39   
#define PIN_UWB_CS     10   
#define PIN_I2C_SDA    17
#define PIN_I2C_SCL    18

#define BMI270_REG_CHIP_ID        0x00  // expect 0x24
#define BMI270_REG_ERR_REG        0x02
#define BMI270_REG_STATUS         0x03
#define BMI270_REG_DATA_ACC_X_LSB 0x0C  
#define BMI270_REG_INTERNAL_STATUS 0x21 
#define BMI270_REG_ACC_CONF       0x40
#define BMI270_REG_ACC_RANGE      0x41
#define BMI270_REG_GYR_CONF       0x42
#define BMI270_REG_GYR_RANGE      0x43
#define BMI270_REG_INIT_CTRL      0x59
#define BMI270_REG_INIT_ADDR_0    0x5B
#define BMI270_REG_INIT_ADDR_1    0x5C
#define BMI270_REG_INIT_DATA      0x5E
#define BMI270_REG_PWR_CONF       0x7C
#define BMI270_REG_PWR_CTRL       0x7D
#define BMI270_REG_CMD            0x7E

#define BMI270_CMD_SOFTRESET      0xB6
#define BMI270_SPI_READ_BIT       0x80

// per spec section 3.1: accelerometer +-8g, gyroscope +-1000 deg/s.
#define BMI270_ACC_LSB_PER_G      4096.0f
#define BMI270_GYR_LSB_PER_DPS    32.8f

#define LIS3MDL_ADDR_SDO_HIGH  0x1E
#define LIS3MDL_ADDR_SDO_LOW   0x1C
#define LIS3MDL_REG_WHO_AM_I   0x0F  // expect 0x3D
#define LIS3MDL_REG_CTRL1      0x20
#define LIS3MDL_REG_CTRL2      0x21
#define LIS3MDL_REG_CTRL3      0x22
#define LIS3MDL_REG_CTRL4      0x23
#define LIS3MDL_REG_STATUS     0x27
#define LIS3MDL_REG_OUT_X_L    0x28 

#define LIS3MDL_LSB_PER_GAUSS  6842.0f
#include "bmi270_config.h"

struct IMUSensorData {
    float ax, ay, az;   // g
    float gx, gy, gz;   // deg/s
    float mx, my, mz;   // gauss
    float temperature;  // deg C (not wired in this file — BMI270 has an internal temp
                         // register at 0x22/0x23 if needed; omitted for brevity)
    float roll, pitch, yaw;
};

// 9-DOF Madgwick filter state (adds magnetometer-based yaw correction on
// top of the 6-DOF accel+gyro fusion used on the secondary band)
struct MadgwickFilter9 {
    float beta;
    float q0, q1, q2, q3;
    MadgwickFilter9() {
        beta = 0.1f;
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    }
};

class IMU {
public:
    IMU();
    bool init();
    void calibrateAccelGyro();
    void calibrateMag();
    void update(float dt);
    const IMUSensorData& getData() const;
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

private:
    MadgwickFilter9 madgwick;
    IMUSensorData sensorData;
    float acc_bias[3] {0.f, 0.f, 0.f}; // LSB
    float gyro_bias[3] {0.f, 0.f, 0.f}; // LSB
    float mag_bias[3] {0.f, 0.f, 0.f};  // gauss (hard iron)
    float mag_scale[3] {1.f, 1.f, 1.f}; // soft iron
    uint8_t lis3mdl_addr = LIS3MDL_ADDR_SDO_HIGH;

    // --- BMI270 (SPI) low-level ---
    void bmiCsLow()  { digitalWrite(PIN_IMU_CS, LOW); }
    void bmiCsHigh() { digitalWrite(PIN_IMU_CS, HIGH); }
    uint8_t bmiReadReg(uint8_t reg);
    void bmiReadRegs(uint8_t reg, uint8_t *dest, uint16_t len);
    void bmiWriteReg(uint8_t reg, uint8_t val);
    bool bmiLoadConfigFile();
    bool bmiConfigureRangesAndODR();

    // --- LIS3MDL (I2C) low-level ---
    bool lisWriteReg(uint8_t reg, uint8_t val);
    bool lisReadRegs(uint8_t reg, uint8_t *dest, uint8_t len);
    bool lisDetectAddress();  // tries both possible SDO-strap addresses
    bool lisReadMagRaw(int16_t &mx, int16_t &my, int16_t &mz);

    void madgwickUpdate9DOF(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz, float dt);
    void quaternionToEuler();
};

IMU::IMU() {}

// BMI270 low-level SPI
uint8_t IMU::bmiReadReg(uint8_t reg) {
    uint8_t val = 0;
    bmiReadRegs(reg, &val, 1);
    return val;
}

void IMU::bmiReadRegs(uint8_t reg, uint8_t *dest, uint16_t len) {
    bmiCsLow();
    SPI.transfer(reg | BMI270_SPI_READ_BIT);
    SPI.transfer(0x00); // BMI270 SPI read protocol: one dummy byte after the address byte
    for (uint16_t i = 0; i < len; ++i) {
        dest[i] = SPI.transfer(0x00);
    }
    bmiCsHigh();
}

void IMU::bmiWriteReg(uint8_t reg, uint8_t val) {
    bmiCsLow();
    SPI.transfer(reg & 0x7F); // write: MSB = 0
    SPI.transfer(val);
    bmiCsHigh();
}

bool IMU::bmiLoadConfigFile() {
    if (bmi270_config_file_len != 8192) {
        Serial.println("[MAIN] BMI270 config blob missing or wrong size — "
                        "sensor will NOT produce valid data. See IMU_main_bmi270_lis3mdl.h header.");
        return false;
    }

    bmiWriteReg(BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
    delay(5);

    bmiWriteReg(BMI270_REG_PWR_CONF, 0x00);
    delay(1);
    bmiWriteReg(BMI270_REG_INIT_CTRL, 0x00);

    const uint16_t chunkSize = 256;
    for (uint16_t offset = 0; offset < bmi270_config_file_len; offset += chunkSize) {
        uint16_t addrWord = offset / 2;
        bmiWriteReg(BMI270_REG_INIT_ADDR_0, (uint8_t)(addrWord & 0x0F));
        bmiWriteReg(BMI270_REG_INIT_ADDR_1, (uint8_t)(addrWord >> 4));
        bmiCsLow();
        SPI.transfer(BMI270_REG_INIT_DATA & 0x7F);
        for (uint16_t i = 0; i < chunkSize && (offset + i) < bmi270_config_file_len; ++i) {
            SPI.transfer(bmi270_config_file[offset + i]);
        }
        bmiCsHigh();
    }

    bmiWriteReg(BMI270_REG_INIT_CTRL, 0x01);
    delay(20);

    uint8_t status = bmiReadReg(BMI270_REG_INTERNAL_STATUS);
    if ((status & 0x0F) != 0x01) {
        Serial.print("[MAIN] BMI270 init failed, INTERNAL_STATUS=0x");
        Serial.println(status, HEX);
        return false;
    }
    Serial.println("[MAIN] BMI270 config file loaded OK.");
    return true;
}

bool IMU::bmiConfigureRangesAndODR() {
    bmiWriteReg(BMI270_REG_ACC_RANGE, 0x03);
    bmiWriteReg(BMI270_REG_ACC_CONF, 0xA8); 

    bmiWriteReg(BMI270_REG_GYR_RANGE, 0x01);
    bmiWriteReg(BMI270_REG_GYR_CONF, 0xA9); 
   
    bmiWriteReg(BMI270_REG_PWR_CTRL, 0x0E);
    delay(10);
    return true;
}


bool IMU::lisWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(lis3mdl_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool IMU::lisReadRegs(uint8_t reg, uint8_t *dest, uint8_t len) {
    Wire.beginTransmission(lis3mdl_addr);
    Wire.write(reg | 0x80); 
    if (Wire.endTransmission(false) != 0) return false;
    uint8_t received = Wire.requestFrom(lis3mdl_addr, len);
    if (received != len) return false;
    for (uint8_t i = 0; i < len; ++i) dest[i] = Wire.read();
    return true;
}

bool IMU::lisDetectAddress() {
    for (uint8_t candidate : {LIS3MDL_ADDR_SDO_HIGH, LIS3MDL_ADDR_SDO_LOW}) {
        lis3mdl_addr = candidate;
        uint8_t id = 0;
        if (lisReadRegs(LIS3MDL_REG_WHO_AM_I, &id, 1) && id == 0x3D) {
            Serial.print("[MAIN] LIS3MDL found at 0x");
            Serial.println(candidate, HEX);
            return true;
        }
    }
    Serial.println("[MAIN] LIS3MDL not found at 0x1E or 0x1C — check SDO/SA1 strap.");
    return false;
}

bool IMU::lisReadMagRaw(int16_t &mx, int16_t &my, int16_t &mz) {
    uint8_t status = 0;
    if (!lisReadRegs(LIS3MDL_REG_STATUS, &status, 1)) return false;
    if ((status & 0x08) == 0) return false;
    uint8_t raw[6];
    if (!lisReadRegs(LIS3MDL_REG_OUT_X_L, raw, 6)) return false;
    mx = (int16_t)((raw[1] << 8) | raw[0]);
    my = (int16_t)((raw[3] << 8) | raw[2]);
    mz = (int16_t)((raw[5] << 8) | raw[4]);
    return true;
}

bool IMU::init() {
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_IMU_CS);
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); 

    uint8_t chipId = bmiReadReg(BMI270_REG_CHIP_ID);
    Serial.print("[MAIN] BMI270 CHIP_ID: 0x");
    Serial.println(chipId, HEX); // expect 0x24
    if (chipId != 0x24) {
        Serial.println("[MAIN] BMI270 not detected on SPI bus.");
        return false;
    }

    if (!bmiLoadConfigFile()) return false;
    bmiConfigureRangesAndODR();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000); // LIS3MDL supports Fast-Mode 400kHz per spec

    if (!lisDetectAddress()) {
        Serial.println("[MAIN] Continuing without magnetometer — yaw will drift (gyro-only).");
    } else {
        lisWriteReg(LIS3MDL_REG_CTRL1, 0x70); // temp comp on, ultra-high perf XY, ODR ~80Hz
        lisWriteReg(LIS3MDL_REG_CTRL2, 0x00); // +-4 gauss full scale
        lisWriteReg(LIS3MDL_REG_CTRL3, 0x00); // continuous-conversion mode
        lisWriteReg(LIS3MDL_REG_CTRL4, 0x0C); // ultra-high perf Z axis
    }

    Serial.println("[MAIN] IMU init complete.");
    return true;
}

// Madgwick 9-DOF (adds magnetometer correction vs. the 6-DOF secondary-band version)

void IMU::madgwickUpdate9DOF(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float mx, float my, float mz, float dt) {
    float q0 = madgwick.q0, q1 = madgwick.q1, q2 = madgwick.q2, q3 = madgwick.q3;

    float qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    float accNorm = sqrtf(ax*ax + ay*ay + az*az);
    float magNorm = sqrtf(mx*mx + my*my + mz*mz);

    if (accNorm > 1e-6f && magNorm > 1e-6f) {
        ax/=accNorm; ay/=accNorm; az/=accNorm;
        mx/=magNorm; my/=magNorm; mz/=magNorm;

        float _2q0mx=2.0f*q0*mx, _2q0my=2.0f*q0*my, _2q0mz=2.0f*q0*mz;
        float _2q1mx=2.0f*q1*mx;
        float _2q0=2.0f*q0, _2q1=2.0f*q1, _2q2=2.0f*q2, _2q3=2.0f*q3;
        float _2q0q2=2.0f*q0*q2, _2q2q3=2.0f*q2*q3;
        float q0q0=q0*q0, q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
        float q1q1=q1*q1, q1q2=q1*q2, q1q3=q1*q3;
        float q2q2=q2*q2, q2q3=q2*q3, q3q3=q3*q3;

        float hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
        float hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
        float _2bx = sqrtf(hx*hx + hy*hy);
        float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;
        float _4bx = 2.0f*_2bx, _4bz = 2.0f*_2bz;

        float f1 = _2q1*q3 - _2q0*q2 - ax;
        float f2 = _2q0*q1 + _2q2*q3 - ay;
        float f3 = 1.0f - _2q1*q1 - _2q2*q2 - az;
        float f4 = _2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx;
        float f5 = _2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my;
        float f6 = _2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz;

        float s0 = -_2q2*f1 + _2q1*f2 - _2bz*q2*f4 + (-_2bx*q3+_2bz*q1)*f5 + _2bx*q2*f6;
        float s1 =  _2q3*f1 + _2q0*f2 - 4.0f*q1*f3 + _2bz*q3*f4 + (_2bx*q2+_2bz*q0)*f5 + (_2bx*q3-_4bz*q1)*f6;
        float s2 = -_2q0*f1 + _2q3*f2 - 4.0f*q2*f3 + (-_4bx*q2-_2bz*q0)*f4 + (_2bx*q1+_2bz*q3)*f5 + (_2bx*q0-_4bz*q2)*f6;
        float s3 =  _2q1*f1 + _2q2*f2 + (-_4bx*q3+_2bz*q1)*f4 + (-_2bx*q0+_2bz*q2)*f5 + _2bx*q1*f6;

        float sNorm = sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);
        if (sNorm > 1e-6f) {
            s0/=sNorm; s1/=sNorm; s2/=sNorm; s3/=sNorm;
            qDot1 -= madgwick.beta*s0;
            qDot2 -= madgwick.beta*s1;
            qDot3 -= madgwick.beta*s2;
            qDot4 -= madgwick.beta*s3;
        }
    } else if (accNorm > 1e-6f) {
        // Mag unavailable this cycle — fall back to 6-DOF accel-only correction
        // so a transient magnetometer read failure doesn't stall orientation updates.
        ax/=accNorm; ay/=accNorm; az/=accNorm;
        float f1 = 2.0f*(q1*q3-q0*q2) - ax;
        float f2 = 2.0f*(q0*q1+q2*q3) - ay;
        float f3 = 2.0f*(0.5f-q1*q1-q2*q2) - az;
        float s0 = -2.0f*q2*f1 + 2.0f*q1*f2;
        float s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 4.0f*q1*f3;
        float s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3;
        float s3 =  2.0f*q1*f1 + 2.0f*q2*f2;
        float sNorm = sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);
        if (sNorm > 1e-6f) {
            s0/=sNorm; s1/=sNorm; s2/=sNorm; s3/=sNorm;
            qDot1 -= madgwick.beta*s0;
            qDot2 -= madgwick.beta*s1;
            qDot3 -= madgwick.beta*s2;
            qDot4 -= madgwick.beta*s3;
        }
    }

    q0 += qDot1*dt; q1 += qDot2*dt; q2 += qDot3*dt; q3 += qDot4*dt;
    float qNorm = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    madgwick.q0=q0/qNorm; madgwick.q1=q1/qNorm; madgwick.q2=q2/qNorm; madgwick.q3=q3/qNorm;
}

void IMU::quaternionToEuler() {
    float q0=madgwick.q0, q1=madgwick.q1, q2=madgwick.q2, q3=madgwick.q3;
    sensorData.roll = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2)) * 180.0f/PI;
    float sinp = 2.0f*(q0*q2-q3*q1);
    sinp = constrain(sinp, -1.0f, 1.0f);
    sensorData.pitch = asinf(sinp) * 180.0f/PI;
    sensorData.yaw = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3)) * 180.0f/PI;
}

void IMU::update(float dt) {
    uint8_t raw[12];
    bmiReadRegs(BMI270_REG_DATA_ACC_X_LSB, raw, 12); // ACC x,y,z then GYR x,y,z, 2B each

    int16_t ax_raw = (int16_t)((raw[1]<<8) | raw[0]);
    int16_t ay_raw = (int16_t)((raw[3]<<8) | raw[2]);
    int16_t az_raw = (int16_t)((raw[5]<<8) | raw[4]);
    int16_t gx_raw = (int16_t)((raw[7]<<8) | raw[6]);
    int16_t gy_raw = (int16_t)((raw[9]<<8) | raw[8]);
    int16_t gz_raw = (int16_t)((raw[11]<<8) | raw[10]);

    int16_t ax_cal = ax_raw - (int16_t)acc_bias[0];
    int16_t ay_cal = ay_raw - (int16_t)acc_bias[1];
    int16_t az_cal = az_raw - (int16_t)acc_bias[2];
    int16_t gx_cal = gx_raw - (int16_t)gyro_bias[0];
    int16_t gy_cal = gy_raw - (int16_t)gyro_bias[1];
    int16_t gz_cal = gz_raw - (int16_t)gyro_bias[2];

    sensorData.ax = (float)ax_cal / BMI270_ACC_LSB_PER_G;
    sensorData.ay = (float)ay_cal / BMI270_ACC_LSB_PER_G;
    sensorData.az = (float)az_cal / BMI270_ACC_LSB_PER_G;
    sensorData.gx = (float)gx_cal / BMI270_GYR_LSB_PER_DPS;
    sensorData.gy = (float)gy_cal / BMI270_GYR_LSB_PER_DPS;
    sensorData.gz = (float)gz_cal / BMI270_GYR_LSB_PER_DPS;

    int16_t mx_raw=0, my_raw=0, mz_raw=0;
    bool magOk = lisReadMagRaw(mx_raw, my_raw, mz_raw);
    if (magOk) {
        sensorData.mx = ((float)mx_raw / LIS3MDL_LSB_PER_GAUSS - mag_bias[0]) * mag_scale[0];
        sensorData.my = ((float)my_raw / LIS3MDL_LSB_PER_GAUSS - mag_bias[1]) * mag_scale[1];
        sensorData.mz = ((float)mz_raw / LIS3MDL_LSB_PER_GAUSS - mag_bias[2]) * mag_scale[2];
    }

    if (magOk) {
        madgwickUpdate9DOF(
            sensorData.gx * PI/180.0f, sensorData.gy * PI/180.0f, sensorData.gz * PI/180.0f,
            sensorData.ax, sensorData.ay, sensorData.az,
            sensorData.mx, sensorData.my, sensorData.mz, dt
        );
    } else {
        madgwickUpdate9DOF(
            sensorData.gx * PI/180.0f, sensorData.gy * PI/180.0f, sensorData.gz * PI/180.0f,
            sensorData.ax, sensorData.ay, sensorData.az,
            0.0f, 0.0f, 0.0f, dt
        );
    }
    quaternionToEuler();
}

void IMU::calibrateAccelGyro() {
    const uint16_t sample_count = 200;
    int64_t acc_sum[3] = {0,0,0}, gyro_sum[3] = {0,0,0};
    for (uint16_t i = 0; i < sample_count; ++i) {
        uint8_t raw[12];
        bmiReadRegs(BMI270_REG_DATA_ACC_X_LSB, raw, 12);
        acc_sum[0]  += (int16_t)((raw[1]<<8)|raw[0]);
        acc_sum[1]  += (int16_t)((raw[3]<<8)|raw[2]);
        acc_sum[2]  += (int16_t)((raw[5]<<8)|raw[4]);
        gyro_sum[0] += (int16_t)((raw[7]<<8)|raw[6]);
        gyro_sum[1] += (int16_t)((raw[9]<<8)|raw[8]);
        gyro_sum[2] += (int16_t)((raw[11]<<8)|raw[10]);
        delay(5);
    }
    float acc_avg[3], gyro_avg[3];
    for (int i=0;i<3;++i) {
        acc_avg[i]=(float)acc_sum[i]/sample_count;
        gyro_avg[i]=(float)gyro_sum[i]/sample_count;
    }
    if (acc_avg[2] > 0) acc_avg[2] -= BMI270_ACC_LSB_PER_G; else acc_avg[2] += BMI270_ACC_LSB_PER_G;
    acc_bias[0]=acc_avg[0]; acc_bias[1]=acc_avg[1]; acc_bias[2]=acc_avg[2];
    gyro_bias[0]=gyro_avg[0]; gyro_bias[1]=gyro_avg[1]; gyro_bias[2]=gyro_avg[2];
}

void IMU::calibrateMag() {
    const uint16_t sample_count = 500;
    int16_t mag_max[3] = {INT16_MIN,INT16_MIN,INT16_MIN};
    int16_t mag_min[3] = {INT16_MAX,INT16_MAX,INT16_MAX};
    uint16_t collected = 0;
    for (uint16_t attempts=0; collected<sample_count && attempts<sample_count*3; ++attempts) {
        int16_t mx=0,my=0,mz=0;
        if (!lisReadMagRaw(mx,my,mz)) { delay(5); continue; }
        if (mx>mag_max[0]) mag_max[0]=mx; if (my>mag_max[1]) mag_max[1]=my; if (mz>mag_max[2]) mag_max[2]=mz;
        if (mx<mag_min[0]) mag_min[0]=mx; if (my<mag_min[1]) mag_min[1]=my; if (mz<mag_min[2]) mag_min[2]=mz;
        ++collected; delay(12);
    }
    mag_bias[0] = ((float)(mag_max[0]+mag_min[0])/2.0f) / LIS3MDL_LSB_PER_GAUSS;
    mag_bias[1] = ((float)(mag_max[1]+mag_min[1])/2.0f) / LIS3MDL_LSB_PER_GAUSS;
    mag_bias[2] = ((float)(mag_max[2]+mag_min[2])/2.0f) / LIS3MDL_LSB_PER_GAUSS;
    float sx = (float)(mag_max[0]-mag_min[0]) / 2.0f;
    float sy = (float)(mag_max[1]-mag_min[1]) / 2.0f;
    float sz = (float)(mag_max[2]-mag_min[2]) / 2.0f;
    float avg = (sx+sy+sz)/3.0f;
    mag_scale[0]=avg/sx; mag_scale[1]=avg/sy; mag_scale[2]=avg/sz;
}

const IMUSensorData& IMU::getData() const { return sensorData; }
float IMU::getRoll() const { return sensorData.roll; }
float IMU::getPitch() const { return sensorData.pitch; }
float IMU::getYaw() const { return sensorData.yaw; }

#endif 