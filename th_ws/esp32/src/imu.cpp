#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

namespace Imu {

namespace {
Adafruit_BNO055 bno(-1, IMU_I2C_ADDR, &Wire);
}

bool init() {
    Wire.begin(IMU_SDA, IMU_SCL);
    if (!bno.begin()) {
        return false;
    }
    // 外部32.768kHz水晶(DSR1603に実装済み)を使い時刻/フュージョン精度を上げる
    bno.setExtCrystalUse(true);
    return true;
}

void read(float& qw, float& qx, float& qy, float& qz,
          float& wx, float& wy, float& wz,
          float& ax, float& ay, float& az,
          uint8_t& calibStatus) {
    imu::Quaternion q = bno.getQuat();
    qw = (float)q.w();
    qx = (float)q.x();
    qy = (float)q.y();
    qz = (float)q.z();

    // VECTOR_GYROSCOPE は rad/s (Adafruit_BNO055 ライブラリ仕様)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    wx = (float)gyro.x();
    wy = (float)gyro.y();
    wz = (float)gyro.z();

    // VECTOR_LINEARACCEL は重力成分除去済み m/s^2
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    ax = (float)accel.x();
    ay = (float)accel.y();
    az = (float)accel.z();

    uint8_t sys, gyroCal, accelCal, magCal;
    bno.getCalibration(&sys, &gyroCal, &accelCal, &magCal);
    calibStatus = (uint8_t)(((sys & 0x03) << 6) | ((gyroCal & 0x03) << 4) |
                             ((accelCal & 0x03) << 2) | (magCal & 0x03));
}

} // namespace Imu
