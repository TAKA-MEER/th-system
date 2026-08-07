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

    // VECTOR_GYROSCOPE は **dps (度/秒)** で返る。rad/s へ変換すること。
    //
    // Adafruit_BNO055::begin() 内で UNIT_SEL レジスタを書く処理はライブラリ側で
    // コメントアウトされており(Adafruit_BNO055.cpp の "Set the output units" 付近)、
    // BNO055 は電源投入時の既定単位のまま動く。既定は GYR_Unit=0=dps。
    // getVector() の除数もそれに合わせた dps 用スケーリング(/16.0、コメントに
    // "1dps = 16 LSB")になっている。Euler が /16=度、加速度が /100=m/s^2 と
    // 3つとも既定単位で一貫していることが裏付け。
    //
    // 以前はここに「rad/s (ライブラリ仕様)」という誤ったコメントがあり、値を
    // そのまま rad/s として扱っていた(57.3倍)。影響は2箇所(2026-08-06 修正):
    //   1. main.cpp の直進ドリフト補正 — ループゲインが設計値 0.385 に対し約22に
    //      なり、実ヨーレート 0.012 rad/s 相当で既に DRIFT_CORRECTION_MAX_MPS へ
    //      飽和する。事実上常時飽和し wz の符号反転ごとに±0.1m/s を反転する
    //      bang-bang 振動になっていた(直進時のみ左右に振動する事象の原因)。
    //   2. /esp32/imu_data の angular_velocity — sensor_msgs/Imu は rad/s 規定
    //      なので、EKF が 57.3 倍のヨーレートを信じてオドメトリが壊れる。
    // M_PI はヘッダ依存で入らないことがあるため定数を直書きする (pi/180)
    static const float DPS_TO_RAD_S = 0.017453293f;
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    wx = (float)gyro.x() * DPS_TO_RAD_S;
    wy = (float)gyro.y() * DPS_TO_RAD_S;
    wz = (float)gyro.z() * DPS_TO_RAD_S;

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
