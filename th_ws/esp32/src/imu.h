#pragma once
#include <Arduino.h>

// ============================================================
// imu.h — DSR1603 (Bosch BNO055) I2C 9軸センサドライバ
//
// BNO055 のオンチップフュージョンが出力するクォータニオン・角速度・
// 線形加速度(重力除去済み)・キャリブレーション状態を取得する。
// ============================================================

namespace Imu {

// I2C 初期化 + BNO055 検出。センサ未実装の個体でも起動を止めないよう
// 戻り値で成否を返す(false の場合 read() を呼んではいけない)。
bool init();

// calibStatus: sys/gyro/accel/mag を上位から2bitずつパック
// (bit7-6=sys, bit5-4=gyro, bit3-2=accel, bit1-0=mag、各0-3)
void read(float& qw, float& qx, float& qy, float& qz,
          float& wx, float& wy, float& wz,
          float& ax, float& ay, float& az,
          uint8_t& calibStatus);

} // namespace Imu
