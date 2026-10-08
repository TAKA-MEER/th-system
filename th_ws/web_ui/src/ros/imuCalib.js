// ros/imuCalib.js — /esp32/imu_calib_status（std_msgs/UInt8）の分解（純関数）。
// bit 配分は機体側 check_core.unpack_calib と同じ（BNO055 CALIB_STAT 準拠）:
//   [1:0]=mag / [3:2]=accel / [5:4]=gyro / [7:6]=sys。各 0〜3（3 が校正済み）。
export function unpackImuCalib(v) {
  if (typeof v !== 'number' || !Number.isFinite(v)) return null
  const n = v & 0xff
  return {
    sys: (n >> 6) & 3,
    gyro: (n >> 4) & 3,
    accel: (n >> 2) & 3,
    mag: n & 3,
  }
}
