// ============================================================
// robotMode.js — RobotMode.msg の定数
//
// th_system_msgs/msg/RobotMode.msg と一対一で対応する。
// msg 側に値を足したらここも直すこと。
// ============================================================

export const MODE = {
  INIT:              0,
  IDLE:              1,
  FOLLOWING:         2,
  MOVING_TO_PANEL:   3,
  AT_PANEL:          4,
  MANUAL:            5,
  ESTOP:             6,
  FOLLOWING_MAPLESS: 7,
  SUMMONING:         8,
}
