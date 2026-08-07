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

// モード名 → バッジ色。操作 UI (App.jsx) と観客表示 (audience/) で共有する。
// 片方だけ色が変わると同じデモを見ている人の間で認識がズレるため 1 箇所に置く
const MODE_COLORS = {
  INIT: '#888', IDLE: '#2196F3', FOLLOWING: '#4CAF50',
  MOVING_TO_PANEL: '#FF9800', AT_PANEL: '#9C27B0',
  MANUAL: '#00BCD4', ESTOP: '#F44336', FOLLOWING_MAPLESS: '#8BC34A',
  SUMMONING: '#E91E63',
}

export const modeColorOf = (modeName) => MODE_COLORS[modeName] ?? '#888'
