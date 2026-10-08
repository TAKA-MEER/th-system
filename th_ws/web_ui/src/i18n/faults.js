// i18n/faults.js — FaultStatus.fault_type -> Japanese.
// Source of truth: docs/plan/spec/Spec-webui.md §8 ("フォルトの表示 | 日本語。
// 原因と対処を1行で添える"). fault_type values come from
// th_system_msgs/FaultStatus.msg と、safety_monitor が実際に出す種別すべて
// (test/unit/fault-labels.test.js が C++ のソースから機械的に突き合わせる)。
//
// This was left unbuilt by WP-UI-01 because /safety/fault wasn't in its
// interface contract yet (DetailedDesign-wp1.md WP-UI-01 §11 "W-1 の一般化").
export const FAULT_LABELS = {
  NONE: '',
  // 回復フォルト（Spec-safety.md §3.5）
  LIDAR_LOST: 'LiDARからの点群が途絶えています。LiDARの電源とケーブル接続を確認してください',
  ESP32_DISCONNECTED: 'ESP32との通信が途絶えています。ESP32の電源とラズパイとのシリアルケーブルを確認してください',
  PERSON_TRACKER_LOST: '対象の人物追跡が途絶えています。試験員がLiDARの見える範囲にいるか確認してください',
  UI_DISCONNECTED: 'タブレットとの接続が途切れています。通信状態を確認してください',
  // 重大フォルト（Spec-safety.md §3.5。非常停止に落ちる）
  LIMITER_DEAD: '障害物リミッタが応答していません。システムを再起動してください',
  MUX_DEAD: '速度指令の経路が途絶えています。システムを再起動してください',
  DRIVE_RUNAWAY: '駆動が指令と異なる動きをしました。車輪まわりを確認し、安全を確かめてください',
  STATE_INCONSISTENT: 'システムの状態が食い違っています。システムを再起動してください',
  LOCALIZATION_LOST: '自己位置を見失いました。メインメニューから位置を合わせ直してください',
  ESTOP_BYPASS_ACTIVE: '非常停止の回避設定が有効になっています。ESP32のファームウェア設定を確認してください',
}

export const FAULT_UNKNOWN_LABEL = '異常が発生しています（種別不明）'

export function faultLabel(faultType) {
  if (!faultType) return null
  return FAULT_LABELS[faultType] ?? FAULT_UNKNOWN_LABEL
}
