// i18n/faults.js — FaultStatus.fault_type -> Japanese.
// Source of truth: docs/plan/spec/Spec-webui.md §8 ("フォルトの表示 | 日本語。
// 原因と対処を1行で添える"). fault_type values come from
// th_system_msgs/FaultStatus.msg ("NONE" / "LIDAR_LOST" /
// "ESP32_DISCONNECTED" / "PERSON_TRACKER_LOST").
//
// This was left unbuilt by WP-UI-01 because /safety/fault wasn't in its
// interface contract yet (DetailedDesign-wp1.md WP-UI-01 §11 "W-1 の一般化").
export const FAULT_LABELS = {
  NONE: '',
  LIDAR_LOST: 'LiDARからの点群が途絶えています。再起動するか、LiDARの電源とケーブル接続を確認してください',
  ESP32_DISCONNECTED: 'ESP32との通信が途絶えています。ESP32の電源とWi-Fi接続を確認してください',
  PERSON_TRACKER_LOST: '対象の人物追跡が途絶えています。試験員がカメラの視界内にいるか確認してください',
}

export const FAULT_UNKNOWN_LABEL = '異常が発生しています（種別不明）'

export function faultLabel(faultType) {
  if (!faultType) return null
  return FAULT_LABELS[faultType] ?? FAULT_UNKNOWN_LABEL
}
