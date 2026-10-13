// ros/mapSessionState.js — /map_session/status (th_system_msgs/MapSessionStatus) の
// 読み取りと「会場地図を開いたか」の判定（Spec-webui.md §3.11 S-21。2026-10-13）。
// ROS も React も import しない（node --test で直接試験する）。
//
// 機体（slam_control）は mode を 3 値で出す:
//   MAPPING = 地図作成中 / LOCALIZING = 会場地図を読み込めていて作成していない /
//   UNLOADED = それ以外（起動直後・破棄後・再起動後など）。
// 「地図作成が止まっているだけ」は LOCALIZING にならない。

// 発行側は RELIABLE＋TRANSIENT_LOCAL・depth 1。後から開いた画面（手押し復帰で作り直された
// S-21、再読み込み）にも最新の 1 通が届くのが肝。queue_length は 1 のまま・throttle なし。
export function subscribeMapSession(ROSLIB, ros, topicName, messageType, onMessage) {
  const topic = new ROSLIB.Topic({
    ros, name: topicName, messageType, queue_length: 1,
  })
  topic.subscribe(onMessage)
  return topic
}

// メッセージ -> { slot, mode, dirty, instance_id }。読めなければ null。
export function parseMapSession(msg) {
  if (!msg || typeof msg.slot !== 'string' || typeof msg.mode !== 'string') return null
  return {
    slot: msg.slot,
    mode: msg.mode,
    dirty: !!msg.dirty,
    instance_id: msg.instance_id ?? '',
  }
}

// 会場地図を開いている（読み込み済み）か。status が null（まだ届いていない）なら null。
// 機体の値だけで決める（画面ローカルの成功応答には頼らない）。
export function venueMapOpenedFromStatus(status) {
  if (!status) return null
  return status.slot === 'VENUE' && status.mode === 'LOCALIZING'
}

// S-21 の mapOpened。機体の値が届いていればそれを優先する（UNLOADED／MAPPING に
// 戻ったら、ローカルの成功応答が残っていても偽）。届く前（null）だけ従来どおり
// ローカルの成功応答（venueOk）で決める。
export function resolveMapOpened(status, venueOk) {
  const fromRobot = venueMapOpenedFromStatus(status)
  if (fromRobot !== null) return fromRobot
  return venueOk === true
}
