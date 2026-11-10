// ros/activeFaultsState.js — /safety/active_faults (th_system_msgs/ActiveFaults)
// の読み取りと購読（ROS も React も import しない。node --test で直接試験する）。
//
// /safety/fault は「最新の 1 件」しか持たないため、LIDAR_LOST と
// ESP32_DISCONNECTED が同時に立つと後者が見えない（2026-11-10 実機: ラズパイの
// 電源断）。W-2 の ESP32 切断表示は、こちらの集合を正本にする。
//
// 機体の発行側は RELIABLE＋TRANSIENT_LOCAL・depth 1（safety_monitor）。後から
// 開いた画面にも最新の集合が 1 通届くので、queue_length は 1 のままにする。
export function subscribeActiveFaults(ROSLIB, ros, topicName, messageType, onMessage) {
  const topic = new ROSLIB.Topic({
    ros, name: topicName, messageType, queue_length: 1,
  })
  topic.subscribe(onMessage)
  return topic
}

// メッセージ -> フォルト種別の配列。読めなければ null（購読側は前の集合を保つ）。
export function parseActiveFaults(msg) {
  if (!msg || !Array.isArray(msg.types)) return null
  return msg.types.filter((t) => typeof t === 'string')
}
