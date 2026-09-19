// parts/scanToPoints.js — sensor_msgs/LaserScan を base_link 相対の点群に変換する
// 純関数（brief-MAP-SCAN）。ROS/React に依存しないので node:test で直接テストできる。
//
// ranges[i] は angle_min + i * angle_increment 方向の距離。range_min <= r <= range_max
// かつ有限のものだけを点として採用する（Infinity/NaN は「検出なし」なので除外）。
// 極座標→直交座標（x=前方=yaw0、y=左）。laser_link は base_link と x/y/yaw が同一
// （Z のみ車輪半径分違う、mapGeometry.js の baseToWorld() コメント参照）なので、
// この点群はそのまま base_link 相対として使える。
//
// 間引きはしない（全点返す）。間引きは呼び出し側（描画側）の裁量。
export function scanToPoints(scan) {
  if (!scan || !Array.isArray(scan.ranges)) return []
  const { angle_min = 0, angle_increment = 0, range_min = 0, range_max = Infinity } = scan
  const out = []
  for (let i = 0; i < scan.ranges.length; i += 1) {
    const r = scan.ranges[i]
    if (typeof r !== 'number' || !Number.isFinite(r)) continue
    if (r < range_min || r > range_max) continue
    const angle = angle_min + i * angle_increment
    out.push({ x: r * Math.cos(angle), y: r * Math.sin(angle) })
  }
  return out
}