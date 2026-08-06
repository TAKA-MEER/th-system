// ============================================================
// mapGeometry.js — 地図座標まわりの共有ジオメトリ
//
// MapView (操作 UI の地図カード) と audience/WorldCanvas (観客向け表示) の
// 両方が同じ変換を使う。片方だけ直して座標がズレる事故を防ぐため、
// 変換式はこのファイル 1 箇所にまとめる。
// ============================================================

export function quatToYaw(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

// map座標系の (x, y) [m] を canvas ピクセル座標 [px, py] に変換する。
// OccupancyGrid の行0はmapフレーム下端・originはセル(0,0)の座標という前提
export function worldToCanvas(x, y, mapInfo, canvasW, canvasH) {
  const { resolution, width, height, origin } = mapInfo
  const originYaw = quatToYaw(origin.orientation)
  const dx = x - origin.position.x
  const dy = y - origin.position.y
  const cos = Math.cos(-originYaw), sin = Math.sin(-originYaw)
  const gx = (dx * cos - dy * sin) / resolution   // グリッドセル座標 (列)
  const gy = (dx * sin + dy * cos) / resolution   // グリッドセル座標 (行, 下端基準)
  const scaleX = canvasW / width
  const scaleY = canvasH / height
  return [gx * scaleX, canvasH - gy * scaleY]     // 地図と同じ上下反転
}

// base_link 相対の点 (x=前方, y=左) を map 座標へ。
// laser_link は base_link と x/y/yaw が同一 (Z のみ車輪半径分違う、
// th_robot.urdf.xacro の laser_joint) なので、点群・脚検出候補・追跡対象の
// いずれも robotPose をそのまま使ってよい。
export function baseToWorld(localX, localY, robotPose) {
  const cos = Math.cos(robotPose.yaw), sin = Math.sin(robotPose.yaw)
  return [
    robotPose.x + localX * cos - localY * sin,
    robotPose.y + localX * sin + localY * cos,
  ]
}
