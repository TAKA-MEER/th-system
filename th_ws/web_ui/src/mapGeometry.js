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

// 地図グリッドを canvas にどう載せるかを決める (等方スケール + 中央寄せ)。
//
// 2026-08-07 まで X と Y で別々の倍率 (canvasW/width, canvasH/height) を掛けて
// グリッドを canvas いっぱいに引き伸ばしていた。canvas と地図グリッドの縦横比が
// 違うと形が崩れ、さらに地図原点に回転が入っている場合は「回転してから非等方
// スケール」になるため直角が直角でなくなり平行四辺形に歪む。観客向け表示
// (横長ペイン全面) で顕著だった。倍率は必ず 1 つに保つこと。
//
// zoom/panX/panY は操作 UI のパン・ズーム用。観客向けは既定値のまま使う。
export function computeMapView(mapInfo, canvasW, canvasH, opts = {}) {
  const { zoom = 1, panX = 0, panY = 0 } = opts
  const fit = Math.min(canvasW / mapInfo.width, canvasH / mapInfo.height)
  const scale = fit * zoom            // canvas px / グリッドセル
  const drawW = mapInfo.width * scale
  const drawH = mapInfo.height * scale
  return {
    scale,
    drawW,
    drawH,
    offX: (canvasW - drawW) / 2 + panX,
    offY: (canvasH - drawH) / 2 + panY,
    mPerPx: mapInfo.resolution / scale,
  }
}

// map座標系の (x, y) [m] を canvas ピクセル座標 [px, py] に変換する。
// OccupancyGrid の行0はmapフレーム下端・originはセル(0,0)の座標という前提。
// view は computeMapView() の戻り値。
export function worldToCanvas(x, y, mapInfo, view) {
  const { resolution, origin } = mapInfo
  const originYaw = quatToYaw(origin.orientation)
  const dx = x - origin.position.x
  const dy = y - origin.position.y
  const cos = Math.cos(-originYaw), sin = Math.sin(-originYaw)
  const gx = (dx * cos - dy * sin) / resolution   // グリッドセル座標 (列)
  const gy = (dx * sin + dy * cos) / resolution   // グリッドセル座標 (行, 下端基準)
  return [
    view.offX + gx * view.scale,
    view.offY + view.drawH - gy * view.scale,     // 地図と同じ上下反転
  ]
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
