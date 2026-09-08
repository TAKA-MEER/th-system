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

// ROS の yaw（rad、反時計回り正）を SVG の rotate 角度（度、画面上は時計回り正）に変換する。
// SVG は y 軸が下向きなので、toSvg の y 反転（CY - y）と合わせるには符号を反転する
// （brief-onsite-fix F-3: 符号を反転しないと yaw=+90° で矢印が画面の下を指す）。
export function yawToSvgDeg(yawRad) {
  return -((yawRad * 180) / Math.PI) + 0
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

// ── S-20 / S-21 共用の試験場内地図ビュー ─────────────────────────────
// 実地図（/route/map_view、map フレーム）を 1 枚の SVG に載せるための純関数
// （brief-onsite-fix C）。地図がありのときは OccupancyGrid の info から
// computeMapView() で等方スケールの view を作り、ピン・ロボットも worldToCanvas()
// で同じ変換を通す（ズレ防止）。地図が未受信（null）のときは従来 S-20/S-21 の
// 空 SVG（中央寄せ 1m=24px のデモ見た目）と同じ変換にフォールバックする。
//
// 戻り値:
//   { mapData, view, viewW, viewH, toPx(x, y) }
//     - mapData: 元の OccupancyGrid（ラスタ描画に使う）。null でもよい
//     - view: computeMapView() の戻り値（地図あり時）。null なら地図なしフォールバック
//     - toPx(x, y): map 座標 [m] → SVG ピクセル [px, py]。地図あり/なしで同じ形
export function onsiteMapTransform(mapData, viewW, viewH, fallbackPxPerM = 24, insetPx = 0) {
  // insetPx: 地図を SVG の内側に寄せる余白 [px]。ピンのラベルは丸の下 26px に
  // 描かれるので、余白ゼロだと縁のピンのラベルが SVG の外で切れる（UX-7）。
  // 既定 0 なので既存の呼び出し・ユニットテストの期待値は変わらない。
  const info = mapData?.info
  const data = mapData?.data
  const infoOk = info
    && Number.isFinite(info.resolution) && info.resolution > 0
    && Number.isInteger(info.width) && info.width > 0
    && Number.isInteger(info.height) && info.height > 0
    && info.origin?.position
  const dataOk = (Array.isArray(data) || ArrayBuffer.isView(data))
    && data.length >= info?.width * info?.height
  if (!infoOk || !dataOk) {
    const scale = fallbackPxPerM
    return {
      mapData: null,
      view: null,
      viewW,
      viewH,
      toPx(x, y) { return [viewW / 2 + x * scale, viewH / 2 - y * scale] },
    }
  }
  const view = computeMapView(
    info, viewW - insetPx * 2, viewH - insetPx * 2, { panX: insetPx, panY: insetPx })
  return {
    mapData,
    view,
    viewW,
    viewH,
    toPx(x, y) { return worldToCanvas(x, y, info, view) },
  }
}
