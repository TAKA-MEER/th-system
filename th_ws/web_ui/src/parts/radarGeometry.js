// parts/radarGeometry.js — RadarSelect（対象選択レーダー）の座標変換の純関数。
// .jsx から分離して node --test でユニットテストできるようにする
// （parts/stickGeometry.js と同じ流儀）。
//
// base_link（+x 前方 / +y 左）を「前方＝上・左＝左」の見下ろし図に写す。
// 画面 x は右が正なので左（+y）は -y、画面 y は下が正なので前方（+x）は -x。
// 車のパーキングセンサ表示と同じ直感（brief-onsite-fix F-1）。
export const RADAR_PX_PER_M = 60
export const RADAR_CX = 120
export const RADAR_CY = 120

export function radarToSvg(x, y) {
  return [RADAR_CX - y * RADAR_PX_PER_M, RADAR_CY - x * RADAR_PX_PER_M]
}
