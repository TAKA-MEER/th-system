// parts/mapColors.js — 地図の配色の定数（Spec-webui.md §6.1・2026-11-27 D78〜D80）。
//
// 地図は色相を使わず濃淡だけで描く。色（緑・黄）は機体の状態と操作の意味に
// だけ使う（Spec-webui.md §3.3.1、JIS B 9960-1）。全ての地図（試験場内地図・
// 教示経路プレビュー・教示再生）はこの 1 ファイルの色を使う。
//   - 未探索: 画面の地と同じ色（theme.css の --bg）にして見えなくする
//   - 床（空き）: 白／壁（占有）: 中間の灰
//   - 点群: 壁より濃い灰。減速が始まる距離より近い点は注意の黄
//   - コストマップの「入れない所」: 薄い灰（内接・致死のセルだけ。costmapPixels.js）
//   - 進むの色（緑）: 機体・経路・行き先の輪
//   - 注意の黄: 経路が塞がれた所（BLOCKED の円）・近い点
//   - 中立の色（黒・白）: ピン・追跡中の人・タップの印。選んでいるものは反転
//
// 緑・黄はモックアップ（docs/plan/spec/mockup/index.html の col-jis）の
// --route/--robot/--dest（#5cc477）と --jis-y（#ffd100）に合わせた。

export const MAP_UNKNOWN = '#0f1117' // 画面の地（theme.css --bg）と同じ。未探索。
export const MAP_FREE = '#ffffff' // 床（空き）＝白
export const MAP_OCCUPIED = '#808080' // 壁（占有）＝中間の灰
export const MAP_SCAN = '#4a4a4a' // 点群＝壁より濃い灰
export const MAP_NEAR = '#ffd100' // 近い点＝注意の黄（JIS yellow）
export const MAP_COST = '#9e9e9e' // 「入れない所」＝薄い灰
export const MAP_GO = '#4caf50' // 進むの色＝緑（機体・経路・行き先の輪）
export const MAP_WARN = '#ffd100' // 注意の黄（塞がれた所の円）
export const MAP_PIN = '#1a1a1a' // ピン・人の中立の塗り（黒）
export const MAP_PIN_EDGE = '#ffffff' // ピン・人の縁（白）

  // 機体の大きさ（Spec-webui.md §6.1: 実寸 0.55 m 角・内接半径 0.275 m）
export const ROBOT_SIZE_M = 0.55
export const ROBOT_HALF_M = 0.275

// 「減速が始まる距離」の仮値。機体側（obstacle_limiter の判定距離）は
// WebUI に届いていない（LimiterStatus にあるのはいまの距離であって閾値が
// ない）ため、ブリーフ W0b の指示どおり約 0.6 m を仮に使う。
// 実機で決まったらここだけ直す。
export const SLOWDOWN_DISTANCE_M = 0.6

// 機体を中心に SLOWDOWN_DISTANCE_M より近い点は注意の黄、それ以外は濃い灰。
// 距離は LiDAR 原点（機体中心）からの直線距離で読む（scanToPoints の出力は
// 機体中心相対、OnsiteMap の scanPoints は map フレーム済み）。
export function scanPointFill(distM) {
  return Number.isFinite(distM) && distM <= SLOWDOWN_DISTANCE_M ? MAP_NEAR : MAP_SCAN
}

// 点群の点が減速距離より近いか（機体から見た距離で判定）。
export function isNearPoint(distM) {
  return Number.isFinite(distM) && distM <= SLOWDOWN_DISTANCE_M
}

// 占有格子の値 → RGBA の色（occupancyGridToPixels と共有する判定）。
// 未知（-1、cbor 化で 255）→ 透明（画面の地＝未探索）／
// 0（自由）→ 白／100（占有）→ 中間の灰。50 付近の中間値は白と中間の灰の間。
// alpha を返すため、描画側で RGBA に組む。
export function occupancyGray(v) {
  if (v < 0 || v > 100) return null // 未知＝透明
  if (v <= 0) return 255
  if (v >= 100) return 128
  return 255 - Math.round((v / 100) * (255 - 128))
}
