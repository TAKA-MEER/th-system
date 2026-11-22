// screens/routeMapWarning.js — 地図の画像（pgm/yaml）が無いことの表示判定（A-21）。
//
// 保存（S-13）: 経路は保存できたが画像を残せなかった → RouteStatus.map_save_failed。
// 再生（S-14）: 選んだ経路に画像が無い → RouteStatus.localize_quality === 'unknown'。
// どちらも Spec-transit.md §3.5／§4.2。S13TeachManual.jsx と S14Replay.jsx が使う。

// S-13: 「保存しました」に警告を添えるか。saved（JSON を書けた）が真で、
// かつ画像が揃わなかったときだけ。保存自体が失敗したら「保存されていません」を出す。
export function mapSaveWarningVisible(status) {
  return status?.saved === true && status?.map_save_failed === true
}

// S-14: 地図の画像を持たない経路か。このとき案内を出し、
// 「完全グローバルで探す」を押せなくする（押しても効かないため）。
export function noMapRoute(status) {
  return status?.localize_quality === 'unknown'
}
