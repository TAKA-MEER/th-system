// screens/screenZone.js — 場内（ゾーン IN）の画面の判定（純関数）。
// 正本は th_state/zones.py の SCREEN_ZONE（DetailedDesign-names.md §4）。ここは
// ブラウザ側で「高速プリセットを出さない」判定にだけ使う写し。ブラウザに速度上限は
// 持たせない（上限の権威は obstacle_limiter）ので、持つのは画面 ID の集合だけ。
// zones.py と食い違わないことは test/unit/screen-zone.test.js が縛る。
// Spec-webui.md「速度プリセット」: 場内の画面（S-20・S-21）では「高速」を出さない。
export const IN_ZONE_SCREEN_IDS = ['S-20', 'S-21']

export function isInZoneScreen(screenId) {
  return IN_ZONE_SCREEN_IDS.includes(screenId)
}

// 場内の画面では速度プリセットの割合を中速までに丸める。高速のまま場内の画面に入っても
// 中速に落ちる（選択の状態そのものを書き換えず、表示・送出の側で丸める）。
export function clampPresetForZone(ratio, inZone, midRatio) {
  return inZone ? Math.min(ratio, midRatio) : ratio
}
