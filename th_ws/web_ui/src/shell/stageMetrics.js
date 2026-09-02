// shell/stageMetrics.js — 固定論理キャンバス（ステージ）の寸法と拡大率。
//
// 2026-09-02: WebUI が端末の画面サイズに影響されやすい問題への対策。
// それまでは #app が width:100% / height:100dvh の流動レイアウトで、
// 画面の実寸と縦横比がそのまま中身の折り返し・行数・図の大きさに効いていた。
// タブレットを替えたり画面を回したりするたびに見え方が変わり、詰まる。
//
// 代わりに「決め打ちの論理サイズで描いて、画面いっぱいまで等倍拡大する」
// 方式にする（モックアップ docs/plan/spec/mockup/index.html が #stage と
// 固定サイズ #app でやっているのと同じ考え方）。中身は常に同じピクセル数の
// 箱に描かれるので、レイアウトは端末に依存しなくなる。余りは黒帯
// （レターボックス）になる。
//
// ROS も React も要らない純関数なので、ユニットテストで直接検証する
// （test/unit/stage-metrics.test.js）。
"use strict"

// 横長 16:9 / 縦長 9:16。縦横比はユーザー指定（2026-09-02）。
export const STAGE_LANDSCAPE = { width: 1280, height: 720 }
export const STAGE_PORTRAIT = { width: 720, height: 1280 }

/**
 * ビューポートの実寸から、論理キャンバスの寸法と拡大率を決める。
 *
 * 拡大率は「はみ出さずに最大」= min(横の余裕, 縦の余裕)。縦横同率なので
 * 中身の縦横比は絶対に崩れない（片方だけ伸ばすと図と当たり判定がずれる）。
 *
 * @param {number} viewportW ビューポート幅 (px)
 * @param {number} viewportH ビューポート高さ (px)
 * @returns {{width:number, height:number, scale:number, orientation:'landscape'|'portrait'}}
 */
export function stageMetrics(viewportW, viewportH, opts = {}) {
  const landscape = opts.landscape ?? STAGE_LANDSCAPE
  const portrait = opts.portrait ?? STAGE_PORTRAIT

  // 測定前 / 非表示タブ / jsdom では 0 や NaN が来る。ここで 0 除算して
  // scale=Infinity や NaN を CSS に流すと画面が消えるので、素の等倍に倒す。
  const ok = Number.isFinite(viewportW) && Number.isFinite(viewportH)
    && viewportW > 0 && viewportH > 0
  if (!ok) {
    return { ...landscape, scale: 1, orientation: 'landscape' }
  }

  // 正方形はどちらでもよいので横長に倒す（タブレットの既定の持ち方）。
  const orientation = viewportW >= viewportH ? 'landscape' : 'portrait'
  const base = orientation === 'landscape' ? landscape : portrait
  const scale = Math.min(viewportW / base.width, viewportH / base.height)

  return { width: base.width, height: base.height, scale, orientation }
}
