// stageMetrics() — 固定論理キャンバスの寸法と拡大率（shell/stageMetrics.js）。
//
// ここが守る性質は 3 つ:
//   1. 中身の縦横比を絶対に崩さない（縦横で別々の倍率をかけない）
//   2. はみ出さない（拡大後が必ずビューポートに収まる）
//   3. 壊れた入力で NaN/Infinity を CSS に流さない（画面が消える）
import test from 'node:test'
import assert from 'node:assert/strict'
import { stageMetrics, STAGE_LANDSCAPE, STAGE_PORTRAIT } from '../../src/shell/stageMetrics.js'

const close = (a, b, msg) => assert.ok(Math.abs(a - b) < 1e-9, `${msg}: ${a} != ${b}`)

test('横長のビューポートでは 16:9 の論理サイズを返す', () => {
  const m = stageMetrics(1920, 1080)
  assert.equal(m.orientation, 'landscape')
  assert.equal(m.width, STAGE_LANDSCAPE.width)
  assert.equal(m.height, STAGE_LANDSCAPE.height)
})

test('縦長のビューポートでは 9:16 の論理サイズを返す', () => {
  const m = stageMetrics(800, 1280)
  assert.equal(m.orientation, 'portrait')
  assert.equal(m.width, STAGE_PORTRAIT.width)
  assert.equal(m.height, STAGE_PORTRAIT.height)
})

test('ちょうど 16:9 なら拡大率 1（1280x720 = 論理サイズそのもの）', () => {
  close(stageMetrics(1280, 720).scale, 1, 'scale')
})

test('縦横比が一致していれば、大きい画面でも小さい画面でも比例した倍率になる', () => {
  close(stageMetrics(2560, 1440).scale, 2, '2x')
  close(stageMetrics(640, 360).scale, 0.5, '0.5x')
})

test('横に余った画面では高さが律速（縦横同率＝比率を崩さない）', () => {
  // 2000x720: 高さちょうど / 幅は余る -> 720/720 = 1 が上限
  close(stageMetrics(2000, 720).scale, 1, 'height-limited')
})

test('縦に余った画面では幅が律速', () => {
  // 1280x2000 は縦長判定なので基準は 720x1280。min(1280/720, 2000/1280)
  const m = stageMetrics(1280, 2000)
  assert.equal(m.orientation, 'portrait')
  close(m.scale, 2000 / 1280, 'width-limited')
})

test('拡大後が必ずビューポートに収まる（はみ出さない）', () => {
  const cases = [[1920, 1080], [1024, 768], [800, 1280], [360, 640], [1280, 720], [3000, 900]]
  for (const [w, h] of cases) {
    const m = stageMetrics(w, h)
    assert.ok(m.width * m.scale <= w + 1e-9, `width overflow at ${w}x${h}`)
    assert.ok(m.height * m.scale <= h + 1e-9, `height overflow at ${w}x${h}`)
  }
})

test('正方形は横長に倒す', () => {
  assert.equal(stageMetrics(1000, 1000).orientation, 'landscape')
})

test('0 / 負 / NaN では等倍に倒し、NaN や Infinity を返さない', () => {
  for (const [w, h] of [[0, 0], [0, 720], [1280, 0], [-1, -1], [NaN, 720], [1280, Infinity]]) {
    const m = stageMetrics(w, h)
    assert.ok(Number.isFinite(m.scale), `scale not finite at ${w}x${h}`)
    assert.equal(m.scale, 1)
    assert.ok(m.width > 0 && m.height > 0)
  }
})

test('論理サイズは差し替えられる（将来モックアップ寸法に合わせる余地）', () => {
  const m = stageMetrics(1120, 720, { landscape: { width: 1120, height: 720 } })
  assert.equal(m.width, 1120)
  close(m.scale, 1, 'custom base')
})
