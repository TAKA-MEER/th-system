// S-11 の計器・ガイド線の純関数（parts/surroundGeom.js）。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  speedOfSample, latestSpeed, angularFromWheels, history5,
  limitOf, brakeCutOf, gapOf, guideFor, GUIDE_LENGTH_M,
} from '../../src/parts/surroundGeom.js'

test('speedOfSample は左右の平均、欠けたサンプルは null', () => {
  assert.equal(speedOfSample({ left: 0.4, right: 0.6 }), 0.5)
  assert.equal(speedOfSample({ left: 0.4 }), null)
  assert.equal(speedOfSample(null), null)
})

test('latestSpeed は最新サンプル、空は null', () => {
  const s = [{ t: 1, left: 0, right: 0 }, { t: 2, left: 0.2, right: 0.4 }]
  assert.ok(Math.abs(latestSpeed(s) - 0.3) < 1e-9)
  assert.equal(latestSpeed([]), null)
  assert.equal(latestSpeed(null), null)
})

test('angularFromWheels は車輪間距離が無いと null（推測しない）', () => {
  assert.equal(angularFromWheels({ left: 0.1, right: 0.5 }, 0.4), 1)
  assert.equal(angularFromWheels({ left: 0.1, right: 0.5 }, null), null)
  assert.equal(angularFromWheels({ left: 0.1, right: 0.5 }, 0), null)
})

test('history5 は直近 5 秒だけ残す', () => {
  const s = [
    { t: 0, left: 1, right: 1 },
    { t: 6, left: 0.2, right: 0.2 },
    { t: 9, left: 0.4, right: 0.4 },
    { t: 10, left: 0.6, right: 0.6 },
  ]
  assert.deepEqual(history5(s, 10), [{ t: 6, v: 0.2 }, { t: 9, v: 0.4 }, { t: 10, v: 0.6 }])
  assert.deepEqual(history5([], 10), [])
  assert.deepEqual(history5(null, 10), [])
})

test('limitOf は applied_limit_mps、無ければ null', () => {
  assert.equal(limitOf({ applied_limit_mps: 0.55 }), 0.55)
  assert.equal(limitOf({ applied_limit_mps: 0 }), null)
  assert.equal(limitOf({}), null)
  assert.equal(limitOf(null), null)
})

test('brakeCutOf は削っているときだけ in-out、PASS は 0', () => {
  assert.equal(brakeCutOf({ in_linear: 0.5, out_linear: 0.2, action: 'CLAMP' }), 0.3)
  assert.equal(brakeCutOf({ in_linear: 0.5, out_linear: 0.5, action: 'PASS' }), 0)
  assert.equal(brakeCutOf({ in_linear: 0.2, out_linear: 0.5, action: 'CLAMP' }), 0)
  assert.equal(brakeCutOf(null), 0)
})

test('gapOf は目標と実際の乖離', () => {
  assert.equal(gapOf(0.2, 0.5), 0.3)
  assert.equal(gapOf(null, 0.5), null)
})

test('guideFor: 止まっている間は出さない', () => {
  assert.equal(guideFor(0, 0), null)
  assert.equal(guideFor(0.001, 0.001), null)
  assert.equal(guideFor(null, null), null)
})

test('guideFor: 前進は前に 1.5 m の線', () => {
  const g = guideFor(0.5, 0)
  assert.equal(g.kind, 'path')
  const last = g.points[g.points.length - 1]
  assert.ok(Math.abs(last.x - (0.275 + GUIDE_LENGTH_M)) < 0.01, `x=${last.x}`)
  assert.ok(Math.abs(last.y) < 1e-9)
  // 前進は x が単調に増える
  for (let i = 1; i < g.points.length; i += 1) assert.ok(g.points[i].x >= g.points[i - 1].x)
})

test('guideFor: 後退は後ろへ出る', () => {
  const g = guideFor(-0.5, 0)
  assert.equal(g.kind, 'path')
  const last = g.points[g.points.length - 1]
  assert.ok(last.x < -0.275, `x=${last.x}`)
  assert.ok(Math.abs(last.y) < 1e-9)
})

test('guideFor: 回転しながら進むと曲がる（左回りは左へ）', () => {
  const g = guideFor(0.5, 0.5)
  assert.equal(g.kind, 'path')
  const last = g.points[g.points.length - 1]
  assert.ok(last.y > 0.1, `y=${last.y}`)
})

test('guideFor: その場で回すときは弧', () => {
  const g = guideFor(0, 0.5)
  assert.equal(g.kind, 'arc')
  assert.ok(g.r > 0.275)
})
