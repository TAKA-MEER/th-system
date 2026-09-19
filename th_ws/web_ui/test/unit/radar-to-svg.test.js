// brief-onsite-fix F-1: radarToSvg() のユニットテスト。
// base_link（+x 前方 / +y 左）を「前方＝上・左＝左」の見下ろし図に写す
// （車のパーキングセンサ表示と同じ直感）。変更前は「前方として右」に描かれる
// 90°ずれの写像だった。
import test from 'node:test'
import assert from 'node:assert/strict'
import { radarToSvg, RADAR_PX_PER_M, RADAR_CX, RADAR_CY } from '../../src/parts/radarGeometry.js'

const CX = RADAR_CX
const CY = RADAR_CY

test('radarToSvg: 機体真正面（x=1, y=0）は中心より上', () => {
  const [px, py] = radarToSvg(1, 0)
  assert.equal(px, CX)                       // 左右は不変
  assert.ok(py < CY, '前方（+x）は画面上側（y が小さい）')      // 上
  assert.equal(py, CY - 1 * RADAR_PX_PER_M)
})

test('radarToSvg: 機体左（x=0, y=1）は中心より左', () => {
  const [px, py] = radarToSvg(0, 1)
  assert.ok(px < CX, '左（+y）は画面左側（x が小さい）')
  assert.equal(px, CX - 1 * RADAR_PX_PER_M)
  assert.equal(py, CY)
})

test('radarToSvg: 機体右（x=0, y=-1）は中心より右', () => {
  const [px, py] = radarToSvg(0, -1)
  assert.equal(px, CX + 1 * RADAR_PX_PER_M)
  assert.equal(py, CY)
})

test('radarToSvg: 機体中心（0,0）はビュー中心', () => {
  assert.deepEqual(radarToSvg(0, 0), [CX, CY])
})
