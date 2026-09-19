// DetailedDesign-wp3.md WP-TRANSIT-01 §7: obstacleWarning() の 3 分岐と未受信。
// action 文字列（"PASS"/"CLAMP"/"STOP"）でレベルを分け、nearest_obstacle_m を
// そのまま返す。未受信（null/undefined）は「不明」= null を返す（安全側）——
// "障害物なし" と表示してはいけない（DetailedDesign-wp3.md §6.2）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { obstacleWarning } from '../../src/shell/limits.js'

test('STOP -> { level: "stop" } and passes nearest_obstacle_m through', () => {
  const w = obstacleWarning({ action: 'STOP', nearest_obstacle_m: 0.4, applied_limit_mps: 0.0 })
  assert.deepEqual(w, { level: 'stop', distance_m: 0.4 })
})

test('CLAMP -> { level: "warn" }', () => {
  const w = obstacleWarning({ action: 'CLAMP', nearest_obstacle_m: 0.6 })
  assert.deepEqual(w, { level: 'warn', distance_m: 0.6 })
})

test('PASS -> { level: "none" }', () => {
  const w = obstacleWarning({ action: 'PASS', nearest_obstacle_m: 5.0 })
  assert.deepEqual(w, { level: 'none', distance_m: 5.0 })
})

test('undefined / null (未受信) -> null, not { level: "none" }', () => {
  assert.equal(obstacleWarning(undefined), null)
  assert.equal(obstacleWarning(null), null)
})

// 未知の action 値は none 扱いにしない——安全側に倒れさせる（詳細は
// DetailedDesign-webui.md §4.1 の「UI は権威ではない」）。ただし未受信とは別。
test('unknown action value treated as none (limiter actively reported but odd)', () => {
  const w = obstacleWarning({ action: 'SOMETHING_ELSE', nearest_obstacle_m: 1.0 })
  assert.deepEqual(w, { level: 'none', distance_m: 1.0 })
})
