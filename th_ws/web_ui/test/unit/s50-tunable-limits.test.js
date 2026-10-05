// SG-B9: registry 連動の上限（ros/s50TunableLimits.js）の unit テスト。
// 本番の上限値（generated/param_limits.json）が registry.yaml の導出値と
// 一致すること（生成忘れの検出。speed-presets-from-registry.test.js と同型）と、
// 丸め規則そのもの（上限超え→上限ちょうど、上限内→素通し、対象外→素通し）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { clampTunableParam, tunableFieldMax } from '../../src/ros/s50TunableLimits.js'

const registryYaml = readFileSync(
  fileURLToPath(new URL('../../../src/th_params/config/registry.yaml', import.meta.url)),
  'utf8',
)
const limits = JSON.parse(
  readFileSync(fileURLToPath(new URL('../../src/generated/param_limits.json', import.meta.url)), 'utf8'),
)

// registry.yaml の `- name: X` ブロック直下の `value:` を拾う（given 行用）。
function givenValue(name) {
  const m = registryYaml.match(new RegExp(`- name: ${name}\\n([\\s\\S]*?)(?=\\n- |\\n[A-Za-z_]+:\\s*$|\\z)`))
  assert.ok(m, `registry.yaml に ${name} が無い`)
  const v = /value:\s*([0-9.]+)/.exec(m[1])
  assert.ok(v, `${name} に数値 value が無い`)
  return Number(v[1])
}

test('param_limits.json の v_max は registry の 天井×余裕 と一致する', () => {
  const ceiling = givenValue('drivetrain_ceiling_mps')
  const ratio = givenValue('v_max_headroom_ratio')
  assert.ok(Math.abs(limits.v_max - ceiling * ratio) < 1e-9,
    `param_limits.v_max=${limits.v_max} が ${ceiling}×${ratio}=${ceiling * ratio} と食い違う（作り直し忘れ）`)
})

const CAPS = { follow_planner_mapless: { v_max: limits.v_max } }

test('上限超えは上限ちょうどに丸められる（保存できない）', () => {
  assert.equal(clampTunableParam(CAPS, 'follow_planner_mapless', 'v_max', 1.5), limits.v_max)
})

test('上限以内は素通しする', () => {
  assert.equal(clampTunableParam(CAPS, 'follow_planner_mapless', 'v_max', 0.3), 0.3)
})

test('対象外のノード・項目は素通しする', () => {
  assert.equal(clampTunableParam(CAPS, 'follow_planner_mapless', 'k_ang', 99), 99)
  assert.equal(clampTunableParam(CAPS, 'slam_toolbox', 'v_max', 99), 99)
})

test('数値でない値は素通しする（壊さない）', () => {
  assert.equal(clampTunableParam(CAPS, 'follow_planner_mapless', 'v_max', NaN), NaN)
  assert.equal(clampTunableParam(CAPS, 'follow_planner_mapless', 'v_max', undefined), undefined)
})

test('入力欄の max は registry 上限と欄既定の小さい方になる', () => {
  assert.equal(tunableFieldMax(CAPS, 'follow_planner_mapless', 'v_max', 1.5), limits.v_max)
  assert.equal(tunableFieldMax(CAPS, 'follow_planner_mapless', 'k_ang', 5), 5)
})
