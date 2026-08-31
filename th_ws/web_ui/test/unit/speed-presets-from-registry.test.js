// DetailedDesign-wp3.md WP-UI-03 §7 (R2): the browser must not hard-code any
// speed ceiling, so the 低速/中速/高速 preset ratios come from generated
// speed_presets.json, which scripts/gen_speed_presets.py derives from
// th_params/config/registry.yaml. This test pins the generated JSON to the
// authoritative registry so a hand-edited JSON (a drift back toward literals
// in the browser) is caught. It only reads both files; it does not run the
// generator, because colcon/python is not guaranteed on the Node test path
// (and node --test runs on the host).
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'

const yaml = readFileSync(
  fileURLToPath(new URL('../../../src/th_params/config/registry.yaml', import.meta.url)),
  'utf8',
)
const presets = JSON.parse(
  readFileSync(fileURLToPath(new URL('../../src/generated/speed_presets.json', import.meta.url)), 'utf8'),
)

// Pull each `- name: speed_preset_*` block and its following `value:`.
function fromRegistry() {
  const out = {}
  const blockRe = /- name: (speed_preset_\w+)\n([\s\S]*?)(?=\n- |\n[A-Za-z_]+:\s*$|\z)/g
  let m
  while ((m = blockRe.exec(yaml)) !== null) {
    const valueMatch = /value:\s*([0-9.]+)/.exec(m[2])
    if (valueMatch) out[m[1]] = Number(valueMatch[1])
  }
  return out
}

test('generated speed_presets.json matches registry.yaml exactly', () => {
  const registry = fromRegistry()
  const keys = Object.keys(registry).sort()
  assert.deepEqual(
    keys,
    ['speed_preset_high', 'speed_preset_low', 'speed_preset_mid'],
    'registry must define exactly the three preset keys (no more, no less)',
  )
  for (const key of keys) {
    assert.ok(
      key in presets,
      `speed_presets.json is missing ${key} from registry.yaml (regression toward a hard-coded preset)`,
    )
    assert.equal(presets[key], registry[key], `${key} drifted from registry.yaml (R2)`)
  }
  // And nothing beyond the three (a stray key would be unused dead data).
  assert.deepEqual(Object.keys(presets).sort(), keys)
})

test('the three presets take distinct increasing values', () => {
  const vals = [presets.speed_preset_low, presets.speed_preset_mid, presets.speed_preset_high]
  assert.deepEqual([...vals].sort((a, b) => a - b), vals, 'low < mid < high')
  assert.equal(new Set(vals).size, 3)
})
