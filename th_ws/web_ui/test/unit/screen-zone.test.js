// 場内の画面（ゾーン IN）の判定。ブラウザ側の写し（src/screens/screenZone.js）が
// 正本 th_state/zones.py の SCREEN_ZONE と食い違わないことを縛る
// （Spec-webui.md「速度プリセット」: 場内の画面では高速を出さない）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { IN_ZONE_SCREEN_IDS, isInZoneScreen } from '../../src/screens/screenZone.js'

const zonesPy = readFileSync(
  fileURLToPath(new URL('../../../src/th_state/th_state/zones.py', import.meta.url)), 'utf8')

function screenZoneFromPython() {
  const block = /SCREEN_ZONE: Dict\[str, str\] = \{([\s\S]*?)\n\}/.exec(zonesPy)[1]
  const out = {}
  for (const m of block.matchAll(/"(S-\d+)":\s*"(\w+)"/g)) out[m[1]] = m[2]
  return out
}

test('IN_ZONE_SCREEN_IDS は zones.py の SCREEN_ZONE で IN の画面と一致する', () => {
  const zones = screenZoneFromPython()
  assert.ok(Object.keys(zones).length >= 10, 'zones.py の読み取りに失敗')
  const inScreens = Object.keys(zones).filter((k) => zones[k] === 'IN').sort()
  assert.deepEqual([...IN_ZONE_SCREEN_IDS].sort(), inScreens)
})

test('isInZoneScreen: S-20・S-21 だけ true（場外は従来どおり）', () => {
  assert.equal(isInZoneScreen('S-20'), true)
  assert.equal(isInZoneScreen('S-21'), true)
  for (const id of ['S-10', 'S-11', 'S-13', 'S-14', 'S-15', 'S-16', 'S-50', undefined]) {
    assert.equal(isInZoneScreen(id), false, String(id))
  }
})

import { clampPresetForZone } from '../../src/screens/screenZone.js'
import presets from '../../src/generated/speed_presets.json' with { type: 'json' }

test('clampPresetForZone: 場内では高速→中速に落ち、低速・中速はそのまま。場外は触らない', () => {
  const { speed_preset_low: low, speed_preset_mid: mid, speed_preset_high: high } = presets
  assert.equal(clampPresetForZone(high, true, mid), mid)
  assert.equal(clampPresetForZone(mid, true, mid), mid)
  assert.equal(clampPresetForZone(low, true, mid), low)
  assert.equal(clampPresetForZone(high, false, mid), high)
})
