// 1b-6 (SG-B2): 遷移表の全 guide キーに文言があることを縛る。
// キーの正は th_ws/src/th_state/config/transitions.yaml の guide の `key`
// (guides.js 側を直す。逆にしない)。speed-presets-from-registry.test.js と
// 同じ流儀で、YAML を正規表現で読む (Node 側に YAML 依存を増やさない)。
// 変異: キーを古い名前に戻す (例: summon_clear_timeout -> wait_clear_timeout)
// と赤くなること。
import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { GUIDES, guideLabel } from '../../src/i18n/guides.js'

const yaml = readFileSync(
  fileURLToPath(new URL('../../../src/th_state/config/transitions.yaml', import.meta.url)),
  'utf8',
)

function guideKeysFromTransitions() {
  const keys = []
  const re = /- name: guide\n\s+args:\n\s+key: (\S+)/g
  let m
  while ((m = re.exec(yaml)) !== null) keys.push(m[1])
  return keys
}

test('遷移表から guide キーが読める (読み取り自体の壊れを検出)', () => {
  const keys = guideKeysFromTransitions()
  assert.ok(keys.length > 0, 'guide キーが 0 件。YAML の書式か正規表現が壊れている')
  assert.deepEqual([...new Set(keys)].sort(), [...keys].sort(), 'guide キーに重複があれば見る')
})

test('遷移表の全 guide キーに文言がある', () => {
  for (const key of guideKeysFromTransitions()) {
    assert.ok(GUIDES[key], `${key} の文言が guides.js に無い`)
    assert.ok(GUIDES[key].length > 0, `${key} の文言が空`)
    assert.equal(guideLabel(key), GUIDES[key])
  }
})

test('古いキー名は残っていない', () => {
  assert.ok(!('wait_clear_timeout' in GUIDES), 'wait_clear_timeout は summon_clear_timeout に直したはず')
  // target_lost は T-FOLLOW-06 の正規キーなので残す (register_target_lost とは別)。
})
