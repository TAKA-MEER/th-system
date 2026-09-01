// DetailedDesign-webui.md §9-7: i18n/modes.js must match Spec-webui.md §8.1's
// 18-row table verbatim. The expected table below is transcribed directly
// from that section (docs/plan/spec/Spec-webui.md lines ~691-702) -- this is
// the "snapshot" the design doc calls for.
import test from 'node:test'
import assert from 'node:assert/strict'
import { MODE_NAMES } from '../../src/i18n/modes.js'

const EXPECTED = {
  INIT: '起動中',
  IDLE: '待機',
  ESTOP: '非常停止',
  CARRY: '手押し',
  FOLLOW: '追従走行',
  MANUAL: '手動走行',
  TEACH_FOLLOW: '教示（追従）',
  TEACH_MANUAL: '教示（手動）',
  REPLAY: '教示再生',
  LINE: 'ライン誘導',
  LEASH: '電子リード',
  PREP: '試験準備',
  PANEL_NAV: '配電盤へ移動',
  AT_PANEL: '配電盤前',
  SUMMON: '呼び寄せ',
  HOME_NAV: '待機場所へ移動',
  OPCHECK: '始業点検',
  CALIB: '校正',
}

test('i18n/modes.js has exactly the 18 rows of Spec-webui.md §8.1', () => {
  assert.deepEqual(MODE_NAMES, EXPECTED)
  assert.equal(Object.keys(MODE_NAMES).length, 18)
})
