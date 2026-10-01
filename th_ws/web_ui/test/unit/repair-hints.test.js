// S-31 故障診断のチェック手順（generated/repair_hints.json。正本は th_maintenance の
// repair_hints.yaml。WP-MAINT-03）。i18n/checks.js の NG 理由と手順が 1 対 1 で揃っていること。
import test from 'node:test'
import assert from 'node:assert/strict'
import hints from '../../src/generated/repair_hints.json' with { type: 'json' }
import { CHECK_REASON_LABELS } from '../../src/i18n/checks.js'

test('故障診断の対象は ESTOP と MOTOR だけ（IMU・LiDAR は校正へ誘導）', () => {
  assert.deepEqual(Object.keys(hints).sort(), ['ESTOP', 'MOTOR'])
})

test('ESTOP / MOTOR の NG 理由（ラベルのあるもの）すべてに手順がある', () => {
  for (const item of ['ESTOP', 'MOTOR']) {
    for (const reason of Object.keys(CHECK_REASON_LABELS[item])) {
      assert.ok(hints[item][reason]?.length > 0, `${item}.${reason} の手順が無い`)
    }
  }
})

test('手順に使われている理由はラベルにもある（typo を止める）', () => {
  for (const item of ['ESTOP', 'MOTOR']) {
    for (const reason of Object.keys(hints[item])) {
      assert.ok(CHECK_REASON_LABELS[item][reason], `${item}.${reason} に対応する NG 理由が無い`)
    }
  }
})

test('ESTOP の押しても変わらない系は ESTOP_BENCH_TEST_BYPASS の確認を含む（DEBT-1）', () => {
  assert.ok(hints.ESTOP.no_press.some((s) => s.includes('ESTOP_BENCH_TEST_BYPASS')))
})
