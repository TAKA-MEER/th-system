// brief-onsite-register-fix REG-1: ピン登録フローの拒否理由 7 件が
// REJECT_REASONS に登録されていることを固定する。pin_registrar.py が返す
// reject_reason_key（no_target / target_lost / low_confidence / no_map_tf /
// no_pending / no_first_point / two_point_too_close）をそのままキーにしているので、
// サーバ側でキーを変えるか UI 側で消すとここが落ちて気づける。
import test from 'node:test'
import assert from 'node:assert/strict'
import { REJECT_REASONS, reasonLabel } from '../../src/i18n/reasons.js'

test('ピン登録の拒否理由 7 件が REJECT_REASONS にある', () => {
  const keys = [
    'no_target',
    'target_lost',
    'low_confidence',
    'no_map_tf',
    'no_pending',
    'no_first_point',
    'two_point_too_close',
  ]
  for (const k of keys) {
    assert.ok(REJECT_REASONS[k], `${k} が REJECT_REASONS に無い`)
    assert.ok(REJECT_REASONS[k].length > 0, `${k} の文言が空`)
  }
})

test('reasonLabel は生キーを返さない（日本語に解決する）', () => {
  assert.equal(reasonLabel('two_point_too_close'), REJECT_REASONS.two_point_too_close)
  assert.notEqual(reasonLabel('two_point_too_close'), 'two_point_too_close')
})

test('登録専用の target_lost と対象選択用の tracker_lost は別キーとして両方残る', () => {
  // 文言が同一（どちらも「対象を見失っています」）でも、キーは別物として
  // 両方登録されている必要がある（サーバが返すキーがどちらでも日本語に解決する）。
  assert.equal(REJECT_REASONS.target_lost, '対象を見失っています')
  assert.equal(REJECT_REASONS.tracker_lost, '対象を見失っています')
  assert.notEqual('target_lost', 'tracker_lost')
  assert.ok(REJECT_REASONS.target_lost)
  assert.ok(REJECT_REASONS.tracker_lost)
})

// brief-tracker-default-off §3.4: state_manager が tracker_enabled=false を
// 拒否する理由キー（tracker_required）が登録されていて、日本語に解決する
// ことを固定する（UI は同じ状態でボタンを予め無効化するが、競合で拒否が
// 届いたとき画面に生キーを出さないため）。
test('tracker_required が REJECT_REASONS にあり日本語に解決する', () => {
  assert.ok(REJECT_REASONS.tracker_required, 'tracker_required が REJECT_REASONS に無い')
  assert.ok(REJECT_REASONS.tracker_required.length > 0)
  assert.equal(reasonLabel('tracker_required'), REJECT_REASONS.tracker_required)
  assert.notEqual(reasonLabel('tracker_required'), 'tracker_required')
})

// 1b-9 SG-A9: 作業中の行き先拒否は state_manager（transitions.yaml T-ATP-05-working）が
// この reject_reason_key で返す。画面の文言が消えると理由が生キーで出る。
test('作業中の拒否理由 working_in_progress が日本語に解決する', () => {
  assert.ok(REJECT_REASONS.working_in_progress)
  assert.equal(reasonLabel('working_in_progress'), REJECT_REASONS.working_in_progress)
})

// 1b-7 S-13: SAVED で「走行」・スティックを拒否した理由 teach_saved_finalized
//（T-TEACH-05/-05J/-05M）が日本語に解決する。S-13 が state.last_reject_reason
// から出す。消えると理由が生キーか「理由不明」で出る。
test('保存確定の拒否理由 teach_saved_finalized が日本語に解決する', () => {
  assert.ok(REJECT_REASONS.teach_saved_finalized)
  assert.equal(reasonLabel('teach_saved_finalized'), REJECT_REASONS.teach_saved_finalized)
  assert.notEqual(reasonLabel('teach_saved_finalized'), 'teach_saved_finalized')
})
