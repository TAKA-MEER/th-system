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
