// 1b-6 SG-C4: S-00 の総合・再起動・自動判定の日本語表示（i18n/screens.js）。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  S00_NOT_READY, S00_RESTARTING, s00AutoResultLabel,
  SHUTDOWN_BLOCKED_HINT, SHUTDOWN_STOPPING,
} from '../../src/i18n/screens.js'

test('総合欄: 揃わないときの文言（英字なし）', () => {
  assert.equal(S00_NOT_READY, '必須機器が繋がっていません')
  assert.doesNotMatch(S00_NOT_READY, /[A-Za-z]/)
})

test('再起動表示: spec の文言どおり「制御系を再起動しています（n 回目）」', () => {
  assert.equal(S00_RESTARTING(2), '制御系を再起動しています（2 回目）')
  assert.equal(S00_RESTARTING(3), '制御系を再起動しています（3 回目）')
})

test('自動判定の項目別の結果: OK/NG/WARN の英字を出さない', () => {
  assert.equal(s00AutoResultLabel('OK'), '正常')
  assert.equal(s00AutoResultLabel('WARN'), '要確認')
  assert.equal(s00AutoResultLabel('NG'), '異常')
  for (const label of ['正常', '要確認', '異常', s00AutoResultLabel('CHECKING')]) {
    assert.doesNotMatch(label, /^(OK|NG|WARN)$/)
  }
})

test('停止の進捗とブロック理由の文言がある', () => {
  assert.equal(typeof SHUTDOWN_STOPPING, 'string')
  assert.match(SHUTDOWN_BLOCKED_HINT(2), /残り 2 件/)
})
