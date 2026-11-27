// W0a ヘルプの純関数・設定の縛り。
import test from 'node:test'
import assert from 'node:assert/strict'
import { HELP_TEXTS, helpText } from '../../src/i18n/help.js'

test('ヘルプ文は共通部品のキーを持つ（止める操作は対象外）', () => {
  assert.equal(helpText('stop'), HELP_TEXTS.stop)
  // 止める操作（停止・ジョグ・非常停止）は長押し対象外として載せない。
  // 共通部品にあるのは「停止する」説明だけ（表示用。長押し対象外は部品側の data-nolong相当）。
  assert.ok(helpText('toggle'))
  assert.ok(helpText('finish'))
  assert.equal(helpText('no-such-key'), null)
})
