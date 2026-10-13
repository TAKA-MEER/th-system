// topics.js の TOPICS／SERVICES／MSG_TYPES の取り違えを縛る。
// 2026-10-13 実機: FAULT_HISTORY が SERVICES の中にあり、購読側が TOPICS.FAULT_HISTORY
// (undefined) で購読を張って「subscribe:undefined」になり、履歴が 1 件も届かなかった。
// TEST_MODE は rosbridge を張らないので e2e では見えない。ここで名前の解決を縛る。
import test from 'node:test'
import assert from 'node:assert/strict'
import { TOPICS, SERVICES, MSG_TYPES } from '../../src/ros/topics.js'
import { subscribeFaultHistory } from '../../src/ros/faultHistoryState.js'

test('TOPICS の値はすべて "/" で始まる文字列', () => {
  for (const [k, v] of Object.entries(TOPICS)) {
    assert.equal(typeof v, 'string', `TOPICS.${k}`)
    assert.ok(v.startsWith('/'), `TOPICS.${k} = ${v}`)
  }
})

test('フォルト履歴は TOPICS にあり、SERVICES には無い', () => {
  assert.equal(TOPICS.FAULT_HISTORY, '/system/fault_history')
  assert.equal(SERVICES.FAULT_HISTORY, undefined)
  assert.equal(MSG_TYPES.FAULT_HISTORY, 'th_system_msgs/FaultHistory')
})

test('useFaultHistory が使う購読は、名前・型が定義済みで張られる（undefined を購読しない）', () => {
  const made = []
  class FakeTopic { constructor(o) { made.push(o); this.o = o } subscribe() {} unsubscribe() {} }
  const ROSLIB = { Topic: FakeTopic }
  subscribeFaultHistory(ROSLIB, {}, TOPICS.FAULT_HISTORY, MSG_TYPES.FAULT_HISTORY, () => {})
  assert.equal(made.length, 1)
  assert.equal(made[0].name, '/system/fault_history')
  assert.equal(made[0].messageType, 'th_system_msgs/FaultHistory')
  assert.equal(made[0].queue_length, 1)
})
