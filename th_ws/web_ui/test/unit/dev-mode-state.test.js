// WP-DEV-01B: ros/devModeState.js（/system/dev_mode の純粋部）の試験。
// React を import しない純粋モジュールなので node --test で直接回る。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  DEV_ITEMS, DEV_NO_GATE_ITEMS, DEV_MODE_NODE, DEV_PARAM_MASTER,
  devIgnoreParam, parseDevModeState, effectiveItems,
} from '../../src/ros/devModeState.js'

const FULL = {
  dev_mode: true,
  ignore: { link: true, battery: false, opcheck: true, auto_brake: false },
  effective: { link: true, battery: false, opcheck: false, auto_brake: false },
  estop_hw_known: true,
  estop_hw_pressed: false,
}

test('parseDevModeState: JSON 文字列（実物の形）を読む', () => {
  assert.deepEqual(parseDevModeState(JSON.stringify(FULL)), FULL)
})

test('parseDevModeState: オブジェクトも受ける（TEST_MODE の種）', () => {
  assert.deepEqual(parseDevModeState(FULL), FULL)
})

test('parseDevModeState: 壊れた文字列・null・配列は null', () => {
  assert.equal(parseDevModeState('{nope'), null)
  assert.equal(parseDevModeState(''), null)
  assert.equal(parseDevModeState(null), null)
  assert.equal(parseDevModeState([1, 2]), null)
})

test('parseDevModeState: 欠けたキーは偽で埋める（沈黙は安全側＝無視なし）', () => {
  assert.deepEqual(parseDevModeState('{}'), {
    dev_mode: false,
    ignore: { link: false, battery: false, opcheck: false, auto_brake: false },
    effective: { link: false, battery: false, opcheck: false, auto_brake: false },
    estop_hw_known: false,
    estop_hw_pressed: false,
  })
})

test('effectiveItems: 実効 true の項目だけを DEV_ITEMS 順で返す', () => {
  assert.deepEqual(effectiveItems(parseDevModeState(FULL)), ['link'])
  assert.deepEqual(effectiveItems(null), [])
  assert.deepEqual(parseDevModeState('{}') && effectiveItems(parseDevModeState('{}')), [])
})

test('定数: ノード・パラメータ名は ROS 側の実装と一致する', () => {
  // connectivity_checker.py の _DEV_ITEMS / launch の受け取り先と揃える。
  // 名を変えるときは ROS 側も同時に直すこと。
  assert.deepEqual(DEV_ITEMS, ['link', 'battery', 'opcheck', 'auto_brake'])
  assert.deepEqual(DEV_NO_GATE_ITEMS, ['battery', 'opcheck', 'auto_brake'])
  assert.equal(DEV_MODE_NODE, 'connectivity_checker')
  assert.equal(DEV_PARAM_MASTER, 'dev_mode')
  assert.equal(devIgnoreParam('link'), 'dev_ignore_link')
})
