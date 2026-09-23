// WP-DEV-01B: ros/devModeState.js（/system/dev_mode の純粋部）の試験。
// React を import しない純粋モジュールなので node --test で直接回る。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  DEV_ITEMS, DEV_NO_GATE_ITEMS, DEV_MODE_NODE, DEV_PARAM_MASTER,
  devIgnoreParam, parseDevModeState, effectiveItems,
  SET_PARAMS_SVC, SET_PARAMS_TYPE, setBoolParamRequest,
} from '../../src/ros/devModeState.js'

const FULL = {
  dev_mode: true,
  ignore: {
    link: true, lidar_fault: true, scan_stop: false,
    battery: false, opcheck: true, auto_brake: false,
  },
  effective: {
    link: true, lidar_fault: true, scan_stop: false,
    battery: false, opcheck: false, auto_brake: false,
  },
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
    ignore: {
      link: false, lidar_fault: false, scan_stop: false,
      battery: false, opcheck: false, auto_brake: false,
    },
    effective: {
      link: false, lidar_fault: false, scan_stop: false,
      battery: false, opcheck: false, auto_brake: false,
    },
    estop_hw_known: false,
    estop_hw_pressed: false,
  })
})

test('effectiveItems: 実効 true の項目だけを DEV_ITEMS 順で返す', () => {
  assert.deepEqual(effectiveItems(parseDevModeState(FULL)), ['link', 'lidar_fault'])
  assert.deepEqual(effectiveItems(null), [])
  assert.deepEqual(parseDevModeState('{}') && effectiveItems(parseDevModeState('{}')), [])
})

test('定数: ノード・パラメータ名は ROS 側の実装と一致する', () => {
  // connectivity_checker.py の _DEV_ITEMS・th_safety/dev_mode_core.hpp と揃える。
  // 名を変えるときは ROS 側も同時に直すこと（test_dev_mode.py が 3 者の一致を縛る）。
  assert.deepEqual(DEV_ITEMS,
    ['link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake'])
  assert.deepEqual(DEV_NO_GATE_ITEMS, ['battery', 'opcheck', 'auto_brake'])
  assert.equal(DEV_MODE_NODE, 'connectivity_checker')
  assert.equal(DEV_PARAM_MASTER, 'dev_mode')
  assert.equal(devIgnoreParam('link'), 'dev_ignore_link')
})

// ── 本番経路の送信（setBoolParamRequest） ──
// TEST_MODE を有効にしない（window.__thTestState を定義しない）ことが要件。
// 偽の window.ROSLIB を注入し、サービス名・型・ペイロードを検証する。
// useDevMode.js のフック本体は React が要るためここでは触らず、
// フックが呼ぶ中身（純粋関数）を直接試験する。
function makeFakeRoslib(capture, { response, error } = {}) {
  class FakeService {
    constructor(opts) {
      capture.svcName = opts.name
      capture.svcType = opts.serviceType
      capture.svcRos = opts.ros
    }
    callService(req, ok, err) {
      capture.request = req
      if (error) err(error)
      else ok(response ?? { results: [{ successful: true }] })
    }
  }
  class FakeRequest {
    constructor(obj) { Object.assign(this, obj) }
  }
  return { Service: FakeService, ServiceRequest: FakeRequest }
}

test('setBoolParamRequest: サービス名と型が正しい', async () => {
  const capture = {}
  const ros = {}
  const res = await setBoolParamRequest({
    ROSLIB: makeFakeRoslib(capture), ros, node: 'connectivity_checker',
    name: 'dev_mode', value: true,
  })
  assert.equal(capture.svcName, '/connectivity_checker/set_parameters')
  assert.equal(capture.svcType, 'rcl_interfaces/SetParameters')
  assert.equal(capture.svcRos, ros)
  assert.deepEqual(res, { results: [{ successful: true }] })
})

test('setBoolParamRequest: ペイロードは bool 型で正しく詰める', async () => {
  for (const [name, value] of [['dev_mode', true], ['dev_ignore_link', false]]) {
    const capture = {}
    await setBoolParamRequest({
      ROSLIB: makeFakeRoslib(capture), ros: {}, node: 'connectivity_checker',
      name, value,
    })
    assert.deepEqual(capture.request.parameters, [
      { name, value: { type: 1, bool_value: value } },
    ])
  }
})

test('setBoolParamRequest: サービス側エラーは reject する', async () => {
  await assert.rejects(
    setBoolParamRequest({
      ROSLIB: makeFakeRoslib({}, { error: new Error('boom') }),
      ros: {}, node: 'connectivity_checker', name: 'dev_mode', value: true,
    }),
    /boom/,
  )
})

test('setBoolParamRequest: ROSLIB 不在は reject する', async () => {
  await assert.rejects(
    setBoolParamRequest({
      ROSLIB: undefined, ros: {}, node: 'connectivity_checker',
      name: 'dev_mode', value: true,
    }),
    /roslibjs is not loaded/,
  )
})
