// 8-g P3-ui: 盤前の旋回・前後の並進（ADJUST）の画面側の台帳が spec と一致すること。
// 詳細設計 ^ DetailedDesign-measure.md §6.4（P3-ui の表）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { isW1Active, isWorkingActive } from '../../src/shell/limits.js'
import { onsiteReasons } from '../../src/screens/onsiteSteps.js'
import { stateLabel, STATE_NAMES } from '../../src/i18n/states.js'
import { REJECT_REASONS, reasonLabel } from '../../src/i18n/reasons.js'

test('isWorkingActive: WORKING・ADJUST は真、IDLE_P は偽（working が無い）', () => {
  assert.equal(isWorkingActive({ state: 'WORKING' }), true)
  assert.equal(isWorkingActive({ state: 'ADJUST' }), true)
  assert.equal(isWorkingActive({ state: 'IDLE_P' }), false)
  assert.equal(isWorkingActive({ state: 'PAUSE' }), false)
  assert.equal(isWorkingActive(null), false)
  assert.equal(isWorkingActive(undefined), false)
})

test('isWorkingActive: /system/state.working があればそれを優先する', () => {
  assert.equal(isWorkingActive({ state: 'IDLE_P', working: true }), true)
  assert.equal(isWorkingActive({ state: 'WORKING', working: false }), false)
  assert.equal(isWorkingActive({ state: 'ADJUST', working: false }), false)
  assert.equal(isWorkingActive({ state: 'ADJUST', working: true }), true)
})

test('i18n/states: ADJUST は「測定機能の指示で動作中」', () => {
  assert.equal(STATE_NAMES.ADJUST, '測定機能の指示で動作中')
  assert.equal(stateLabel('ADJUST'), '測定機能の指示で動作中')
})

test('i18n/reasons: path_obstacle / adjust_in_progress / adjust_timeout の文言', () => {
  assert.equal(REJECT_REASONS.path_obstacle, '進む先に障害物があります')
  assert.equal(REJECT_REASONS.adjust_in_progress, '旋回中・並進中は腕が起きているため解けません')
  assert.equal(REJECT_REASONS.adjust_timeout, '旋回・並進が時間内に終わりませんでした')
  assert.equal(reasonLabel('path_obstacle'), REJECT_REASONS.path_obstacle)
  assert.equal(reasonLabel('adjust_in_progress'), REJECT_REASONS.adjust_in_progress)
  assert.equal(reasonLabel('adjust_timeout'), REJECT_REASONS.adjust_timeout)
})

test('onsiteReasons: working（ADJUST を含む）で行き先・手動が working_in_progress になる', () => {
  const r = onsiteReasons({
    mode: 'AT_PANEL', stateName: 'ADJUST', attributes: { AT_PANEL: { jog: 'ok' } },
    pins: [], working: true,
  })
  assert.equal(r.manual, 'working_in_progress')
  assert.equal(r.destHome, 'working_in_progress')
  assert.equal(r.destNextPanel, 'working_in_progress')
  assert.equal(r.destSummonHere, 'working_in_progress')
})

test('isW1Active: AT_PANEL / ADJUST でフォルトがあれば W-1 を出す', () => {
  assert.equal(isW1Active('AT_PANEL', 'ADJUST', true), true)
  assert.equal(isW1Active('AT_PANEL', 'ADJUST', false), false)
  assert.equal(isW1Active('AT_PANEL', 'WORKING', true), true)
  assert.equal(isW1Active('AT_PANEL', 'IDLE_P', true), false)
})
