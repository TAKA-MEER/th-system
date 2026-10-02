// S-14 経路からのずれ（Spec-webui.md §3.7）: いまのずれと最大値を小数 2 桁で出す。
import test from 'node:test'
import assert from 'node:assert/strict'
import { S14_XTRACK_TITLE, S14_XTRACK_EMPTY, S14_XTRACK_VALUE } from '../../src/i18n/screens.js'

test('xtrack title is present', () => {
  assert.equal(typeof S14_XTRACK_TITLE, 'string')
  assert.ok(S14_XTRACK_TITLE.length > 0)
})

test('xtrack value shows now and max with 2 decimals', () => {
  assert.equal(S14_XTRACK_VALUE(0.1234, 0.4567), 'いま 0.12 m・最大 0.46 m')
})

test('xtrack value shows zeros at replay start', () => {
  assert.equal(S14_XTRACK_VALUE(0, 0), 'いま 0.00 m・最大 0.00 m')
})

test('xtrack empty placeholder exists for missing status', () => {
  assert.equal(typeof S14_XTRACK_EMPTY, 'string')
})
