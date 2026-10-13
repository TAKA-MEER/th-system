// /map_session/status から「会場地図を開いたか」を導く純関数（Spec-webui §3.11 S-21）。
import test from 'node:test'
import assert from 'node:assert/strict'
import { parseMapSession, venueMapOpenedFromStatus, resolveMapOpened } from '../../src/ros/mapSessionState.js'

const st = (mode, slot = 'VENUE') => parseMapSession({ slot, mode, dirty: false, instance_id: 'x' })

test('parseMapSession: 読めなければ null', () => {
  assert.equal(parseMapSession(null), null)
  assert.equal(parseMapSession({}), null)
  assert.deepEqual(st('LOCALIZING'), { slot: 'VENUE', mode: 'LOCALIZING', dirty: false, instance_id: 'x' })
})

test('LOCALIZING の VENUE だけが「開いた」', () => {
  assert.equal(venueMapOpenedFromStatus(st('LOCALIZING')), true)
  assert.equal(venueMapOpenedFromStatus(st('UNLOADED')), false)
  assert.equal(venueMapOpenedFromStatus(st('MAPPING')), false)
  assert.equal(venueMapOpenedFromStatus(st('LOCALIZING', 'ROUTE')), false)
  assert.equal(venueMapOpenedFromStatus(null), null)
})

test('resolveMapOpened: 機体の値を優先し、届く前だけローカルの応答を使う', () => {
  assert.equal(resolveMapOpened(st('LOCALIZING'), undefined), true)    // ローカルが空でも開いた
  assert.equal(resolveMapOpened(st('UNLOADED'), true), false)          // ローカルが残っていても偽
  assert.equal(resolveMapOpened(st('MAPPING'), true), false)
  assert.equal(resolveMapOpened(null, true), true)                     // 届く前は従来どおり
  assert.equal(resolveMapOpened(null, undefined), false)
})
