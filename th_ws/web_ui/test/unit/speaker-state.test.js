// ros/speakerState.js（/speaker/cue・/speaker/status の純粋部）の試験。
// React を import しない純粋モジュールなので node --test で直接回る。
import test from 'node:test'
import assert from 'node:assert/strict'
import { TOPICS } from '../../src/ros/topics.js'
import {
  SPEAKER_NODE, SPEAKER_PARAM_ENABLED, SPEAKER_PARAM_VOLUME, SPEAKER_PARAM_TEST,
  parseSpeakerCue, parseSpeakerStatus, subscribeSpeaker,
} from '../../src/ros/speakerState.js'

const CUE = JSON.stringify({ cue: 'drive', enabled: true, volume: 60, test: false, rpi_ok: true })

test('parseSpeakerCue: JSON 文字列（実物の形）を読む', () => {
  assert.deepEqual(parseSpeakerCue(CUE), {
    cue: 'drive', enabled: true, volume: 60, test: false, rpi_ok: true,
  })
})

test('parseSpeakerCue: オブジェクトも受ける（TEST_MODE の種）', () => {
  assert.equal(parseSpeakerCue({ cue: 'blocked', enabled: false, volume: 30 }).cue, 'blocked')
})

test('parseSpeakerCue: 壊れた文字列・未知の cue・非数値の volume は null', () => {
  assert.equal(parseSpeakerCue('{nope'), null)
  assert.equal(parseSpeakerCue(null), null)
  assert.equal(parseSpeakerCue(JSON.stringify({ cue: 'loud', enabled: true, volume: 60 })), null)
  assert.equal(parseSpeakerCue(JSON.stringify({ cue: 'drive', enabled: true, volume: '60' })), null)
  assert.equal(parseSpeakerCue(JSON.stringify({ cue: 'drive', enabled: true, volume: true })), null)
})

test('parseSpeakerCue: volume は 0〜100 に丸める', () => {
  assert.equal(parseSpeakerCue(JSON.stringify({ cue: 'drive', enabled: true, volume: 120 })).volume, 100)
  assert.equal(parseSpeakerCue(JSON.stringify({ cue: 'drive', enabled: true, volume: -5 })).volume, 0)
})

test('parseSpeakerStatus: 再生中の cue・音量・エラーを読む', () => {
  assert.deepEqual(
    parseSpeakerStatus(JSON.stringify({ cue: 'warning', volume: 60, error: 'paplay が無い' })),
    { cue: 'warning', volume: 60, error: 'paplay が無い' })
  assert.equal(parseSpeakerStatus('broken'), null)
})

test('購読は TOPICS の名前・std_msgs/String で張る（undefined を購読しない）', () => {
  assert.equal(TOPICS.SPEAKER_CUE, '/speaker/cue')
  assert.equal(TOPICS.SPEAKER_STATUS, '/speaker/status')
  const made = []
  class FakeTopic {
    constructor(o) { made.push(o) }
    subscribe() {}
    unsubscribe() {}
  }
  const unsub = subscribeSpeaker({ Topic: FakeTopic }, {}, TOPICS.SPEAKER_CUE,
    TOPICS.SPEAKER_STATUS, () => {}, () => {})
  assert.equal(made.length, 2)
  assert.equal(made[0].name, '/speaker/cue')
  assert.equal(made[1].name, '/speaker/status')
  assert.ok(made.every((m) => m.messageType === 'std_msgs/String'))
  unsub()
})

test('パラメータ名はノード側（speaker_announcer.py）と一致する', () => {
  assert.equal(SPEAKER_NODE, 'speaker_announcer')
  assert.equal(SPEAKER_PARAM_ENABLED, 'speaker_enabled')
  assert.equal(SPEAKER_PARAM_VOLUME, 'speaker_volume')
  assert.equal(SPEAKER_PARAM_TEST, 'test_chime')
})
