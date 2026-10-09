// S-50 表示タブ（SG-C14）: 音声マスタ・縦横の固定・コストマップ表示。
// いずれも機体を動かさない表示だけの設定で、localStorage が正本。
import test from 'node:test'
import assert from 'node:assert/strict'
import { voiceQueue } from '../../src/voice/voiceQueue.js'
import { readVoiceMaster } from '../../src/parts/voiceMaster.js'
import {
  readStageOrientation, writeStageOrientation,
  ORIENTATION_AUTO,
} from '../../src/parts/stageOrientation.js'
import { readCostmapVisible } from '../../src/parts/costmapVisibility.js'
import { stageMetrics } from '../../src/shell/stageMetrics.js'

test('音声マスタ OFF は announce を黙らせる（鳴らさず false）', () => {
  assert.equal(voiceQueue.isMasterEnabled(), true)
  voiceQueue.setMasterEnabled(false)
  assert.equal(voiceQueue.isMasterEnabled(), false)
  // playEntry に届く前に落とす（node に AudioContext は無いが、ここまで来ない）。
  assert.equal(voiceQueue.announce('A3'), false)
  const snap = voiceQueue.getSnapshot()
  assert.equal(snap.playingId, null)
  assert.deepEqual(snap.queueIds, [])
  // 継続条件のエッジも鳴らない（例外も出ない）。
  voiceQueue.setCondition('A3', true)
  assert.equal(voiceQueue.getSnapshot().playingId, null)
  voiceQueue.setCondition('A3', false)
  voiceQueue.setMasterEnabled(true)
  assert.equal(voiceQueue.isMasterEnabled(), true)
})

test('表示設定の既定値（storage 無し＝初回）', () => {
  assert.equal(readVoiceMaster(), true)
  assert.equal(readStageOrientation(), ORIENTATION_AUTO)
  assert.equal(readCostmapVisible(), true)
  // 不正値は既定に倒す。
  assert.equal(writeStageOrientation('diagonal'), ORIENTATION_AUTO)
})

test('縦横の固定は向きを上書きする（自動は従来どおり）', () => {
  // 横長ビューポート。
  assert.equal(stageMetrics(1280, 800).orientation, 'landscape')
  assert.equal(stageMetrics(1280, 800, { lock: 'portrait' }).orientation, 'portrait')
  assert.equal(stageMetrics(1280, 800, { lock: 'portrait' }).width, 960)
  // 縦長ビューポート。
  assert.equal(stageMetrics(800, 1280).orientation, 'portrait')
  assert.equal(stageMetrics(800, 1280, { lock: 'landscape' }).orientation, 'landscape')
  assert.equal(stageMetrics(800, 1280, { lock: 'landscape' }).width, 1280)
  // 不正な lock は自動と同じ。
  assert.equal(stageMetrics(1280, 800, { lock: 'diagonal' }).orientation, 'landscape')
})
