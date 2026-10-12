// フォルト履歴の純関数（ros/faultHistoryState.js）。時刻・継続時間の表記、
// 「解消した異常」の選び方、「ほか N 回」、購読の組み立て。
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  parseFaultHistory, formatClock, formatDuration, recordDurationSec, latestRecord,
  latestResolved, repeatCount, resolvedSummary, recentFaultText, subscribeFaultHistory,
} from '../../src/ros/faultHistoryState.js'

const rec = (o = {}) => ({
  fault_type: 'LIDAR_LOST', started_at: 1000, duration_s: 0.1, ongoing: false,
  mode: 'FOLLOW', state: 'RUN', severity: 'RECOVERABLE', outcome: 'PAUSE', ...o,
})

test('継続時間: 0.1 秒を「0 秒」にしない／12 秒／3 分 5 秒', () => {
  assert.equal(formatDuration(0.1), '0.1 秒')
  assert.equal(formatDuration(0.04), '0.1 秒')
  assert.equal(formatDuration(2.34), '2.3 秒')
  assert.equal(formatDuration(9.97), '10 秒')
  assert.equal(formatDuration(12), '12 秒')
  assert.equal(formatDuration(12.4), '12 秒')
  assert.equal(formatDuration(185), '3 分 5 秒')
  assert.equal(formatDuration(180), '3 分')
  assert.equal(formatDuration(-1), '')
})

test('時刻: 時:分:秒（ローカル）。0 埋め', () => {
  const d = new Date(2026, 9, 12, 3, 4, 5)
  assert.equal(formatClock(d.getTime() / 1000), '03:04:05')
  assert.equal(formatClock(0), '--:--:--')
})

test('parse: 新しい順に並べ直し、50 件で切り、読めなければ null', () => {
  assert.equal(parseFaultHistory(null), null)
  assert.equal(parseFaultHistory({}), null)
  const many = Array.from({ length: 60 }, (_, i) => rec({ started_at: i }))
  const out = parseFaultHistory({ records: many })
  assert.equal(out.length, 50)
  assert.equal(out[0].started_at, 59)
  assert.deepEqual(parseFaultHistory({ records: [] }), [])
})

test('継続中の継続時間は started_at と現在時刻から求める', () => {
  const r = rec({ ongoing: true, duration_s: -1, started_at: 100 })
  assert.equal(recordDurationSec(r, 112), 12)
  assert.equal(recordDurationSec(rec({ duration_s: 0.1 }), 999), 0.1)
})

test('直近に解消した記録: ongoing=false の最新。継続中は選ばない', () => {
  const recs = [
    rec({ fault_type: 'MUX_DEAD', started_at: 300, ongoing: true, duration_s: -1 }),
    rec({ started_at: 200, duration_s: 0.1 }),
    rec({ started_at: 100, duration_s: 5 }),
  ]
  assert.equal(latestResolved(recs).started_at, 200)
  assert.equal(latestResolved([recs[0]]), null)
  assert.equal(latestRecord(recs).fault_type, 'MUX_DEAD')
  assert.equal(latestRecord([]), null)
})

test('「ほか N 回」: 直前 60 秒以内の同種別だけ数える', () => {
  const recs = [
    rec({ started_at: 1000 }),
    rec({ started_at: 990 }),
    rec({ started_at: 950 }),
    rec({ started_at: 930 }),                       // 70 秒前: 数えない
    rec({ started_at: 995, fault_type: 'MUX_DEAD' }), // 別種別: 数えない
  ]
  assert.equal(repeatCount(recs, recs[0]), 2)
  const s = resolvedSummary(recs, recs[0])
  assert.equal(s.others, 2)
  assert.equal(s.label, 'LiDAR 途絶')
  assert.equal(s.duration, '0.1 秒')
})

test('ヘッダの文言', () => {
  const d = new Date(2026, 9, 12, 12, 34, 56)
  assert.equal(recentFaultText(rec({ started_at: d.getTime() / 1000 })), '直近の異常：LiDAR 途絶 12:34:56')
  assert.equal(recentFaultText(null), null)
})

test('購読: FaultHistory を queue_length 1 で購読し、受信を渡す', () => {
  const made = []
  class Topic {
    constructor(o) { this.o = o; made.push(this) }
    subscribe(cb) { this.cb = cb }
  }
  const got = []
  const t = subscribeFaultHistory({ Topic }, 'ROS', '/system/fault_history', 'th_system_msgs/FaultHistory', (m) => got.push(m))
  assert.equal(made.length, 1)
  assert.equal(t.o.name, '/system/fault_history')
  assert.equal(t.o.messageType, 'th_system_msgs/FaultHistory')
  assert.equal(t.o.queue_length, 1)
  assert.equal(t.o.ros, 'ROS')
  assert.equal(t.o.throttle_rate, undefined)
  t.cb({ records: [] })
  assert.equal(got.length, 1)
})
