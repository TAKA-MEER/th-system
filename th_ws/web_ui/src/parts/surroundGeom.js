// parts/surroundGeom.js — S-11 の計器と機体のまわりの図の純関数
// (Spec-webui.md §3.5・D81・D82)。
//
// ここは計算だけ（ROS/React に依存しない。node:test で直接テストする）。
// 数値はすべて機体から届いた値（WheelFeedback 実速度・指令、/odom の twist、
// LimiterStatus）から求める。届かない値（モーター出力・エンコーダ）は
// ここでも作らない（画面側で「—」を出す）。

import { ROBOT_HALF_M } from './mapColors.js'

export const GUIDE_LENGTH_M = 1.5
export const GUIDE_STEP_M = 0.05
export const GAUGE_HISTORY_SEC = 5
// その場で回っているとみなす回転の下限 (rad/s)。これ未満は止まっている扱い。
export const ROT_EPS = 0.02

// 1 サンプル {left,right} の前進速度 (m/s)。
export function speedOfSample(s) {
  if (!s || typeof s.left !== 'number' || typeof s.right !== 'number') return null
  return (s.left + s.right) / 2
}

// 最新サンプルから速度 (m/s)。無ければ null。
export function latestSpeed(samples) {
  if (!samples || samples.length === 0) return null
  return speedOfSample(samples[samples.length - 1])
}

// 左右の速度差から回転 (rad/s) を求める。track は車輪間距離 (m)。
// track が無い/不正なら null（推測しない）。
export function angularFromWheels(s, trackM) {
  if (!s || typeof s.left !== 'number' || typeof s.right !== 'number') return null
  if (typeof trackM !== 'number' || !(trackM > 0)) return null
  return (s.right - s.left) / trackM
}

// 直近 GAUGE_HISTORY_SEC 秒の履歴（[{t, v}]）。時刻は秒。
export function history5(samples, nowSec) {
  if (!samples || samples.length === 0 || typeof nowSec !== 'number') return []
  const cutoff = nowSec - GAUGE_HISTORY_SEC
  const out = []
  for (const s of samples) {
    const v = speedOfSample(s)
    if (v == null || typeof s.t !== 'number' || s.t < cutoff) continue
    out.push({ t: s.t, v })
  }
  return out
}

// 速度の上限 (m/s)。applied_limit_mps が届いていればそれを、無ければ null。
export function limitOf(limiter) {
  const v = limiter?.applied_limit_mps
  return typeof v === 'number' && Number.isFinite(v) && v > 0 ? v : null
}

// 自動ブレーキで削った分 (m/s)。in_linear - out_linear が正で、かつ
// 実際に削っている (action が PASS でない) ときだけ。無ければ 0。
export function brakeCutOf(limiter) {
  if (!limiter) return 0
  const { in_linear: inL, out_linear: outL, action } = limiter
  if (typeof inL !== 'number' || typeof outL !== 'number') return 0
  if (action == null || action === 'PASS') return 0
  return Math.max(0, inL - outL)
}

// 目標と実際の乖離 (m/s)。どちらかが無ければ null。
export function gapOf(actual, command) {
  if (actual == null || command == null) return null
  return Math.abs(command - actual)
}

// 進む向きのガイド線（機体座標。x=前方、y=左、原点=機体中心）。
//   止まっている（速度・回転とも小さい）-> null（出さない）
//   その場で回っている -> { kind:'arc', ... }（機体のまわりの弧の矢印）
//   それ以外 -> { kind:'path', points }（いまの速さと回転から予想した進路。
//     前進は前、後退は後ろから 1.5 m、0.5 m ごとに横線を引けるよう点を返す）
// points は機体の外形の縁から始まる（外形の下に隠れない）。
export function guideFor(v, wz) {
  const vv = typeof v === 'number' && Number.isFinite(v) ? v : 0
  const ww = typeof wz === 'number' && Number.isFinite(wz) ? wz : 0
  const moving = Math.abs(vv) >= ROT_EPS
  const turning = Math.abs(ww) >= ROT_EPS
  if (!moving && !turning) return null
  if (!moving && turning) {
    // その場回転: 弧の矢印。半径は機体の外形＋少し外。
    const r = ROBOT_HALF_M + 0.15
    const sweep = ww > 0 ? 1 : 0 // 左回りは反時計回り
    // 弧は前方から回転方向へ 90°（描画側で sweep を使う）
    return { kind: 'arc', r, sweep, fromDeg: -90, toDeg: -90 + (ww > 0 ? 90 : -90) }
  }
  // 進路は機体の向きが回転しながら進む軌跡（いまの速さと回転で積分する）。
  // 後退 (v<0) は後ろから始まり、速さの符号でそのまま後ろへ進む。
  const dir = vv >= 0 ? 1 : -1
  const dt = GUIDE_STEP_M / Math.abs(vv)
  const x0 = dir * ROBOT_HALF_M
  const points = [{ x: x0, y: 0 }]
  let x = x0
  let y = 0
  let phi = 0
  const steps = Math.round(GUIDE_LENGTH_M / GUIDE_STEP_M)
  for (let i = 0; i < steps; i += 1) {
    phi += ww * dt
    x += vv * Math.cos(phi) * dt
    y += vv * Math.sin(phi) * dt
    points.push({ x, y })
  }
  return { kind: 'path', points }
}
