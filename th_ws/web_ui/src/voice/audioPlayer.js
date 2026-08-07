// ============================================================
// audioPlayer.js — Web Audio に触る唯一の場所
//
// キュー規則 (voiceQueue.js) はこのファイルの中身を知らない。
// 逆にこのファイルは優先度もレイヤも知らず、渡された1件を鳴らすだけ。
//
// 再生は優先順に3段のフォールバックで試す:
//   1. entry.clips (動的な数値クリップ。呼び出し時に voiceQueue.announce の
//      overrides で渡される。VISION.md §7.5) の WAV
//   2. entry.file (マニフェストの静的な1ファイル) の WAV
//   3. ビープ合成 (Tier 1 のプレースホルダ。1・2 がどちらも取得できない場合)
// ============================================================

let ctx        = null   // AudioContext (ユーザー操作が来るまで生成しない)
let masterGain = null
const bufferCache = new Map()   // url → AudioBuffer

function ensureContext() {
  if (ctx) return ctx
  const AC = window.AudioContext || window.webkitAudioContext
  if (!AC) return null
  ctx = new AC()
  masterGain = ctx.createGain()
  masterGain.gain.value = 1.0
  masterGain.connect(ctx.destination)
  return ctx
}

/**
 * 自動再生ポリシーの解除。ユーザー操作のハンドラ内から呼ぶこと。
 * ハンドラの外で呼んでも suspended のままになる。
 */
export function unlockAudio() {
  const c = ensureContext()
  if (!c) return Promise.resolve(false)
  if (c.state === 'running') return Promise.resolve(true)
  return c.resume().then(() => c.state === 'running').catch(() => false)
}

/**
 * 'unavailable' | 'uninitialized' | 'suspended' | 'running' | 'closed'
 * トグルが ON なのに鳴らない場合、ここが 'suspended' のまま残る。
 */
export function getAudioState() {
  const AC = window.AudioContext || window.webkitAudioContext
  if (!AC) return 'unavailable'
  if (!ctx) return 'uninitialized'
  return ctx.state
}

// ── WAV の読み込み (Tier 3 用の本番経路) ──────────────────
// entry.file は既に拡張子付き ('N4.mp3')。一方 entry.clips / overrides.clips は
// クリップ id を裸のまま持つ (['hokaku','3','meter'])。どちらも同じ場所から
// 読めるよう、拡張子が無ければ .mp3 を補う
function withExt(name) {
  return /\.\w+$/.test(name) ? name : `${name}.mp3`
}

function clipUrl(name) {
  const base = import.meta.env.BASE_URL ?? '/'
  return `${base}voice/${withExt(name)}`.replace(/\/{2,}/g, '/')
}

async function loadBuffer(c, name) {
  const url = clipUrl(name)
  const cached = bufferCache.get(url)
  if (cached) return cached
  const res = await fetch(url)
  if (!res.ok) throw new Error(`音声ファイルを取得できません: ${url} (${res.status})`)
  const buf = await c.decodeAudioData(await res.arrayBuffer())
  bufferCache.set(url, buf)
  return buf
}

/**
 * entry から再生を試す候補を優先順に並べた配列を得る (各要素がクリップ名の配列)。
 * 候補が無ければ空配列 (ビープへ)。
 *
 *   1. entry.clips — 呼び出し時に渡された動的クリップ (数値の差し替え。VISION.md §7.5)
 *   2. entry.file  — マニフェストの静的な1ファイル
 *
 * 動的クリップが1つでも取得失敗すると、以前は即ビープに落ちていた。しかし
 * entry.file 側は既に用意されている実音声 (値が焼き込みで不正確なだけ) なので、
 * 先にそちらへフォールバックした方が「仮の値でも喋る」体験を保てる。
 * clips を持たない大半のエントリは file 単独の1候補のみで従来と同じ挙動になる。
 */
function clipAttempts(entry) {
  const attempts = []
  if (entry.clips && entry.clips.length) attempts.push(entry.clips)
  if (entry.file) attempts.push([entry.file])
  return attempts
}

// ── ビープ合成 (Tier 1 のプレースホルダ) ───────────────────
// 実 WAV の 0.4〜1.7 秒に対しビープは 0.2 秒程度しかないため、
// 鳴らし終えたあと approxSec まで無音で保持する。これがないと
// キューが一瞬で掃けてしまい、順次再生も割り込みも観測できない。
function scheduleBeep(c, spec, startAt) {
  const nodes = []
  const { wave, gain, freq, ms, repeats, gapMs } = spec
  let t = startAt
  for (let i = 0; i < repeats; i++) {
    const osc = c.createOscillator()
    const env = c.createGain()
    const dur = ms / 1000

    osc.type = wave
    osc.frequency.value = freq
    // 矩形波をそのまま切るとプチッと鳴るので短いフェードを付ける
    env.gain.setValueAtTime(0, t)
    env.gain.linearRampToValueAtTime(gain, t + 0.008)
    env.gain.setValueAtTime(gain, t + dur - 0.008)
    env.gain.linearRampToValueAtTime(0, t + dur)

    osc.connect(env)
    env.connect(masterGain)
    osc.start(t)
    osc.stop(t + dur)
    nodes.push(osc)

    t += dur + gapMs / 1000
  }
  const totalMs = repeats * ms + Math.max(0, repeats - 1) * gapMs
  return { nodes, totalMs }
}

/**
 * 1件を再生する。
 * 戻り値 { done, cancel }:
 *   done   — 再生し終えると解決する Promise。cancel された場合も解決する
 *   cancel — 即座に音を止めて done を解決する
 *
 * 音が出せない状況 (未解除・AudioContext 非対応) でも例外は投げず、
 * approxSec 分だけ無音で待ってから解決する。キューを詰まらせないため。
 */
export function playEntry(entry) {
  const c = ensureContext()
  let cancelled = false
  let timerId   = null
  let liveNodes = []
  let resolveDone

  const done = new Promise((resolve) => { resolveDone = resolve })

  const finish = (reason) => {
    if (timerId !== null) { clearTimeout(timerId); timerId = null }
    resolveDone(reason)
  }

  const cancel = () => {
    if (cancelled) return
    cancelled = true
    for (const n of liveNodes) {
      try { n.stop() } catch { /* 既に停止済み */ }
    }
    liveNodes = []
    finish('cancelled')
  }

  const holdSilently = () => {
    timerId = setTimeout(() => finish('done'), Math.round(entry.approxSec * 1000))
  }

  if (!c || c.state !== 'running') {
    // 未解除。無音で尺だけ消費する
    holdSilently()
    return { done, cancel }
  }

  const playBeep = () => {
    const { nodes, totalMs } = scheduleBeep(c, entry.beep, c.currentTime + 0.01)
    liveNodes = nodes
    const holdMs = Math.max(totalMs, entry.approxSec * 1000)
    timerId = setTimeout(() => finish('done'), Math.round(holdMs))
  }

  const attempts = clipAttempts(entry)

  if (!attempts.length) {
    playBeep()
    return { done, cancel }
  }

  // WAV 経路。1候補内の複数クリップは前から順に隙間なく連結する (数値分割合成)。
  // 候補が取得失敗したら次の候補へ、全滅したらビープへ落ちる
  const tryAttempt = (i) => {
    Promise.all(attempts[i].map((n) => loadBuffer(c, n)))
      .then((buffers) => {
        if (cancelled) return
        let t = c.currentTime + 0.01
        for (const buf of buffers) {
          const src = c.createBufferSource()
          src.buffer = buf
          src.connect(masterGain)
          src.start(t)
          liveNodes.push(src)
          t += buf.duration
        }
        const holdMs = (t - c.currentTime) * 1000
        timerId = setTimeout(() => finish('done'), Math.round(holdMs))
      })
      .catch((e) => {
        if (cancelled) return
        const next = i + 1
        if (next < attempts.length) {
          console.warn('[voice] クリップを取得できないため次の候補へ切替:', entry.id, attempts[i], e)
          tryAttempt(next)
          return
        }
        // 未生成・配信漏れでも無音にせず、プレースホルダのビープに落とす。
        // 「鳴らない」より「仮の音が鳴る」方が異常に気付きやすい
        console.warn('[voice] 音声ファイルを再生できないためビープに切替:', entry.id, e)
        playBeep()
      })
  }
  tryAttempt(0)

  return { done, cancel }
}
