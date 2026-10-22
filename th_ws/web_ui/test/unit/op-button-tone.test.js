// SG-C2 (1b-12): 操作カードの色の決め方。Spec-webui.md §3.3.1（U-17）の表をそのまま
// 試験の表にする。
//   青（塗り）= いまの状態（停止／確認／走行のうち、いま該当している 1 つだけ）
//   緑（塗り）= 押すと確定する動作（保存）
//   赤（塗り）= 次に押す停止（青が走行／確認のとき）
//   白い太枠  = 次に押す走行（青が停止のとき）
//   面＋枠    = 押せるがそれ以外
import test from 'node:test'
import assert from 'node:assert/strict'
import { opButtonTone, stateToBlueButton } from '../../src/shell/limits.js'

const BLUES = ['stop', 'check', 'run', null]
// [slot, blue] -> tone。U-17 の表。
const TABLE = [
  // 停止／確認／走行は、いま該当している 1 つだけが青、残りは枠のみ
  ['stop', 'stop', 'current'], ['stop', 'check', 'next-stop'], ['stop', 'run', 'next-stop'], ['stop', null, 'face'],
  ['check', 'stop', 'face'], ['check', 'check', 'current'], ['check', 'run', 'face'], ['check', null, 'face'],
  ['run', 'stop', 'next-run'], ['run', 'check', 'face'], ['run', 'run', 'current'], ['run', null, 'face'],
  // 保存は状態によらず緑（塗り）
  ['save', 'stop', 'confirm'], ['save', 'check', 'confirm'], ['save', 'run', 'confirm'], ['save', null, 'confirm'],
  // 手動は常に枠のみ（強調のために青を使わない）
  ['manual', 'stop', 'face'], ['manual', 'check', 'face'], ['manual', 'run', 'face'], ['manual', null, 'face'],
]

for (const [slot, blue, tone] of TABLE) {
  test(`U-17: ${slot} / いまの状態=${blue} -> ${tone}`, () => {
    assert.equal(opButtonTone(slot, blue), tone)
  })
}

test('U-17: 青（current）はどの状態でも同じ操作カードに 2 つ以上出ない', () => {
  for (const blue of BLUES) {
    const blues = ['stop', 'check', 'run', 'save', 'manual']
      .filter((slot) => opButtonTone(slot, blue) === 'current')
    assert.ok(blues.length <= 1, `blue=${blue} で青が ${blues.length} 個: ${blues}`)
  }
})

test('U-17: 確認が青のとき走行は青でない（青が 2 つ並ばない）', () => {
  const attrs = { FOLLOW: { run_state: 'RUN', resume_state: 'PAUSE' } }
  const blue = stateToBlueButton('FOLLOW', 'CONFIRM', attrs)
  assert.equal(blue, 'check')
  assert.equal(opButtonTone('check', blue), 'current')
  assert.equal(opButtonTone('run', blue), 'face')
  assert.equal(opButtonTone('stop', blue), 'next-stop')
})

test('次に押す停止（赤）・次に押す走行（白い太枠）は、どの状態でも 1 つずつしか出ない', () => {
  for (const blue of BLUES) {
    for (const t of ['next-stop', 'next-run']) {
      const n = ['stop', 'check', 'run', 'save', 'manual']
        .filter((slot) => opButtonTone(slot, blue) === t).length
      assert.ok(n <= 1, `blue=${blue} で ${t} が ${n} 個`)
    }
  }
  // 赤と白枠が同時に出ることもない
  for (const blue of BLUES) {
    const tones = ['stop', 'check', 'run', 'save', 'manual'].map((s) => opButtonTone(s, blue))
    assert.ok(!(tones.includes('next-stop') && tones.includes('next-run')), `blue=${blue}`)
  }
})

test('透明な枠のみ（outline）はもう返さない', () => {
  for (const blue of BLUES) {
    for (const slot of ['stop', 'check', 'run', 'save', 'manual']) {
      assert.notEqual(opButtonTone(slot, blue), 'outline')
    }
  }
})
