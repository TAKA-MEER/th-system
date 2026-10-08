// SG-C2 (1b-12): 操作カードの色の決め方。Spec-webui.md §3.3.1（U-17）の表をそのまま
// 試験の表にする。
//   青（塗り）= いまの状態（停止／確認／走行のうち、いま該当している 1 つだけ）
//   緑（塗り）= 押すと確定する動作（保存）
//   枠のみ    = 押せるが、いまの状態ではない（それ以外）
import test from 'node:test'
import assert from 'node:assert/strict'
import { opButtonTone, stateToBlueButton } from '../../src/shell/limits.js'

const BLUES = ['stop', 'check', 'run', null]
// [slot, blue] -> tone。U-17 の表。
const TABLE = [
  // 停止／確認／走行は、いま該当している 1 つだけが青、残りは枠のみ
  ['stop', 'stop', 'current'], ['stop', 'check', 'outline'], ['stop', 'run', 'outline'], ['stop', null, 'outline'],
  ['check', 'stop', 'outline'], ['check', 'check', 'current'], ['check', 'run', 'outline'], ['check', null, 'outline'],
  ['run', 'stop', 'outline'], ['run', 'check', 'outline'], ['run', 'run', 'current'], ['run', null, 'outline'],
  // 保存は状態によらず緑（塗り）
  ['save', 'stop', 'confirm'], ['save', 'check', 'confirm'], ['save', 'run', 'confirm'], ['save', null, 'confirm'],
  // 手動は常に枠のみ（強調のために青を使わない）
  ['manual', 'stop', 'outline'], ['manual', 'check', 'outline'], ['manual', 'run', 'outline'], ['manual', null, 'outline'],
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
  assert.equal(opButtonTone('run', blue), 'outline')
  assert.equal(opButtonTone('stop', blue), 'outline')
})
