// shell/stopRun.js — 停止│走行のトグルの判断（Spec-webui §3.3.2）。純関数。
//
// 語は動詞で固定（呼び出し側が渡す。「停止する」「走行する」「再開する」「再生する」）。
// 塗るのはいま押せる側だけ（止める＝黒・走る＝緑）。予約の間は押せない見た目。
// 状態を言うのはヘッダの札だけ（トグルは言わない）。

// stopRunSides({ stopEnabled, runEnabled, reserved }) ->
//   { stop: 'paint-black'|'plain', run: 'paint-green'|'plain' }
// reserved=true（試験準備の予約）なら両方 plain（押せない見た目）。
export function stopRunSides({ stopEnabled = false, runEnabled = false, reserved = false } = {}) {
  if (reserved) return { stop: 'plain', run: 'plain' }
  return {
    stop: stopEnabled ? 'paint-black' : 'plain',
    run: runEnabled ? 'paint-green' : 'plain',
  }
}
