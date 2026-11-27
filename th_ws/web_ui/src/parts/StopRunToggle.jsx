// parts/StopRunToggle.jsx — 停止│走行のトグル（Spec-webui §3.3.2）。
//
// 押せる側だけ塗る（止める＝黒・走る／再開＝緑）。次に押す走行はゆっくり明滅する。
// 「停止する」だけの形（手動系）も同じ部品で出せる（runLabel=null のとき）。
// 判断は shell/stopRun.js。色は theme.css の .stoprun。
import { stopRunSides } from '../shell/stopRun.js'

export default function StopRunToggle({
  stopLabel = '停止する',
  runLabel = '走行する',
  stopEnabled = false,
  runEnabled = false,
  reserved = false,
  stopDenyReason = null,
  runDenyReason = null,
  onStop,
  onRun,
  testId = 'stop-run-toggle',
}) {
  const sides = stopRunSides({ stopEnabled, runEnabled, reserved })
  const showRun = runLabel != null
  return (
    <div className={`stoprun${reserved ? ' reserved' : ''}`} data-testid={testId}>
      <button
        type="button"
        className={`sr-stop ${sides.stop === 'paint-black' ? 'paint' : 'plain'}`}
        data-testid={`${testId}-stop`}
        disabled={reserved || !stopEnabled}
        aria-pressed={sides.stop === 'paint-black'}
        onClick={onStop}
      >
        {stopLabel}
      </button>
      {showRun && <span className="sr-sep" aria-hidden="true" />}
      {showRun && (
        <button
          type="button"
          className={`sr-run ${sides.run === 'paint-green' ? 'paint blink-slow' : 'plain'}`}
          data-testid={`${testId}-run`}
          disabled={reserved || !runEnabled}
          aria-pressed={sides.run === 'paint-green'}
          onClick={onRun}
        >
          {runLabel}
        </button>
      )}
      {/* 理由はそのボタンのそば（§3.3.3）。ヘッダには重ねない。 */}
      {stopDenyReason && <span className="reason-bubble" data-testid={`${testId}-stop-reason`}>押せません　{stopDenyReason}</span>}
      {runDenyReason && <span className="reason-bubble" data-testid={`${testId}-run-reason`}>押せません　{runDenyReason}</span>}
    </div>
  )
}
