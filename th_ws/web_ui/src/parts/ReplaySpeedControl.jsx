// parts/ReplaySpeedControl.jsx — S-14 の「再生速度」低速/中速/高速 ボタン（WS-9T）。
//
// SpeedPreset.jsx と同じく純表示コントローラ: 選ばれた比率を onSelect で親へ返す
// だけで、publish はしない（送出は ros/useReplaySpeedPublisher.js の 1 箇所）。
//
// ここが返すのは「比率」(0..1) であって m/s ではない。実速度への換算は
// replay_runner が replay_cruise_min_mps 〜 cruise_speed_mps の間の補間として持つ
// (route_replay_core.scale_replay_params)。ブラウザに実速度上限を置かない方針は
// jog の SpeedPreset.jsx と同じ（DetailedDesign-wp3.md WP-UI-03 §3.3）。
import { SPD_HIGH, SPD_MID, SPD_LOW } from '../i18n/screens.js'
import { REPLAY_SPEED_RATIOS } from './replaySpeed.js'

const ORDER = [
  { ratio: REPLAY_SPEED_RATIOS.high, label: SPD_HIGH },
  { ratio: REPLAY_SPEED_RATIOS.mid, label: SPD_MID },
  { ratio: REPLAY_SPEED_RATIOS.low, label: SPD_LOW },
]

export default function ReplaySpeedControl({ value, onSelect, disabled = false }) {
  return (
    <div className="spd" data-testid="s14-replay-speed">
      {ORDER.map(({ ratio, label }) => (
        <button
          key={label}
          type="button"
          className={`btn sm ${value === ratio ? 'on' : ''}`}
          disabled={disabled}
          onClick={() => onSelect(ratio)}
        >
          {label}
        </button>
      ))}
    </div>
  )
}
