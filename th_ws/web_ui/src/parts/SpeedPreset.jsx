// parts/SpeedPreset.jsx — the 低速 / 中速 / 高速 preset buttons
// (DetailedDesign-wp3.md WP-UI-03 §4.2). Pure display controller: it only
// reports the chosen preset ratio back to the parent via onSelect; it never
// converts ratios to physical speeds (the ceiling authority is
// obstacle_limiter, not the browser -- §3.3 "ブラウザに上限を持たせない").
//
// The ratios come from generated/speed_presets.json, built by
// scripts/gen_speed_presets.py from th_params/config/registry.yaml, so no
// 0.3 / 0.6 / 1.0 literal lives in JSX (規約 R2). Japanese labels live in
// i18n/screens.js as required by the i18n-only rule (c4).
import presets from '../generated/speed_presets.json'
import { SPD_HIGH, SPD_MID, SPD_LOW } from '../i18n/screens.js'

// Order follows the mockup's #tplStick .spd row (高速 / 中速 / 低速).
const ORDER = [
  { key: 'speed_preset_high', label: SPD_HIGH },
  { key: 'speed_preset_mid', label: SPD_MID },
  { key: 'speed_preset_low', label: SPD_LOW },
]

export default function SpeedPreset({ value, onSelect }) {
  return (
    <div className="spd">
      {ORDER.map(({ key, label }) => (
        <button
          key={key}
          type="button"
          className={`btn sm ${value === presets[key] ? 'on' : ''}`}
          onClick={() => onSelect(presets[key])}
        >
          {label}
        </button>
      ))}
    </div>
  )
}
