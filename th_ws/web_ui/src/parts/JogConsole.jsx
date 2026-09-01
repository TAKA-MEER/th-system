// parts/JogConsole.jsx — the stick console shared by the perpetual drive tab
// and the W-6 floating panel (DetailedDesign-wp3.md WP-UI-03 §8.1: "同じ部品を
// 出す（大きさも同じ）"). It is the mockup `#tplStick`'s `.stickBox`:
//   .slider (a continuous speed %) + VirtualStick + SpeedPreset
//
// It owns the drag-state (held / raw vn,wn) and the speed preset, scales the
// normalized command by the preset (vx = vn * speedPct, wz = wn * speedPct),
// and drives ros/useJogLease.js -- the only place anything is published.
// Because `scaled` is recomputed on every render and useJogLease reads the
// latest cmd from a ref each tick, changing the preset (or slider) while a
// finger is still on the stick takes effect from the next 10 Hz tick.
//
// It never publishes directly; all it knows about the wire is that
// useJogLease exists. This keeps "送出を 1 箇所に閉じる" (§4.2).
import { useState } from 'react'
import { useJogLease } from '../ros/useJogLease.js'
import { useKeyboardJog } from './useKeyboardJog.js'
import VirtualStick from './VirtualStick.jsx'
import SpeedPreset from './SpeedPreset.jsx'
import preset from '../generated/speed_presets.json'
import { SPD_SLIDER_ARIA, KBD_HINT, KBD_HINT_KEYS } from '../i18n/screens.js'

export default function JogConsole({ ros, disabled = false, keyboard = false }) {
  const [speedPct, setSpeedPct] = useState(preset.speed_preset_mid)
  const [held, setHeld] = useState(false)
  const [rawCmd, setRawCmd] = useState({ vn: 0, wn: 0 })

  // WS-4: keyboard is a second proportional source for the same state machine.
  // The stick wins while it is being touched; otherwise the keyboard command
  // drives. publish stays in useJogLease (the only place anything is sent).
  // `keyboard` は 1 画面に 1 つだけ true にする（window の keydown は全 JogConsole に
  // 届くので、常設スティック＋W-6 の両方が反応すると指令が二重になる）。
  const kbd = useKeyboardJog(keyboard && !disabled)
  const effectiveHeld = held || kbd.held
  const effectiveRaw = held ? rawCmd : kbd.cmd

  // Scaled to "the chosen speed as a ratio of the (unknown-to-us) ceiling";
  // obstacle_limiter holds the actual m/s rad/s ceilings (WP-UI-03 §3.3).
  const scaled = { vx: effectiveRaw.vn * speedPct, wz: effectiveRaw.wn * speedPct }

  useJogLease(ros, effectiveHeld && !disabled, scaled)

  const handleChange = (cmd) => {
    setHeld(true)
    setRawCmd({ vn: cmd.vn, wn: cmd.wn })
  }
  const handleRelease = () => {
    setHeld(false)
    setRawCmd({ vn: 0, wn: 0 })
  }

  return (
    <div className="stickBox">
      <div className="slider">
        <input
          type="range"
          min="0"
          max="100"
          step="5"
          value={Math.round(speedPct * 100)}
          aria-label={SPD_SLIDER_ARIA}
          onChange={(e) => setSpeedPct(Number(e.target.value) / 100)}
        />
      </div>
      <VirtualStick disabled={disabled} onChange={handleChange} onRelease={handleRelease} />
      <SpeedPreset value={speedPct} onSelect={setSpeedPct} />
      {keyboard && (
        <div className={`kbdHint${kbd.held ? ' on' : ''}`}>
          <span className="kbdKeys">{KBD_HINT_KEYS}</span>
          <span className="kbdDesc">{KBD_HINT}</span>
        </div>
      )}
    </div>
  )
}
