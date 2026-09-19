// parts/fontScale.js — WS-9X: user font-size multiplier for the S-50
// display tab. theme.css multiplies its clamp()ed --fs by var(--fs-user);
// this module is the single place that reads/writes the choice and pushes
// it onto #app. AppShell restores it on boot, S50Settings changes it.
//
// localStorage can throw (private windows, thumbnailing) -- every access is
// guarded and falls back to 'normal'.

const KEY = 'th.fontScale'

export const FONT_SCALES = { normal: 1, large: 1.15, xlarge: 1.3 }

export function readFontScale() {
  try {
    const v = window.localStorage.getItem(KEY)
    return v in FONT_SCALES ? v : 'normal'
  } catch {
    return 'normal'
  }
}

export function applyFontScale(name) {
  const scale = FONT_SCALES[name] ?? 1
  const app = document.getElementById('app')
  if (app) app.style.setProperty('--fs-user', String(scale))
  try {
    window.localStorage.setItem(KEY, name in FONT_SCALES ? name : 'normal')
  } catch {
    /* ignore: setting still applied for this session */
  }
}
