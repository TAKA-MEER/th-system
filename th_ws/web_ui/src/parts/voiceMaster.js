// parts/voiceMaster.js — S-50 の音声 ON/OFF（マスタ）。
// Spec-webui.md §3.15: 音声は ON/OFF のみ（音量などは今は持たない）。
//
// fontScale.js と同じ流儀: localStorage が正本（ブラウザに記憶される）、
// voiceQueue.setMasterEnabled が実行時の真実。AppShell が起動時に復元し、
// S-50 が切り替える。レイヤ（安全通知・デモ実況）の個別設定は変えない —
// マスタ OFF は両方を黙らせ、ON に戻したらレイヤ設定どおりに鳴る。
import { voiceQueue } from '../voice/voiceQueue.js'

const KEY = 'th.voiceMaster'

function storage() {
  try {
    return typeof window !== 'undefined' ? window.localStorage : null
  } catch {
    return null
  }
}

export function readVoiceMaster() {
  try {
    const v = storage()?.getItem(KEY) ?? null
    return v === null ? true : v === '1'
  } catch {
    return true
  }
}

export function applyVoiceMaster(on) {
  voiceQueue.setMasterEnabled(!!on)
  try {
    storage()?.setItem(KEY, on ? '1' : '0')
  } catch {
    /* ignore: setting still applied for this session */
  }
}
