// shell/effectDispatch.js — /system/effect の振り分け (1b-6, SG-B2).
//
// state_manager が /system/effect に出す effect を 1 箇所で振り分ける純粋関数。
// WebUI の購読は ros/useSystemEffect.js が 1 つだけ持ち、ここは購読しない。
// 副作用は未知の名前の警告 (console.warn, 名前ごとに 1 回だけ) のみ。
import { guideLabel } from '../i18n/guides.js'

// 警告済みの名前 (黙って捨てないための 1 回だけの警告)。
const _warned = new Set()

export function resetEffectWarningsForTest() {
  _warned.clear()
}

function warnOnce(name) {
  const key = String(name ?? '(empty)')
  if (_warned.has(key)) return
  _warned.add(key)
  console.warn(`[effect] 未対応の effect '${key}' を捨てた (/system/effect)`)
}

function parseArgs(argsJson) {
  if (!argsJson) return {}
  try {
    const parsed = JSON.parse(argsJson)
    return (parsed && typeof parsed === 'object' && !Array.isArray(parsed)) ? parsed : {}
  } catch {
    return {}
  }
}

// effect メッセージ -> { kind: 'guide', key } | { kind: 'ignore' }.
// dest が WebUI を含まないものは見ない (例: restart_control_stack は
// connectivity_checker 宛)。dest が "WebUI+jog_gate" のように + 結合の
// ときは WebUI を含むものとして扱う。
export function dispatchEffect(msg) {
  const dest = String(msg?.dest ?? '')
  if (!dest.split('+').includes('WebUI')) return { kind: 'ignore' }
  const name = msg?.name
  switch (name) {
    case 'guide': {
      const key = parseArgs(msg?.args_json).key
      if (typeof key === 'string' && key && guideLabel(key)) return { kind: 'guide', key }
      warnOnce(`guide:${String(key)}`)
      return { kind: 'ignore' }
    }
    // W-1・W-2・W-5・show_resume は /system/state から出す。状態から出す方が、
    // 再接続した端末にも出るので正しい。effect では何もしない。
    case 'open_window':
    case 'close_window':
    case 'show_resume':
    // 教示の保存 (SG-B3)・運用の終了 (SG-B6) の作業で扱う。state_manager の
    // 挙動を変える必要があるので、この作業では触らない。
    case 'ask_save':
    case 'ask_save_if_unsaved':
    case 'enable_main_menu':
    // ジョグ UI の有効化は別パケット。ここでは何もしない。
    case 'disable_jog_ui':
    case 'enable_jog_ui':
    // 校正・点検の誘導。押すと既存の遷移 (ui.enter_mode) を送るだけで画面から
    // 閉じられるが、この作業では扱わない (報告参照)。
    case 'offer_calib':
    case 'offer_opcheck':
      return { kind: 'ignore' }
    default:
      warnOnce(name)
      return { kind: 'ignore' }
  }
}

// W-3 の自動クローズ条件 (Spec-webui.md §4「条件解消で自動クローズ」)。
// 少なくともモードが変わったら閉じる。estop_held_at_boot は物理非常停止が
// 離されたら閉じる。
export function shouldGuideClose(guide, { mode, estopHw }) {
  if (!guide) return true
  if (mode !== guide.modeAtOpen) return true
  if (guide.key === 'estop_held_at_boot' && !estopHw) return true
  return false
}
