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

// ROS の時刻 (header.stamp / SystemState.since) を ms にする。rosbridge は
// secs/nsecs、rcl の JSON は sec/nanosec で来るので両方読む。0・欠落は
// null (比較不能。後述のフォールバック)。
export function stampToMs(stamp) {
  if (!stamp || typeof stamp !== 'object') return null
  const sec = Number(stamp.sec ?? stamp.secs ?? NaN)
  const nsec = Number(stamp.nanosec ?? stamp.nsecs ?? 0)
  if (!Number.isFinite(sec) || sec <= 0) return null
  return sec * 1000 + Math.floor(nsec / 1e6)
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
      if (typeof key === 'string' && key && guideLabel(key)) {
        return { kind: 'guide', key, stampMs: stampToMs(msg?.header?.stamp) }
      }
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
//
// state_manager は 1 回の遷移で effect を先に publish し、そのあと
// /system/state を publish する (画面には guide → 新しい状態の順で届く)。
// そのため単なる「モードが変わった」では、同じ遷移でモードが変わる guide
// (home_arrived 等) が出た瞬間に閉じてしまう。guide より後に入ったモード
// (since が guide の stamp より後) に変わったときだけ閉じる。同時刻
// (同じ遷移の effect→state は通常同一 ms) は閉じない。
// estop_held_at_boot は「押されているのを一度見てから離れた」ときだけ閉じる
// (guide 到着時に estop_hw=false のまま＝状態が後から届く、では閉じない)。
// stamp／since のどちらかが比較不能 (0・欠落) のときはモードでは閉じない
// (閉じるボタンは常にある。消える方向の誤動作より残る方向を選ぶ)。
export function shouldGuideClose(guide, { mode, sinceMs, estopHw }) {
  if (!guide) return true
  if (mode !== guide.modeAtOpen) {
    if (guide.stampMs == null || sinceMs == null) return false
    if (sinceMs > guide.stampMs) return true
  }
  if (guide.key === 'estop_held_at_boot' && guide.seenPressed && !estopHw) return true
  return false
}
