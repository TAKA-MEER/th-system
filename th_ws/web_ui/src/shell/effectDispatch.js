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
      return { kind: 'ignore' }
    // 教示の保存確認 (1b-7, SG-B3)。機体側が ask_save（記録途切れ）／
    // ask_save_if_unsaved（「終了」）を出し、WebUI が W-4 を出して
    //「はい」で保存・「いいえ」で破棄する。開く条件は shouldOpenSaveAsk。
    case 'ask_save':
    case 'ask_save_if_unsaved': {
      const { route_id } = parseArgs(msg?.args_json)
      return {
        kind: name,
        routeId: typeof route_id === 'string' && route_id ? route_id : null,
        stampMs: stampToMs(msg?.header?.stamp),
      }
    }
    case 'enable_main_menu':
    // 1b-6: 画面だけで閉じている。疎通確認が済むまでメインメニューを押せない
    // ことは /system/state の INIT で既に代わりになっている（S-00 の進行
    // ボタンは IDLE 到達まで出ず、S-01 の操作は mode INIT/stale で disabledAll）。
    // effect は読まず ignore のままにする。
    // ジョグ UI の有効化は別パケット。ここでは何もしない。
    case 'disable_jog_ui':
    case 'enable_jog_ui':
    // 校正・点検の誘導。1b-6 で調べた結果、Spec-checks.md に「どの画面に何を
    // 出し、押すと何を送るか」の定義が無いため作らない（報告参照）。
    // なお誘導の導線自体は別に存在する（S-30 の「校正へ」・S-40 の「始業点検へ
    // 戻る」）。spec に定義が足されたら、押すと既存の遷移（ui.enter_mode）を
    // 送るだけの形で実装する。
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
// (since が guide の stamp より GUIDE_SAME_TRANSITION_GRACE_MS を超えて後)
// に変わったときだけ閉じる。
// 猶予の理由: 機体側は effect を publish した後に _publish_state() で since
// を取る (effect ごとの logger.info を挟む) ため、同じ遷移でも since は
// stamp より数百 µs〜数 ms 後になり、ms 境界をまたぐと出た瞬間に閉じる。
// 次の遷移は人の操作・秒単位の時間経過で起きるので 1 秒あれば区別できる。
// estop_held_at_boot は「押されているのを一度見てから離れた」ときだけ閉じる
// (guide 到着時に estop_hw=false のまま＝状態が後から届く、では閉じない)。
// stamp／since のどちらかが比較不能 (0・欠落) のときはモードでは閉じない
// (閉じるボタンは常にある。消える方向の誤動作より残る方向を選ぶ)。
export const GUIDE_SAME_TRANSITION_GRACE_MS = 1000

export function shouldGuideClose(guide, { mode, sinceMs, estopHw }) {
  if (!guide) return true
  if (mode !== guide.modeAtOpen) {
    if (guide.stampMs == null || sinceMs == null) return false
    if (sinceMs > guide.stampMs + GUIDE_SAME_TRANSITION_GRACE_MS) return true
  }
  if (guide.key === 'estop_held_at_boot' && guide.seenPressed && !estopHw) return true
  return false
}

// W-4（保存確認）を開くかの純粋判定 (1b-7, SG-B3)。
//
// decided: dispatchEffect の戻り値（kind が ask_save / ask_save_if_unsaved のもの）。
// status: /route/status の最新値（route_recorder が教示系モードのときだけ出す）。
//   replay_runner も同じトピックに出すが、経路読み込み中は current.id が入る
//   （記録側は載せない）ので、current.id があるものは記録の未保存とみなさない。
//
// - ask_save（記録の連続性切れ → IDLE）: 未保存の記録があるとき開く。
//   status 未到来（途切れが記録開始の直後で status がまだ無い）は開かない
//   （空の記録を保存してゴミの経路を作らない。recorder は開いたままなので
//   次の教示が上書きする）。
// - ask_save_if_unsaved（「終了」→ IDLE）: どのモードの終了でも飛ぶので、
//   未保存の教示記録があるときだけ開く（無ければ開かないのが「あれば問う」）。
export function shouldOpenSaveAsk(decided, status) {
  if (!decided || (decided.kind !== 'ask_save' && decided.kind !== 'ask_save_if_unsaved')) {
    return false
  }
  return isUnsavedRecording(status)
}

export function isUnsavedRecording(status) {
  if (!status || status.saved === true) return false
  if (status.current?.id) return false
  return (status.points ?? 0) > 0
}
