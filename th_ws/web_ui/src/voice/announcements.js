// ============================================================
// announcements.js — 音声アナウンスのマニフェスト
//
// VISION.md §7.7 の文案表をそのままデータ化したもの。
// このファイルは純データで、再生方法もキュー規則も知らない。
//
// WAV 導入時 (Tier 3) にコードを触らずに済ませるための約束:
//   ・単一クリップなら file に 'safety/a1.wav' を入れる
//   ・数値分割合成が要るものは clips に ['hokaku','3','meter'] を入れる
//     (VISION.md §7.5。N4/N7/N15/N16/N19/N20/N36/N37 が該当)
//   ・どちらも null の間は beep のプレースホルダが鳴る
// ============================================================

/** レイヤ。トグルの単位であり、同時に優先度でもある (VISION.md §7.3/§7.6) */
export const LAYER = {
  SAFETY: 'safety',
  DEMO:   'demo',
}

/** 自動トリガの実装状況。開発パネルの表示分けに使う */
export const AUTO = {
  /** 既存の購読・応答から確実に判定できる */
  WIRED:     'auto',
  /** 判定できるが文字列一致など脆い手段に依存している */
  HEURISTIC: 'heuristic',
  /** 追従系ノードの状態 publish (Tier 2) 待ち。手動発火のみ可能 */
  TIER2:     'tier2',
}

// ── プレースホルダのビープ ────────────────────────────────
// WAV が用意できるまでの仮の音。レイヤが聴き分けられれば十分なので
// 安全通知 / 復帰 / デモ実況 の 3 種類だけ用意する。
export const BEEP = {
  SAFETY:   { wave: 'square', gain: 0.18, freq: 880, ms: 200, repeats: 2, gapMs: 80 },
  RECOVERY: { wave: 'square', gain: 0.15, freq: 660, ms: 160, repeats: 2, gapMs: 60 },
  DEMO:     { wave: 'sine',   gain: 0.12, freq: 440, ms: 120, repeats: 1, gapMs: 0  },
}

/** 再発話の既定打ち切り回数 (VISION.md §7.6) */
export const DEFAULT_REPEAT_MAX = 5

// entry を書くときの定型を減らすヘルパ。
// 引数の順序は VISION.md §7.7 の表の列順に合わせてある。
const entry = (id, layer, trigger, text, approxSec, opts = {}) => ({
  id,
  layer,
  trigger,
  text,
  approxSec,
  beep:      opts.beep ?? (layer === LAYER.SAFETY ? BEEP.SAFETY : BEEP.DEMO),
  file:      opts.file ?? null,
  clips:     opts.clips ?? null,
  repeatSec: opts.repeatSec ?? null,
  repeatMax: opts.repeatSec ? (opts.repeatMax ?? DEFAULT_REPEAT_MAX) : null,
  auto:      opts.auto ?? AUTO.TIER2,
  note:      opts.note ?? null,
})

const S = LAYER.SAFETY
const D = LAYER.DEMO

// ── 安全通知レイヤ (13件) ─────────────────────────────────
const SAFETY_ANNOUNCEMENTS = [
  entry('A1', S, 'LIDAR_LOST', '停止。LiDAR 異常', 1.0,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('A2', S, 'ESP32_DISCONNECTED', '停止。モーター通信断', 1.2,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('A3', S, 'E-Stop 発動 (hw / tablet)', '緊急停止', 0.7,
    { auto: AUTO.WIRED }),
  entry('B1', S, 'PERSON_TRACKER_LOST → IDLE 強制', '見失い。待機へ', 1.0,
    { repeatSec: 15, auto: AUTO.WIRED }),
  entry('B2', S, 'タブレットとの通信断 (全モード)', '通信断', 0.8,
    { auto: AUTO.WIRED }),
  entry('B3', S, 'モード遷移拒否', 'その操作は不可', 1.0,
    { auto: AUTO.WIRED }),
  entry('C3', S, '捜索打ち切り (search_timeout_sec)', '発見できず。待機', 1.4,
    { repeatSec: 15, auto: AUTO.WIRED }),
  entry('C4', S, '退避不可 (retreat_blocked)', '逃げ場なし。離れて', 1.3,
    { repeatSec: 5, auto: AUTO.WIRED }),
  entry('C7', S, '進路上障害物で停止 (mapless)', '障害物。停止', 1.0,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('C8', S, '自己位置・スキャン未確立で停止', '自己位置なし。停止', 1.3,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('D1', S, 'フォルト復旧 (FAULT CLEARED)', '復旧。再開操作を', 1.2,
    { beep: BEEP.RECOVERY, auto: AUTO.WIRED }),
  entry('D2', S, 'E-Stop 解除 → IDLE', '解除。待機中', 1.0,
    { beep: BEEP.RECOVERY, auto: AUTO.WIRED }),
  entry('D3', S, 'INIT → IDLE (初期化完了)', '待機。位置は捕捉中', 1.3,
    { beep: BEEP.RECOVERY, auto: AUTO.WIRED }),
]

// ── デモ実況レイヤ (36件) ─────────────────────────────────
// N3・N11・N13・N22・N28・N42 は安全通知と内容が重複するため意図的に欠番
// (VISION.md §7.7 末尾)。
const DEMO_ANNOUNCEMENTS = [
  entry('N1',  D, 'ノード起動完了', '起動中', 0.5,
    { note: 'ノード起動の観測手段がない。rosbridge 接続で代用するか要検討' }),
  entry('N2',  D, 'startup grace 経過・センサ健全', 'センサ正常', 0.8,
    { note: 'フォルト無しかつ全ノード生存の判定手段がない' }),
  entry('N4',  D, '/person/status 初回捕捉', '捕捉。前方 3 メートル', 1.3,
    { auto: AUTO.WIRED }),
  entry('N5',  D, 'IDLE → FOLLOWING_MAPLESS', '追従開始。地図なし', 1.3,
    { auto: AUTO.WIRED }),
  entry('N6',  D, 'MaplessState.TRACKING', '軌跡を追従中', 1.0,
    { auto: AUTO.WIRED }),
  entry('N7',  D, '追従速度が v_max 到達', '秒速 30 センチ', 1.1,
    { note: '速度は publish 済み。ただし文案が v_max の値を直書きしており、'
          + 'パラメータ変更で嘘になる。数値クリップ (clips) の導入が前提' }),
  entry('N8',  D, '加速レート制限が作用', 'ゆっくり加速', 0.9,
    { note: 'レート制限が効いたかどうかを MaplessOutput が返していない' }),
  entry('N9',  D, 'STOPPED / person_close', '接近。停止', 0.8,
    { auto: AUTO.WIRED }),
  entry('N10', D, 'STOPPED → TRACKING 復帰', '離れた。再開', 0.9,
    { auto: AUTO.WIRED }),
  entry('N12', D, '障害物解消 → 再開', '回避完了。再開', 1.2,
    { auto: AUTO.WIRED }),
  entry('N14', D, 'FollowState.TRACKING', '軌跡追従中', 0.9,
    { auto: AUTO.WIRED }),
  entry('N15', D, 'TRACKING → PREPARE', '接近 3 メートル。退避準備', 1.7,
    { auto: AUTO.WIRED }),
  entry('N16', D, '退避方向スキャン開始', '16 方向スキャン', 1.3,
    { note: 'PREPARE 突入と同じエッジで発火するため N15 と区別できない。'
          + '第一声のみの方針 (VISION.md §7.4) では両方は鳴らせない' }),
  entry('N17', D, '退避方向決定', '右後方に空き', 1.0,
    { note: '退避方向は escape_angle として publish 済み。ただし「右後方」等の'
          + '方位語を音声にするには方位ごとのクリップが必要' }),
  entry('N18', D, '機体の向き合わせ中', '向き合わせ中', 1.0,
    { note: 'PREPARE 内の下位フェーズで、状態としては N15 と同じ区間にある' }),
  entry('N19', D, 'PREPARE → EVADING', '2 メートル。退避開始', 1.4,
    { auto: AUTO.WIRED }),
  entry('N20', D, '退避走行中', '退避中。秒速 15 センチ', 1.6,
    { note: 'EVADING 突入と同じエッジで発火するため N19 と区別できない' }),
  entry('N21', D, '距離回復 → 追従復帰', '距離確保。追従へ', 1.3,
    { auto: AUTO.WIRED }),
  entry('N23', D, 'lost_reason=DETECTION_LOST', 'ロスト', 0.5,
    { auto: AUTO.WIRED }),
  entry('N24', D, 'lost_reason=TARGET_SWITCHED', '別人の恐れ。中止', 1.3,
    { auto: AUTO.WIRED }),
  entry('N25', D, '予測追従開始', '位置を推定中', 1.0,
    { auto: AUTO.WIRED }),
  entry('N26', D, '予測上限超過 → 捜索旋回', '旋回して捜索', 1.1,
    { auto: AUTO.WIRED }),
  entry('N27', D, '捜索中の再捕捉', '再発見。追従再開', 1.4,
    { auto: AUTO.WIRED }),
  entry('N29', D, '呼び寄せ受付', '呼び寄せ開始', 0.9,
    { auto: AUTO.WIRED }),
  entry('N30', D, '呼び寄せ拒否 (対象ロスト)', '対象ロスト。不可', 1.3,
    { auto: AUTO.HEURISTIC,
      note: 'summon_navigator_core.py の日本語メッセージとの部分一致で判定している' }),
  entry('N31', D, '呼び寄せ拒否 (信頼度不足)', '信頼度不足。不可', 1.4,
    { auto: AUTO.HEURISTIC,
      note: 'summon_navigator_core.py の日本語メッセージとの部分一致で判定している' }),
  entry('N32', D, 'Nav2 ゴール受理', '移動開始', 0.8,
    { auto: AUTO.WIRED }),
  entry('N33', D, '呼び寄せ到着', '到着', 0.4,
    { auto: AUTO.WIRED }),
  entry('N34', D, '経路計画失敗・ゴール拒否', '経路なし。中止', 1.2,
    { auto: AUTO.WIRED }),
  entry('N35', D, 'SUMMONING 離脱でキャンセル', '呼び寄せ中止', 1.0,
    { auto: AUTO.WIRED }),
  entry('N36', D, '配電盤への移動開始', '配電盤 1 へ移動', 1.4,
    { auto: AUTO.WIRED }),
  entry('N37', D, 'Nav2 走行中の進捗', '残り 3 メートル', 1.1,
    { note: 'Nav2 の距離フィードバックを WebUI が購読していない' }),
  entry('N38', D, '配電盤前に到着', '到着。作業どうぞ', 1.3,
    { auto: AUTO.WIRED }),
  entry('N39', D, '作業完了通知の受理', '作業完了', 0.8,
    { note: '/panel_navigator/complete_inspection を WebUI が呼んでいない' }),
  entry('N40', D, 'IDLE → MANUAL', '手動モード', 0.9,
    { auto: AUTO.WIRED }),
  entry('N41', D, '/manual/target_pose 受信', '指定地点へ移動', 1.4,
    { note: 'sendManualGoal を呼ぶ UI が現状ない (地図タップ未実装)' }),
]

/** VISION.md §7.7 の全 49 件。安全通知 13 + デモ実況 36 */
export const ANNOUNCEMENTS = [...SAFETY_ANNOUNCEMENTS, ...DEMO_ANNOUNCEMENTS]

const BY_ID = new Map(ANNOUNCEMENTS.map((a) => [a.id, a]))

/** id からエントリを引く。未知の id は undefined */
export function getAnnouncement(id) {
  return BY_ID.get(id)
}

/** レイヤ別のエントリ一覧 (マニフェスト順) */
export const ANNOUNCEMENTS_BY_LAYER = {
  [LAYER.SAFETY]: SAFETY_ANNOUNCEMENTS,
  [LAYER.DEMO]:   DEMO_ANNOUNCEMENTS,
}
