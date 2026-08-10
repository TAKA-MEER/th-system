// ============================================================
// announcements.js — 音声アナウンスのマニフェスト
//
// VISION.md §7.7 の文案表をそのままデータ化したもの。
// このファイルは純データで、再生方法もキュー規則も知らない。
//
// 音声ファイルは public/voice/<ID>.mp3 に置く。th_ws/scripts/generate_voice.py が
// このマニフェストを読んで一括生成するので、文案を直したら流し直すだけでよい。
//   ・既定では <ID>.mp3 を再生する
//   ・数値分割合成が要るものは呼び出し側が announce(id, { clips: ['hokaku','3','meter'] })
//     のように実行時の値でクリップ列を渡す (VISION.md §7.5)。渡された clips が
//     1つでも取得失敗すると静的な <ID>.mp3 へフォールバックする (audioPlayer.js)。
//     現状 N4 のみ実装済み。N7/N15/N16/N19/N20/N36/N37 は未着手で、文案に
//     書かれた数値がそのまま焼き込まれている
//   ・分割合成用の語彙クリップ (数字・単位など) は CLIP_WORDS に持つ。
//     announcements 本体とは違いキューの再生対象にはならない、生成専用のデータ
//   ・ファイルが取得できない場合は beep のプレースホルダに落ちる
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
  // generate_voice.py が全件を <ID>.mp3 で出すため既定はそれ。取得に失敗したら
  // audioPlayer が beep に落とすので、未生成の状態でも無音にはならない
  file:      opts.file ?? `${id}.mp3`,
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
  entry('A1', S, 'LIDAR_LOST', '停止。LiDAR 異常', 1.55,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('A2', S, 'ESP32_DISCONNECTED', '停止。駆動系の通信に異常', 1.88,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('A3', S, 'E-Stop 発動 (hw / tablet)', '緊急停止', 0.99,
    { auto: AUTO.WIRED }),
  entry('B1', S, 'PERSON_TRACKER_LOST → IDLE 強制', '対象ロスト。待機へ', 1.71,
    { repeatSec: 15, auto: AUTO.WIRED }),
  entry('B2', S, 'タブレットとの通信断 (全モード)', '通信途絶', 1.02,
    { auto: AUTO.WIRED }),
  entry('B3', S, 'モード遷移拒否', 'その操作は不可', 1.21,
    { auto: AUTO.WIRED }),
  entry('C3', S, '捜索打ち切り (search_timeout_sec)', '発見できず。待機', 1.67,
    { repeatSec: 15, auto: AUTO.WIRED }),
  entry('C4', S, '退避不可 (retreat_blocked)', '逃げ場なし。離れて', 1.58,
    { repeatSec: 5, auto: AUTO.WIRED }),
  entry('C7', S, '進路上障害物で停止 (mapless)', '障害物。停止', 1.65,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('C8', S, '自己位置・スキャン未確立で停止', '自己位置なし。停止', 1.67,
    { repeatSec: 10, auto: AUTO.WIRED }),
  entry('D1', S, 'フォルト復旧 (FAULT CLEARED)', '復旧。再開操作を', 1.83,
    { beep: BEEP.RECOVERY, auto: AUTO.WIRED }),
  entry('D2', S, 'E-Stop 解除 → IDLE', '解除。待機中', 1.4,
    { beep: BEEP.RECOVERY, auto: AUTO.WIRED }),
  entry('D3', S, 'INIT → IDLE (初期化完了)', '待機。位置は捕捉中', 1.73,
    { beep: BEEP.RECOVERY, auto: AUTO.WIRED }),
]

// ── デモ実況レイヤ (38件) ─────────────────────────────────
// N3・N11・N13・N22・N28・N42 は安全通知と内容が重複するため意図的に欠番
// (VISION.md §7.7 末尾)。
const DEMO_ANNOUNCEMENTS = [
  entry('N1',  D, 'ノード起動完了', '起動中', 0.77,
    { note: 'ノード起動の観測手段がない。rosbridge 接続で代用するか要検討' }),
  entry('N2',  D, 'startup grace 経過・センサ健全', 'センサ正常', 1.02,
    { note: 'フォルト無しかつ全ノード生存の判定手段がない' }),
  entry('N4',  D, '/person/status 初回捕捉 (追従中のみ)', '捕捉。前方 3 メートル', 2.06,
    { auto: AUTO.WIRED }),
  entry('N5',  D, '→ FOLLOWING_MAPLESS (任意モードから)', '軌跡追従開始', 1.32,
    { auto: AUTO.WIRED }),
  entry('N6',  D, 'MaplessState.TRACKING', '軌跡を追従中', 1.26,
    { note: '追従開始と同時に TRACKING になるため N5「追従開始。地図なし」と'
          + '重なる。追従中の再突入だけを拾う手段もないため手動発火のみ' }),
  entry('N7',  D, '追従速度が v_max 到達', '秒速 30 センチ', 1.99,
    { note: '速度は publish 済み。ただし文案が v_max の値を直書きしており、'
          + 'パラメータ変更で嘘になる。数値クリップ (clips) の導入が前提' }),
  entry('N8',  D, '加速レート制限が作用', '最大加速度', 1.13,
    { note: 'レート制限が効いたかどうかを MaplessOutput が返していない' }),
  entry('N9',  D, 'STOPPED / person_close', '接近。停止', 1.53,
    { auto: AUTO.WIRED }),
  entry('N10', D, 'STOPPED → TRACKING 復帰', '再開', 0.79,
    { auto: AUTO.WIRED }),
  entry('N12', D, '障害物解消 → 再開', '障害物なし。再開', 1.94,
    { auto: AUTO.WIRED }),
  entry('N14', D, '→ FOLLOWING (任意モードから)', '軌跡追従中', 1.11,
    { auto: AUTO.WIRED,
      note: '地図あり追従には N5 相当の開始アナウンスがないため、これがその役割を兼ねる' }),
  entry('N15', D, 'TRACKING → PREPARE', '接近 3 メートル。退避準備', 2.41,
    { auto: AUTO.WIRED }),
  entry('N16', D, '退避方向スキャン開始', '16 方向スキャン', 1.69,
    { note: 'PREPARE 突入と同じエッジで発火するため N15 と区別できない。'
          + '第一声のみの方針 (VISION.md §7.4) では両方は鳴らせない' }),
  entry('N17', D, '退避方向決定', '右後方に空き', 1.23,
    { note: '退避方向は escape_angle として publish 済み。ただし「右後方」等の'
          + '方位語を音声にするには方位ごとのクリップが必要' }),
  entry('N18', D, '機体の向き合わせ中', '向き合わせ中', 1.02,
    { note: 'PREPARE 内の下位フェーズで、状態としては N15 と同じ区間にある' }),
  entry('N19', D, 'PREPARE → EVADING', '2 メートル。退避開始', 1.66,
    { auto: AUTO.WIRED }),
  entry('N20', D, '退避走行中', '退避中。秒速 15 センチ', 2.19,
    { note: 'EVADING 突入と同じエッジで発火するため N19 と区別できない' }),
  entry('N21', D, '距離回復 → 追従復帰', '距離確保。追従へ', 1.69,
    { auto: AUTO.WIRED }),
  entry('N23', D, 'lost_reason=DETECTION_LOST (追従中のみ)', '対象ロスト', 0.93,
    { auto: AUTO.WIRED }),
  entry('N24', D, 'lost_reason=TARGET_SWITCHED (追従中のみ)', '対象を変更', 1.22,
    { auto: AUTO.WIRED }),
  entry('N25', D, '予測追従開始 (追従中のみ)', '位置を推定中', 1.18,
    { auto: AUTO.WIRED }),
  entry('N26', D, '予測上限超過 → 捜索旋回 (追従中のみ)', '旋回して捜索', 1.34,
    { auto: AUTO.WIRED }),
  entry('N27', D, '捜索中の再捕捉 (追従中のみ)', '再発見。追従再開', 1.66,
    { auto: AUTO.WIRED }),
  entry('N29', D, '呼び寄せ受付', '呼び寄せ開始', 1.02,
    { auto: AUTO.WIRED }),
  entry('N30', D, '呼び寄せ拒否 (対象ロスト)', '対象ロスト。不可', 1.56,
    { auto: AUTO.HEURISTIC,
      note: 'summon_navigator_core.py の日本語メッセージとの部分一致で判定している' }),
  entry('N31', D, '呼び寄せ拒否 (信頼度不足)', '信頼度不足。不可', 1.74,
    { auto: AUTO.HEURISTIC,
      note: 'summon_navigator_core.py の日本語メッセージとの部分一致で判定している' }),
  entry('N32', D, 'Nav2 ゴール受理', '移動開始', 0.87,
    { auto: AUTO.WIRED }),
  entry('N33', D, '呼び寄せ到着', '到着', 0.69,
    { auto: AUTO.WIRED }),
  entry('N34', D, '経路計画失敗・ゴール拒否', '経路なし。中止', 1.48,
    { auto: AUTO.WIRED }),
  entry('N35', D, 'SUMMONING 離脱でキャンセル', '呼び寄せ中止', 1.02,
    { auto: AUTO.WIRED }),
  entry('N36', D, '配電盤への移動開始', '配電盤 1 へ移動', 1.78,
    { auto: AUTO.WIRED }),
  entry('N37', D, 'Nav2 走行中の進捗', '残り 3 メートル', 1.97,
    { note: 'Nav2 の距離フィードバックを WebUI が購読していない' }),
  entry('N38', D, '配電盤前に到着', '到着。作業どうぞ', 1.67,
    { auto: AUTO.WIRED }),
  entry('N39', D, '作業完了通知の受理', '作業完了', 1,
    { note: '/panel_navigator/complete_inspection を WebUI が呼んでいない' }),
  entry('N40', D, '→ MANUAL (任意モードから)', '手動操作', 1.01,
    { auto: AUTO.WIRED }),
  entry('N41', D, '/manual/target_pose 受信', '指定地点へ移動', 1.25,
    { note: 'sendManualGoal を呼ぶ UI が現状ない (地図タップ未実装)' }),
  entry('N43', D, '任意モード → IDLE (通常終了。D2/D3 の場面は除く)', '待機中', 0.82,
    { auto: AUTO.WIRED }),
  entry('N44', D, '手動での対象切替 (/person_tracker/select_target 成功)', '対象を選択', 1.25,
    { auto: AUTO.WIRED }),
  entry('N45', D, '→ FACING (IDLE から・展示用)', '簡易追従', 1.0,
    { auto: AUTO.WIRED,
      note: 'FACING は IDLE からのみ遷移可能なため N5/N14/N40/N43 と異なり遷移元の区別は不要' }),
]

/** VISION.md §7.7 の全 52 件。安全通知 13 + デモ実況 39 */
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

// ── 数値分割合成用の語彙クリップ (VISION.md §7.5) ──────────
// ANNOUNCEMENTS とは別枠。1件が1つの完成した発話ではなく、announce() の
// overrides.clips で組み合わせて使う単語の断片。generate_voice.py が
// ANNOUNCEMENTS と同じ要領で <id>.mp3 を public/voice/ に生成する。
//
// 現状 N4「捕捉。前方 N メートル」のみが実装対象 (hokaku + 数字 + meter)。
// centi (秒速 N センチ) / ban (配電盤 N) は VISION.md §7.5 に単位として
// 明記されているが、対応する N7/N36 のトリガ配線はまだ無いため見送っている。
export const CLIP_WORDS = [
  { id: 'hokaku', text: '捕捉。前方' },
  ...Array.from({ length: 21 }, (_, i) => ({ id: String(i), text: String(i) })),
  { id: 'meter', text: 'メートル' },
]
