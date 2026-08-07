// ============================================================
// JudgementPanel.jsx — 観客向け「ロボットの判断」(VISION.md §6.3)
//
// 左が「何を見ているか」なら、こちらは「何を決めたか」。
// 文字列の写像は th_system_msgs の各 .msg のコメントが正であり、
// あちらを変えたらここも直すこと。
// ============================================================
import { modeColorOf } from '../robotMode.js'

// FollowStatus.msg の state / reason
const FOLLOW_STATE = {
  INACTIVE: '待機',
  TRACKING: '追従中',
  STOPPED:  '停止中',
  PREPARE:  '退避準備',
  EVADING:  '退避中',
}
const FOLLOW_REASON = {
  tracking:        '正常',
  person_lost:     '試験員を見失った',
  person_close:    '近づきすぎたので停止',
  obstacle_ahead:  '進路上に障害物',
  no_scan:         'LiDAR 未受信',
  no_pose:         '自己位置が未確立',
  preparing:       '向き合わせ中',
  evading:         '退避走行中',
  retreat_blocked: '逃げ場がない',
}
// SearchStatus.msg の phase
const SEARCH_PHASE = {
  NONE:       { label: '捕捉中',       tone: 'ok' },
  PREDICTING: { label: '位置を予測中', tone: 'warn' },
  SEARCHING:  { label: '捜索旋回中',   tone: 'warn' },
  GAVE_UP:    { label: '発見できず',   tone: 'ng' },
}
// PersonStatus.msg の lost_reason
const LOST_REASON = {
  DETECTION_LOST: '検出が途切れた',
  LOW_CONFIDENCE: '信頼度が低い',
  TARGET_SWITCHED: '別人に乗り移った疑い',
}

const fmt = (v, digits = 2, unit = '') =>
  Number.isFinite(v) ? `${v.toFixed(digits)}${unit}` : '—'

function Row({ label, value, tone }) {
  return (
    <div className="aud-row">
      <span className="aud-row-label">{label}</span>
      <span className={`aud-row-value ${tone ? `tone-${tone}` : ''}`}>{value}</span>
    </div>
  )
}

export default function JudgementPanel({
  connected, modeName, fault, personStatus, candidates,
  followStatus, searchStatus, captions, conditions,
}) {
  const isTracked = personStatus && !personStatus.is_lost
  const phase = SEARCH_PHASE[searchStatus?.phase] ?? SEARCH_PHASE.NONE

  // 距離は FollowStatus が持っているが、追従モード外では流れない。
  // その場合は /person/status の相対座標から出す
  const distFromStatus = personStatus
    ? Math.hypot(personStatus.position.x, personStatus.position.y)
    : NaN
  const dist = followStatus && followStatus.person_distance >= 0
    ? followStatus.person_distance
    : distFromStatus

  return (
    <aside className="aud-panel">

      <div className="aud-mode" style={{ borderColor: modeColorOf(modeName) }}>
        <span className="aud-mode-caption">モード</span>
        <span className="aud-mode-name" style={{ color: modeColorOf(modeName) }}>
          {connected ? (modeName ?? '—') : '未接続'}
        </span>
      </div>

      {fault?.active && (
        <div className="aud-alert">⚠ {fault.fault_type} — {fault.description}</div>
      )}

      {/* ── 人の認識 (この画面のもう一方の主役) ── */}
      <section className="aud-section">
        <h2>人の認識</h2>
        <Row
          label="追跡対象"
          value={isTracked ? '捕捉中' : (personStatus ? '見失い中' : '—')}
          tone={isTracked ? 'ok' : 'ng'}
        />
        <Row label="距離"       value={fmt(dist, 2, ' m')} />
        <Row label="信頼度"     value={fmt(personStatus?.confidence, 2)} />
        <Row label="検出候補"   value={`${candidates?.length ?? 0} 人`} />
        <Row label="捜索段階"   value={phase.label} tone={phase.tone} />
        {!isTracked && personStatus?.lost_reason && (
          <Row
            label="ロスト理由"
            value={LOST_REASON[personStatus.lost_reason] ?? personStatus.lost_reason}
            tone="ng"
          />
        )}
        {searchStatus?.lost_elapsed_sec > 0 && (
          <Row label="ロスト経過" value={fmt(searchStatus.lost_elapsed_sec, 1, ' 秒')} />
        )}
      </section>

      {/* ── 追従の判断 ── */}
      <section className="aud-section">
        <h2>追従の判断</h2>
        <Row
          label="状態"
          value={FOLLOW_STATE[followStatus?.state] ?? '—'}
        />
        <Row
          label="理由"
          value={FOLLOW_REASON[followStatus?.reason] ?? (followStatus?.reason || '—')}
        />
        <Row label="指令速度" value={fmt(followStatus?.linear_x, 2, ' m/s')} />
      </section>

      {/* ── 継続中の警告 (字幕は流れて消えるので、続いている状態は常時出す) ── */}
      {conditions.length > 0 && (
        <div className="aud-conditions">
          {conditions.map((c) => (
            <span key={c.id} className="aud-condition">{c.text}</span>
          ))}
        </div>
      )}

      {/* ── 実況ログ ── */}
      <section className="aud-section aud-log-section">
        <h2>実況</h2>
        <ul className="aud-log">
          {captions.length === 0 && <li className="aud-log-empty">—</li>}
          {captions.map((c) => (
            <li key={c.key} className={`aud-log-item layer-${c.layer}`}>
              <time>{c.at.toLocaleTimeString('ja-JP', { hour12: false })}</time>
              <span>{c.text}</span>
            </li>
          ))}
        </ul>
      </section>

    </aside>
  )
}
