// screens/S00Connect.jsx — S-00, the connection-check screen (SCREEN_NAMES.S00
// in i18n/screens.js; DetailedDesign-wp1.md WP-UI-02).
//
// Spec-webui §3.1 (型: 見る・変える): 主表示は機器のつながりの図
// （タブレット → PC → Wi-Fi AP → ラズパイ → LiDAR・ESP32、AP の先にカメラ）。
// 各機器に結果の印と実データ（受信の間隔・応答の時間）。つながらない機器から
// 先の線を灰の点線にする。必須と任意は印の大きさで分け、任意はだめでも
// 図の線を灰にしない。図の下に対処を 1 行と確かめた時刻を出す。
// 右列は上＝「メインメニューへ」、下＝自動判定の 4 つの印、最下段に
// 「開発モードの設定」。ヘッダの札は「確認中」／「再起動中　n 回目」。
// 「開き直したときに INIT 以外ならその画面を開く」振る舞い（screenRouting）
// は変えない。
import { useSystemState } from '../ros/useSystemState.js'
import { useAutoCheckStatus } from '../ros/useAutoCheckStatus.js'
import { useLinkStatus } from '../ros/useLinkStatus.js'
import { linkRowView, linkDownKeys } from '../ros/linkStatusState.js'
import HelpTip from '../parts/HelpTip.jsx'
import ReasonButton from '../parts/ReasonButton.jsx'
import {
  S00_CONNECT_TITLE, S00_DIAG_PC, S00_DIAG_AP, S00_DIAG_TABLET, S00_DIAG_LIDAR, S00_DIAG_ESP32, S00_DIAG_CAM,
  S00_DIAG_CHECKED_AT,
  S00_DIAG_FIX_LIDAR, S00_DIAG_FIX_ESP32, S00_DIAG_FIX_PC, S00_DIAG_FIX_AP, S00_DIAG_FIX_CAM,
  S00_LINK_TEXT, S00_ADVANCE, S00_OPEN_DEV,
  S00_AUTO_TITLE, S00_AUTO_CHECKING, S00_AUTO_OK, S00_AUTO_WARN, S00_AUTO_NG,
  S00_AUTO_SUPPRESSED, S00_AUTO_NOT_YET,
  S00_AUTO_ITEM_LABELS, S00_AUTO_ITEM_ORDER,
  s00AutoResultLabel,
} from '../i18n/screens.js'
import { DISCONNECTED_LABEL } from '../i18n/states.js'

// 自動判定の総合 → 印の形と色クラス。
function autoOverallMark(auto) {
  if (!auto) return { cls: 'none', sym: '—', label: S00_AUTO_NOT_YET }
  if (auto.suppressed) return { cls: 'none', sym: '—', label: S00_AUTO_SUPPRESSED }
  switch (auto.overall) {
    case 'OK': return { cls: 'ok', sym: '✓', label: S00_AUTO_OK }
    case 'WARN': return { cls: 'warn', sym: '!', label: S00_AUTO_WARN }
    case 'NG': return { cls: 'ng', sym: '×', label: S00_AUTO_NG }
    default: return { cls: 'warn', sym: '!', label: S00_AUTO_CHECKING }
  }
}

function autoItemMark(result) {
  switch (result) {
    case 'OK': return { cls: 'ok', sym: '✓' }
    case 'WARN': return { cls: 'warn', sym: '!' }
    case 'NG': return { cls: 'ng', sym: '×' }
    default: return { cls: 'none', sym: '—' }
  }
}

// 図の配置（テストの矩形 920x470。モックアップと同じ位置）。
const N = {
  pc: { x: 180, y: 250, name: S00_DIAG_PC, req: true },
  ap: { x: 540, y: 250, name: S00_DIAG_AP, req: true },
  tab: { x: 540, y: 70, name: S00_DIAG_TABLET, req: true },
  lidar: { x: 910, y: 110, name: S00_DIAG_LIDAR, req: true },
  esp: { x: 910, y: 250, name: S00_DIAG_ESP32, req: true },
  cam: { x: 910, y: 400, name: S00_DIAG_CAM, req: false },
}
const EDGES = [['pc', 'ap'], ['ap', 'tab'], ['ap', 'lidar'], ['ap', 'esp'], ['ap', 'cam']]

// onOpenSettings（2026-09-23）: 機器が揃わず INIT を抜けられないときに、開発モードの
// 設定（S-50 の開発モードの区分）へ入って項目を外し、IDLE へ進むための導線。
export default function S00Connect({ onAdvance, onOpenSettings }) {
  const { state, stale, ros } = useSystemState()
  const { auto } = useAutoCheckStatus(ros)
  const { link } = useLinkStatus(ros)
  const mode = state?.mode ?? null
  const ready = !stale && mode != null && mode !== 'INIT'
  const overall = autoOverallMark(auto)
  // 1b-6 SG-C4: 項目別の状態が届いているのに必須 3 者が揃わないときは
  // 「必須機器が繋がっていません」。
  const linkNotReady = linkDownKeys(link).length > 0
  // 1b-6 SG-B7: 再起動中（start.sh が数えた起動回数が 2 以上）の表示。
  const restartAttempt = link?.restart?.attempt
  const restarting = !stale && !ready && restartAttempt != null && restartAttempt >= 2

  // 図の各機器の状態。実データだけ出す（届かない値を作らない）。
  // PC は nodes、LiDAR は lidar、ESP32 は feedback・loopback 両方。
  // AP・タブレットはこの画面（タブレット）が rosbridge に繋がっているか。
  // カメラは機体側に状態が届かないので「—」を出す（値は作らない）。
  const pcView = linkRowView(link, 'nodes', S00_LINK_TEXT)
  const lidarView = linkRowView(link, 'lidar', S00_LINK_TEXT)
  const espFbView = linkRowView(link, 'esp32_feedback', S00_LINK_TEXT)
  const espLbView = linkRowView(link, 'esp32_loopback', S00_LINK_TEXT)
  const espOk = espFbView.tone === 'tone-ok' && espLbView.tone === 'tone-ok'
  const espKnown = espFbView.tone !== 'tone-warn'
  const nodes = {
    pc: stale ? { m: 'ng', tone: 'tone-ng', detail: DISCONNECTED_LABEL }
      : { m: pcView.tone === 'tone-ok' ? 'ok' : pcView.tone === 'tone-ng' ? 'ng' : 'none', tone: pcView.tone, detail: pcView.detail },
    ap: stale ? { m: 'ng', tone: 'tone-ng', detail: '' } : { m: 'ok', tone: 'tone-ok', detail: '' },
    tab: stale ? { m: 'ng', tone: 'tone-ng', detail: DISCONNECTED_LABEL } : { m: 'ok', tone: 'tone-ok', detail: '' },
    lidar: stale ? { m: 'ng', tone: 'tone-ng', detail: DISCONNECTED_LABEL }
      : { m: lidarView.tone === 'tone-ok' ? 'ok' : lidarView.tone === 'tone-ng' ? 'ng' : 'none', tone: lidarView.tone, detail: lidarView.detail },
    esp: stale ? { m: 'ng', tone: 'tone-ng', detail: DISCONNECTED_LABEL }
      : espOk ? { m: 'ok', tone: 'tone-ok', detail: espFbView.detail }
      : espKnown ? { m: 'ng', tone: 'tone-ng', detail: espFbView.detail || espLbView.detail }
      : { m: 'none', tone: 'tone-warn', detail: '' },
    cam: { m: 'none', tone: '', detail: '' },
  }

  // 図の節点の実データ（リンク行の detail）。カメラは値が無いので出さない。
  const DATA = {
    pc: nodes.pc.detail, ap: '', tab: '', lidar: nodes.lidar.detail, esp: nodes.esp.detail, cam: '',
  }

  // つながっていない機器から先の線を灰の点線に。必須でない（カメラ）の
  // 無到達では線を灰にしない（D101）。
  function edgeDead(to) {
    if (to === 'cam') return false
    return nodes[to].m !== 'ok'
  }
  const edge = (a, b) => {
    const [p, q] = [N[a], N[b]]
    const dead = edgeDead(b)
    return (
      <line
        key={`${a}-${b}`}
        x1={p.x} y1={p.y} x2={q.x} y2={q.y}
        stroke={dead ? 'var(--bd-hi)' : 'var(--ok)'}
        strokeWidth={dead ? 3 : 5}
        strokeDasharray={dead ? '10 10' : undefined}
        data-testid={`s00-edge-${b}`}
        data-dead={dead ? 'true' : 'false'}
      />
    )
  }
  const node = (k) => {
    const n = N[k]
    const m = nodes[k].m
    // §1.2: 名前は 15px 以上。主表示いっぱいを前提に字を大きく（SVG の縮小を踏まえる）。
    const w = k === 'pc' ? 300 : k === 'ap' ? 400 : n.req ? 340 : 250
    const h = n.req ? 84 : 68
    const sym = { ok: '✓', ng: '×', none: '—' }[m]
    const fill = { ok: 'var(--ok)', ng: 'var(--jis-y,#ffd100)', none: 'var(--ctrl-hi)' }[m]
    return (
      <g key={k} data-testid={`s00-node-${k}`} data-m={m} data-tone={nodes[k].tone}>
        <rect
          x={n.x - w / 2} y={n.y - h / 2} width={w} height={h} rx="8"
          style={{ fill: 'var(--panel)', stroke: m === 'ng' ? 'var(--warn)' : 'var(--bd-hi)' }}
          strokeWidth={m === 'ng' ? 4 : 2}
        />
        {m === 'ng' ? (
          <rect x={n.x - w / 2 + 10} y={n.y - 15} width="30" height="30" rx="5" style={{ fill }} />
        ) : (
          <circle cx={n.x - w / 2 + 26} cy={n.y} r={n.req ? 16 : 12} style={{ fill }} />
        )}
        <text
          x={n.x - w / 2 + 26 - (m === 'ng' ? 1 : 0)} y={n.y + 8} textAnchor="middle"
          fontSize={n.req ? 34 : 30} fontWeight="900"
          style={{ fill: m === 'ok' ? '#fff' : '#161b22' }}
        >
          {sym}
        </text>
        <text
          x={n.x - w / 2 + 56} y={n.y + (DATA[k] ? -1 : 8)}
          fontSize={n.req ? 32 : 28} fontWeight={n.req ? 800 : 600}
          style={{ fill: 'var(--tx)' }}
        >
          {n.name}
        </text>
        {DATA[k] ? (
          <text x={n.x - w / 2 + 56} y={n.y + 24} fontSize="26" style={{ fill: 'var(--mut)' }}>
            {DATA[k]}
          </text>
        ) : null}
      </g>
    )
  }
  const svg = (
    <svg viewBox="0 30 1080 420" preserveAspectRatio="xMidYMid meet" data-testid="s00-diag">
      {EDGES.map(([a, b]) => edge(a, b))}
      {Object.keys(N).map(node)}
    </svg>
  )

  // 対処の 1 行（だめな機器だけ）。
  let fix = null
  if (!stale) {
    if (nodes.lidar.m === 'ng') fix = S00_DIAG_FIX_LIDAR
    else if (nodes.esp.m === 'ng') fix = S00_DIAG_FIX_ESP32
    else if (nodes.pc.m === 'ng') fix = S00_DIAG_FIX_PC
    else if (nodes.ap.m === 'ng' || nodes.tab.m === 'ng') fix = S00_DIAG_FIX_AP
  }
  const checkedAt = new Date().toLocaleTimeString('ja-JP', { hour12: false })

  const st = (k) => nodes[k].m

  return (
    <div className="screen" id="s00">
      <div className="content g-conn">
        <div><h3>{S00_CONNECT_TITLE}</h3></div>
        <div className="diag" data-testid="s00-diagram">
          {svg}
          <div className="fix">
            {fix && (
              <span data-testid="s00-fix">
                <span className="ac"><span className="mk ng" aria-hidden="true">×</span></span>
                <span>{fix}</span>
              </span>
            )}
            <span className="time" data-testid="s00-fixed-at">{S00_DIAG_CHECKED_AT(checkedAt)}</span>
          </div>
        </div>
        <div className="side2">
          <HelpTip helpKey="s00-advance">
            <ReasonButton
              className="primary wide"
              data-testid="s00-advance"
              disabled={!ready}
              reason={!ready ? (restarting ? `制御系を再起動しています（${restartAttempt} 回目）` : (linkNotReady ? '必須機器が繋がっていません' : '確認中')) : null}
              reasonTestId="s00-advance-reason"
              onClick={onAdvance}
            >
              <span>{S00_ADVANCE}</span>
            </ReasonButton>
          </HelpTip>
          {restarting && (
            <div className="note" data-testid="s00-restart">
              <span>{`制御系を再起動しています（${restartAttempt} 回目）`}</span>
            </div>
          )}
          <div className="card" data-testid="s00-auto-check">
            <h3>{S00_AUTO_TITLE}</h3>
            <div className="row">
              <span className={`pill ${overall.cls === 'ok' ? 'ok' : overall.cls}`} data-testid="s00-auto-overall">
                {overall.label}
              </span>
            </div>
            {auto && !auto.suppressed && (
              <div className="ac">
                {S00_AUTO_ITEM_ORDER.map((key) => {
                  const it = auto.items[key] ?? { result: 'CHECKING', reason: '' }
                  return (
                    <div key={key} data-testid={`s00-auto-${key}`}>
                      <span>{s00AutoResultLabel(it.result)}{it.reason ? ` (${it.reason})` : ''}</span>
                    </div>
                  )
                })}
              </div>
            )}
          </div>
          <div className="push-bottom">
            {onOpenSettings && (
              <HelpTip helpKey="s00-dev">
                <button
                  type="button"
                  className="btn wide"
                  data-testid="s00-open-dev"
                  onClick={onOpenSettings}
                >
                  {S00_OPEN_DEV}
                </button>
              </HelpTip>
            )}
          </div>
        </div>
      </div>
    </div>
  )
}
