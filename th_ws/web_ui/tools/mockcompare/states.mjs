// tools/mockcompare/states.mjs — 状態の対応表（モックアップの状態 ↔ 実画面の状態）。
//
// 1 項目 = 1 つの比べる状態。名前は `<画面>-<状態>`。
//   mock   モックアップで状態を作る JS（docs/manual/tools/capture.py の HELPERS_JS と同じ小道具）:
//            st('replay','run')                画面 replay の状態 run（data-scr / data-sub をクリック）
//            st('follow','entry',{teach:true}) 教示（追従・手動）は teach を立てる
//            ab('fault')                       異常: none / fault / estop / carry / resolved
//            act1('jog')                       画面の中のボタン（data-act）を押す
//          S・render() などモックアップの関数もそのまま呼べる。向き（S.ori）は道具が後から設定する。
//   real   実画面で状態を作るもの:
//            screen   window.__thTestScreen（main.jsx）。省略すると 'S01'
//            state    window.__thTestState（ros/useSystemState.js）
//            seeds    window.<名前> に入れる種（__thTestOnsitePins など。e2e/helpers.js と同じ）
//            after    読み込み後に page で行うこと（async (page) => {...}）
//          null のときは載せられなかった状態。why に理由を書く（報告の「載せられなかった状態」）。
//   oris   測る向き（既定 ['L','P']）
//   ignore 意図して合わせない領域 { 領域名: '理由（spec の節など）' } か { 領域名: { why, only: ['x','y','w','h','字','色','背景',…] } }。表には「除外」として残り、超過に数えない。
//          spec とモックアップが食い違う所・機体から届かない値など（brief-F0「一致させない」）だけに使う
//
// 実画面の値（地図・点群・数値）はモックアップの見本と違ってよい。枠と配置を比べる（brief-F0）。

const pin = (id, name, kind, x, y, yawDeg = 90) => {
  const r = (yawDeg * Math.PI) / 180
  return {
    id, name, kind, registered_at: '',
    pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(r / 2), w: Math.cos(r / 2) } },
  }
}
const PINS = [
  pin('p1', 'A-1', 'PANEL', 2, 1, 92), pin('p2', 'A-2', 'PANEL', 4, 1, 91),
  pin('p3', 'B-1', 'PANEL', 2, 3), pin('p4', 'B-2', 'PANEL', 4, 3),
  pin('p5', 'C-1', 'PANEL', 6, 2), pin('p6', '待機場所', 'HOME', 0, 0, 274),
]
const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }, { x: 2, y: 1 }, { x: -2, y: 1 }],
  selected_index: 0, confidence: 0.87, is_lost: false, lost_reason: '',
}
const TARGETS_NONE = { ...TARGETS, selected_index: -1 }
const ONSITE = {
  __thTestOnsitePins: PINS, __thTestPersonTargets: TARGETS,
  __thTestOdomPose: { x: 0, y: 0, yaw: 0 }, __thTestRoutePose: { x: 0, y: 0, yaw: 0 },
  __thTestHomeDeclared: true,
  __thTestMapSession: { slot: 'VENUE', mode: 'LOADED', dirty: false, instance_id: 'x' },
}
const ROUTES = [
  { id: 'route_c2', name: '試験ブロックC2', length_m: 24.6, point_count: 180, start_yaw: 0.1, recorded_at: 0 },
  { id: 'route_a', name: '経路A', length_m: 8.2, point_count: 60, start_yaw: 0.1, recorded_at: 0 },
  { id: 'route_b', name: '経路B', length_m: 12.4, point_count: 90, start_yaw: 0.1, recorded_at: 0 },
]
const PREVIEW = Array.from({ length: 40 }, (_, i) => ({ x: i * 0.2, y: Math.sin(i / 5) }))
const SCAN = {
  angle_min: -3.14, angle_increment: 0.0175, range_min: 0.1, range_max: 12,
  ranges: Array.from({ length: 360 }, (_, i) => 2 + Math.sin(i / 20)),
}
const REPLAY = { __thTestRouteCatalog: ROUTES, __thTestRoutePreview: PREVIEW, __thTestScan: SCAN }
const LINK_ITEM = { ok: true, age_ms: 80, ignored: false, excluded: false }
const LINK_OK = {
  items: {
    esp32_feedback: LINK_ITEM, esp32_loopback: LINK_ITEM,
    lidar: { ...LINK_ITEM, points: 720, expected_points: 720 }, nodes: { ...LINK_ITEM, missing: [] },
  },
  estop_hw: { seen: true, pressed: false },
}
const LINK_LIDAR = {
  ...LINK_OK,
  items: { ...LINK_OK.items, lidar: { ok: false, age_ms: null, points: 0, expected_points: 720, ignored: false, excluded: false } },
}
const CALIB_DETAIL = {
  commanded: 1.0, current: { wheel_radius_scale: 1.0 }, progress: 0.4, warnings: [],
  last_calibrated: { LINEAR: '2026-10-01T10:00:00', ROTATION: '', IMU: '' },
  history: { LINEAR: [{ generation: 1, values: { wheel_radius_scale: 0.98 }, calibrated_at: '2026-09-30T09:00:00' }], ROTATION: [], IMU: [] },
}
const calib = (o = {}) => ({ item: 'LINEAR', step: 'S1', result: 'GUIDE', preview_before: '{"a":1}', preview_after: '{"a":2}', detail: CALIB_DETAIL, ...o })
const opcheck = (item, result = 'UNKNOWN', detail = '') => ({ __thTestOpcheckStatus: { item, result, detail, next_screen: '' } })

const click = (testId) => async (page) => { await page.getByTestId(testId).click() }
const clicks = (...ids) => async (page) => { for (const id of ids) await page.getByTestId(id).click() }
const fault = (f) => async (page) => { await page.evaluate((p) => window.__thSetTestFault(p), f) }
const setState = (st) => async (page) => { await page.evaluate((p) => window.__thSetTestState(p), st) }
const LIDAR_FAULT = { active: true, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' }

// 全状態に効く除外（理由つき）。画面ごとのものは各状態の ignore に書く
export const GLOBAL_IGNORE = {
  // Spec-webui §1.1「高見舎ロゴ」。実装は本物のロゴ（src/shell/logo.js。横長 56×24 を白地の札に載せる）、
  // モックアップは仮の「TH」（30×30）。ロゴが 38 px 広いぶん、プロジェクト名と画面名称が右へずれる（x だけ除く）
  'header-logo': 'Spec-webui §1.1: 本物の高見舎ロゴ（モックアップは仮の TH）',
  'header-brand': { why: 'ロゴの幅の差（header-logo）で右へずれる', only: ['x'] },
  'header-title': { why: 'ロゴの幅の差（header-logo）で右へずれる', only: ['x'] },
}

const S = (name, mock, real, extra = {}) => ({ name, mock, real, ...extra })
const NA = (why) => ({ na: why })

export const STATES = [
  // ---------------- S-00 接続確認（見る・変える）----------------
  S('S00-ok', "st('connect','ok')", { screen: 'S00', state: { mode: 'INIT', state: 'CHECK' }, seeds: { __thTestLinkStatus: LINK_OK } }),
  S('S00-lidar', "st('connect','lidar')", { screen: 'S00', state: { mode: 'INIT', state: 'CHECK' }, seeds: { __thTestLinkStatus: LINK_LIDAR } }),
  S('S00-camera', "st('connect','camera')", NA('実画面の接続確認にカメラの項目が無い（機体の疎通確認がカメラを見ていない）')),
  S('S00-restart', "st('connect','restart')", NA('「再起動中」を実画面で作る種が無い（制御系の再起動はヘッダの札だけで、試験用の縫い目が無い）')),

  // ---------------- S-01 メインメニュー ----------------
  S('S01-top', "st('menu','top')", { screen: 'S01', state: { mode: 'IDLE', state: 'NONE', tracker_enabled: true } }),
  S('S01-field', "st('menu','field')", { screen: 'S01', state: { mode: 'IDLE', state: 'NONE', tracker_enabled: true }, after: click('s01-group-field') }),
  S('S01-drive', "st('menu','drive')", { screen: 'S01', state: { mode: 'IDLE', state: 'NONE', tracker_enabled: true }, after: click('s01-group-drive') }),
  S('S01-maint', "st('menu','maint')", { screen: 'S01', state: { mode: 'IDLE', state: 'NONE', tracker_enabled: true }, after: click('s01-group-maint') }),
  S('S01-shutdown', "st('menu','shutdown')", { screen: 'S01', state: { mode: 'IDLE', state: 'NONE', tracker_enabled: true, unsaved: ['venue_map', 'route'] }, after: click('s01-shutdown') }),
  S('S01-autoask', "st('menu','autoask')", { screen: 'S01', state: { mode: 'IDLE', state: 'NONE', tracker_enabled: true }, after: click('s01-open-auto') }),

  // ---------------- S-10 追従走行（走らせる）----------------
  S('S10-pick', "st('follow','pick')", { screen: 'S10', state: { mode: 'FOLLOW', state: 'SELECT', tracker_enabled: true }, seeds: { ...ONSITE, __thTestPersonTargets: TARGETS_NONE } }),
  S('S10-ready', "st('follow','ready')", { screen: 'S10', state: { mode: 'FOLLOW', state: 'CONFIRM', tracker_enabled: true }, seeds: ONSITE }),
  S('S10-run', "st('follow','run')", { screen: 'S10', state: { mode: 'FOLLOW', state: 'RUN', tracker_enabled: true }, seeds: ONSITE }),
  S('S10-paused', "st('follow','paused')", { screen: 'S10', state: { mode: 'FOLLOW', state: 'PAUSE', tracker_enabled: true }, seeds: ONSITE }),

  // ---------------- S-11 手動走行（走らせる）----------------
  S('S11-idle', "st('manual','idle')", { screen: 'S11', state: { mode: 'MANUAL', state: 'PAUSE', tracker_enabled: true } }),
  S('S11-drive', "st('manual','drive')", { screen: 'S11', state: { mode: 'MANUAL', state: 'RUN', tracker_enabled: true }, seeds: { __thTestScan: SCAN } }),
  S('S11-back', "st('manual','back')", { screen: 'S11', state: { mode: 'MANUAL', state: 'RUN', tracker_enabled: true }, seeds: { __thTestScan: SCAN } }),
  S('S11-spin', "st('manual','spin')", { screen: 'S11', state: { mode: 'MANUAL', state: 'RUN', tracker_enabled: true }, seeds: { __thTestScan: SCAN } }),
  S('S11-near', "st('manual','near')", { screen: 'S11', state: { mode: 'MANUAL', state: 'RUN', tracker_enabled: true }, seeds: { __thTestScan: SCAN } }),
  S('S11-paused', "st('manual','paused')", { screen: 'S11', state: { mode: 'MANUAL', state: 'PAUSE', tracker_enabled: true } }),

  // ---------------- S-12 教示（追従）----------------
  S('S12-entry', "st('follow','entry',{teach:true})", { screen: 'S12', state: { mode: 'TEACH_FOLLOW', state: 'ROUTE_SEL', tracker_enabled: true }, seeds: { __thTestRouteCatalog: ROUTES, __thTestPersonTargets: TARGETS } }),
  S('S12-pick', "st('follow','pick',{teach:true})", NA('実画面の教示（追従）は経路を選んだ直後に記録へ進み、「対象を選ぶ」の段を別に持たない（機体が受け付ける順。brief-F0 一致させない）')),
  S('S12-ready', "st('follow','ready',{teach:true})", NA('同上（「選んだ・止まっている」の段が無い）')),
  S('S12-run', "st('follow','run',{teach:true})", { screen: 'S12', state: { mode: 'TEACH_FOLLOW', state: 'REC', tracker_enabled: true }, seeds: { __thTestRouteCatalog: ROUTES, __thTestRoutePreview: PREVIEW, __thTestPersonTargets: TARGETS, __thTestRouteStatus: { recorded_m: 12.4, elapsed_sec: 92, points: 100, start_yaw: 0.5, saved: false } } }),
  S('S12-paused', "st('follow','paused',{teach:true})", { screen: 'S12', state: { mode: 'TEACH_FOLLOW', state: 'PAUSE', tracker_enabled: true }, seeds: { __thTestRouteCatalog: ROUTES, __thTestRoutePreview: PREVIEW, __thTestPersonTargets: TARGETS } }),

  // ---------------- S-13 教示（手動）----------------
  S('S13-entry', "st('manual','entry',{teach:true})", { screen: 'S13', state: { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL', tracker_enabled: true }, seeds: { __thTestRouteCatalog: ROUTES } }),
  S('S13-idle', "st('manual','idle',{teach:true})", { screen: 'S13', state: { mode: 'TEACH_MANUAL', state: 'PAUSE', tracker_enabled: true }, seeds: { __thTestRoutePreview: PREVIEW } }),
  S('S13-drive', "st('manual','drive',{teach:true})", { screen: 'S13', state: { mode: 'TEACH_MANUAL', state: 'REC', tracker_enabled: true }, seeds: { __thTestRoutePreview: PREVIEW, __thTestScan: SCAN } }),
  S('S13-paused', "st('manual','paused',{teach:true})", { screen: 'S13', state: { mode: 'TEACH_MANUAL', state: 'PAUSE', tracker_enabled: true }, seeds: { __thTestRoutePreview: PREVIEW } }),
  S('S13-saved', "st('manual','saved',{teach:true})", { screen: 'S13', state: { mode: 'TEACH_MANUAL', state: 'SAVED', tracker_enabled: true }, seeds: { __thTestRoutePreview: PREVIEW } }),

  // ---------------- S-14 教示再生 ----------------
  S('S14-select', "st('replay','select')", { screen: 'S14', state: { mode: 'REPLAY', state: 'ROUTE_SEL', tracker_enabled: true }, seeds: REPLAY }),
  S('S14-loading', "st('replay','loading')", NA('地図の読み込み中を作る種が無い（/route/status の読み込み中の印を実画面が持たない）')),
  S('S14-search', "st('replay','search')", { screen: 'S14', state: { mode: 'REPLAY', state: 'LOCALIZE', tracker_enabled: true }, seeds: REPLAY }),
  S('S14-notfound', "st('replay','notfound')", NA('「見つからない」を作る種を確かめていない（s14-localize-stuck の時間待ちが要る）')),
  S('S14-far', "st('replay','far')", NA('「経路から離れている」を作る種を確かめていない')),
  S('S14-ready', "st('replay','ready')", { screen: 'S14', state: { mode: 'REPLAY', state: 'READY', tracker_enabled: true }, seeds: REPLAY }),
  S('S14-run', "st('replay','run')", { screen: 'S14', state: { mode: 'REPLAY', state: 'RUN', tracker_enabled: true }, seeds: { ...REPLAY, __thTestRouteStatus: { target_index: 5, cross_track_m: 0.1, cross_track_max_m: 0.3 } } }),
  S('S14-paused', "st('replay','paused')", { screen: 'S14', state: { mode: 'REPLAY', state: 'PAUSE', tracker_enabled: true }, seeds: REPLAY }),
  S('S14-end', "st('replay','end')", NA('終点の状態を作る種を確かめていない')),

  // ---------------- S-20 試験準備 ----------------
  S('S20-entry', "st('prep','entry')", NA('実画面の試験準備に入口のカードの状態を作る種を確かめていない')),
  S('S20-mapping', "st('prep','mapping')", { screen: 'S20', state: { mode: 'PREP', state: 'MAPPING', tracker_enabled: true, zone: 'IN' }, seeds: { ...ONSITE, __thTestMappingActive: true } }),
  S('S20-reg', "st('prep','reg')", { screen: 'S20', state: { mode: 'PREP', state: 'REGISTER', tracker_enabled: true, zone: 'IN' }, seeds: { ...ONSITE, __thTestMappingActive: true } }),
  S('S20-return', "st('prep','return')", { screen: 'S20', state: { mode: 'PREP', state: 'RETURN', tracker_enabled: true, zone: 'IN' }, seeds: { ...ONSITE, __thTestMappingActive: true } }),

  // ---------------- S-21 試験（当日）----------------
  S('S21-unloaded', "st('day','unloaded')", { screen: 'S21', state: { mode: 'AT_HOME', state: 'IDLE_H', tracker_enabled: true, zone: 'IN' }, seeds: { ...ONSITE, __thTestMapSession: { slot: 'VENUE', mode: 'UNLOADED', dirty: false, instance_id: 'x' } } }),
  S('S21-home', "st('day','home')", { screen: 'S21', state: { mode: 'AT_HOME', state: 'IDLE_H', tracker_enabled: true, zone: 'IN' }, seeds: ONSITE }),
  S('S21-moving', "st('day','moving')", { screen: 'S21', state: { mode: 'PANEL_NAV', state: 'NAV', tracker_enabled: true, zone: 'IN' }, seeds: ONSITE }),
  S('S21-paused', "st('day','paused')", { screen: 'S21', state: { mode: 'PANEL_NAV', state: 'PAUSE', tracker_enabled: true, zone: 'IN' }, seeds: ONSITE }),
  S('S21-blocked', "st('day','blocked')", { screen: 'S21', state: { mode: 'PANEL_NAV', state: 'BLOCKED', tracker_enabled: true, zone: 'IN' }, seeds: ONSITE }),
  S('S21-panel', "st('day','panel')", { screen: 'S21', state: { mode: 'AT_PANEL', state: 'IDLE_P', tracker_enabled: true, zone: 'IN' }, seeds: ONSITE }),
  S('S21-working', "st('day','working')", { screen: 'S21', state: { mode: 'AT_PANEL', state: 'WORKING', tracker_enabled: true, zone: 'IN' }, seeds: ONSITE }),
  S('S21-summon', "st('day','summon')", { screen: 'S21', state: { mode: 'SUMMON', state: 'POINT', tracker_enabled: true, zone: 'IN' }, seeds: ONSITE }),
  S('S21-waitclear', "st('day','waitclear')", { screen: 'S21', state: { mode: 'SUMMON', state: 'WAIT_CLEAR', tracker_enabled: true, zone: 'IN' }, seeds: { ...ONSITE, __thTestWaitClear: { distance_m: 0.4, remaining_sec: 7, satisfied: false, verdict: 'WAITING' } } }),

  // ---------------- S-30 始業点検・S-31 故障診断（手順を進める）----------------
  S('S30-list', "st('check','list')", { screen: 'S30', state: { mode: 'OPCHECK', state: 'LIST' } }),
  S('S30-estop', "st('check','estop')", { screen: 'S30', state: { mode: 'OPCHECK', state: 'RUNNING_CHECK' }, seeds: opcheck('ESTOP') }),
  S('S30-motor', "st('check','motor')", { screen: 'S30', state: { mode: 'OPCHECK', state: 'RUNNING_CHECK' }, seeds: opcheck('MOTOR', 'UNKNOWN', 'done= hold=NONE') }),
  S('S30-imu', "st('check','imu')", { screen: 'S30', state: { mode: 'OPCHECK', state: 'RUNNING_CHECK' }, seeds: opcheck('IMU') }),
  S('S30-lidarNg', "st('check','lidarNg')", { screen: 'S30', state: { mode: 'OPCHECK', state: 'RUNNING_CHECK' }, seeds: { ...opcheck('LIDAR', 'NG', 'coverage_gap'), __thTestScan: SCAN } }),
  S('S31-repair', "st('check','repair')", { screen: 'S31', state: { mode: 'OPCHECK', state: 'REPAIR' } }),

  // ---------------- S-40 校正（手順を進める）----------------
  S('S40-list', "st('calib','list')", { screen: 'S40', state: { mode: 'CALIB', state: 'LIST' }, seeds: { __thTestCalibStatus: calib({ result: 'IDLE', item: '', step: '' }) } }),
  S('S40-s1', "st('calib','s1')", { screen: 'S40', state: { mode: 'CALIB', state: 'S1' }, seeds: { __thTestCalibStatus: calib({ step: 'S1', result: 'GUIDE' }) } }),
  S('S40-s2', "st('calib','s2')", { screen: 'S40', state: { mode: 'CALIB', state: 'S2' }, seeds: { __thTestCalibStatus: calib({ step: 'S2', result: 'RUNNING' }) } }),
  S('S40-s3', "st('calib','s3')", { screen: 'S40', state: { mode: 'CALIB', state: 'S3' }, seeds: { __thTestCalibStatus: calib({ step: 'S3', result: 'WAIT_MEASURED' }) } }),
  S('S40-s4', "st('calib','s4')", { screen: 'S40', state: { mode: 'CALIB', state: 'S4' }, seeds: { __thTestCalibStatus: calib({ step: 'S4', result: 'VERIFY' }) } }),
  S('S40-imu1', "st('calib','imu1')", { screen: 'S40', state: { mode: 'CALIB', state: 'S1' }, seeds: { __thTestCalibStatus: calib({ item: 'IMU', step: 'S1', result: 'GUIDE' }) } }),
  S('S40-blind', "st('calib','blind')", { screen: 'S40', state: { mode: 'CALIB', state: 'S3' }, seeds: { __thTestCalibStatus: calib({ item: 'BLIND', step: 'S3', result: 'WAIT_MEASURED' }), __thTestScan: SCAN } }),
  S('S40-hist', "st('calib','hist')", NA('履歴の窓を開く操作の対応を確かめていない（実画面は一覧の「履歴」ボタンから）')),

  // ---------------- S-50 設定（見る・変える）----------------
  ...['run', 'loc', 'notify', 'screen', 'link', 'log', 'blind', 'dev'].map((sec) =>
    S(`S50-${sec}`, `st('settings','${sec}')`, {
      screen: 'S01', state: { mode: 'IDLE', state: 'NONE', tracker_enabled: true },
      after: clicks('s01-open-settings', `s50-sec-${sec}`),
    })),

  // ---------------- S-60 自律測定 ----------------
  S('S60-wait', "st('auto','wait')", NA('実機では未実装の画面（モックアップの付録）')),
  S('S60-turn', "st('auto','turn')", NA('同上')),
  S('S60-paused', "st('auto','paused')", NA('同上')),

  // ---------------- 窓（異常・非常停止・確認）----------------
  // モックアップの図解（docs/manual/tools/shots.py）と同じく、教示再生の再生中に重ねる
  S('W1-fault', "st('replay','run'); ab('fault')", { screen: 'S14', state: { mode: 'REPLAY', state: 'PAUSE', tracker_enabled: true }, seeds: REPLAY, after: fault(LIDAR_FAULT) }),
  S('W1-resolved', "st('replay','run'); ab('fault'); ab('resolved')", { screen: 'S14', state: { mode: 'REPLAY', state: 'PAUSE', tracker_enabled: true }, seeds: REPLAY, after: async (page) => { await fault(LIDAR_FAULT)(page); await fault({ ...LIDAR_FAULT, active: false })(page) } }),
  // 実画面は再生中から非常停止へ移す（mode が ESTOP／CARRY の間、画面は MODE_TO_SCREEN に無いのでメインメニューに戻る。
  // モックアップは再生の画面の上に窓を出す。この違いは表の「実画面に無い」に出る）
  S('W2-estop', "st('replay','run'); ab('estop')", { screen: 'S14', state: { mode: 'REPLAY', state: 'RUN', tracker_enabled: true }, seeds: REPLAY,
    after: setState({ mode: 'ESTOP', state: 'NONE', prev_mode: 'REPLAY', estop_from_ui: true, estop_ui: true, estop_hw: false }) }),
  S('W2-carry', "st('replay','run'); ab('carry')", { screen: 'S14', state: { mode: 'REPLAY', state: 'RUN', tracker_enabled: true }, seeds: REPLAY,
    after: setState({ mode: 'CARRY', state: 'NONE', prev_mode: 'REPLAY', estop_hw: true }) }),
  S('W6-jog', "st('replay','run'); act1('jog')", NA('実画面の教示再生に W-6（手動の窓）を開く操作の対応を確かめていない')),
]
