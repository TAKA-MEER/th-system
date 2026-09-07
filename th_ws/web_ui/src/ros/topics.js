// ros/topics.js — topic/service names and rosbridge message-type strings
// used by the shell. Every name under TOPICS / SERVICES must exist in
// names.json's `endpoints` (topics union services) — see
// test/unit/topics-in-dictionary.test.js and DetailedDesign-webui.md §9.1.
// Screens (WP-UI-02+) add their own topics as those packets land; this file
// only carries what the shell itself talks to.

export const TOPICS = {
  SYSTEM_STATE: '/system/state',
  PARAMS_STATUS: '/system/params_status',
  ACTIVE_SCREEN: '/ui/active_screen',
  ESTOP_UI: '/safety/estop_ui',
  // WP-UI-02: WP-UI-01's own §3.1 already listed this sub (header fault
  // display, W-1), but it wasn't wired up -- see DetailedDesign-wp1.md
  // WP-UI-01 §11 "W-1 generalization".
  SAFETY_FAULT: '/safety/fault',
  // WP-UI-03 (DetailedDesign-wp3.md §3.1): the two topics useJogLease
  // publishes. /ui/jog_lease carries this client's id (not a /system/trigger
  // call -- a synchronous rosbridge round-trip 5x/s would stall on WiFi
  // gaps, DetailedDesign-webui.md §5); /cmd_vel_manual_raw is the raw
  // velocity in front of jog_gate (WebUI is the only /cmd_vel_manual_raw
  // publisher and it only ever reaches the motor through jog_gate ->
  // twist_mux -> obstacle_limiter).
  JOG_LEASE: '/ui/jog_lease',
  CMD_VEL_MANUAL_RAW: '/cmd_vel_manual_raw',
  // WP-TRANSIT-01 §3.1: obstacle_limiter の出力（action / nearest_obstacle_m /
  // applied_limit_mps）。publisher は 20Hz best_effort なので SubscribeOptions
  // も合わせる。S-11 の障害物警告表示に使う。
  LIMITER_STATUS: '/safety/limiter_status',
  // P5 / demo-teach-replay: 教示（手動）S-13 と 教示再生 S-14 が使う経路一覧・
  // 記録状況。route_recorder（教示中）/ replay_runner（再生中）が publish する。
  ROUTE_CATALOG: '/route/catalog',
  ROUTE_STATUS: '/route/status',
  // WS-3 / demo-teach-replay: 教示・再生の経路プレビュー（nav_msgs/Path、odom フレーム、
  // reliable depth1 2Hz）。route_recorder / replay_runner が publish する。
  // /odom と /scan_filtered は辞書に載るが、useRosbridge.js が /scan_filtered を
  // 直書きしている両例に倣い、本パケットでは hook/コンポーネント側でローカル定数に
  // している（topics.js には増やさない）。
  ROUTE_PREVIEW: '/route/preview',
  // WP-UI-06: S-20 試験準備。試験場内地図のピン一覧（PinList、transient_local）。
  ONSITE_PINS: '/onsite/pins',
  // WP-UI-06: 対象選択の候補（PersonTargets。candidates は base_link 相対）。
  PERSON_TARGETS: '/person/targets',
  // WP-UI-07: S-21 試験。呼び寄せの退避待ち状態（WaitClearStatus、5Hz・
  // wait_clear_gate が SUMMON/WAIT_CLEAR の間だけ配信）。
  WAIT_CLEAR: '/onsite/wait_clear',
}

export const SERVICES = {
  SYSTEM_TRIGGER: '/system/trigger',
  // std_srvs/Trigger, not th_system_msgs/UiTrigger -- see ros/useStdTrigger.js.
  SHUTDOWN_PREPARE: '/shutdown/prepare',
  SHUTDOWN_EXECUTE: '/shutdown/execute',
  // WP-UI-06: S-20。2 点指示（two_point）とピンの改名/削除（edit_pin）。
  ONSITE_TWO_POINT: '/onsite/two_point',
  ONSITE_EDIT_PIN: '/onsite/edit_pin',
  // WP-UI-07: S-21。待機場所の宣言（home_declarer）。選んだ配電盤のゴール保留は
  // `/onsite/select_pin` だが、names.json（辞書ゲート）に無いため TOPICS/SERVICES には
  // 載せず useOnsiteService.js 内のローカル定数で扱う（useOdomPose と同型）。
  ONSITE_DECLARE_HOME: '/onsite/declare_home',
}

export const MSG_TYPES = {
  SYSTEM_STATE: 'th_system_msgs/SystemState',
  PARAMS_STATUS: 'th_system_msgs/ParamsStatus',
  ACTIVE_SCREEN: 'th_system_msgs/ActiveScreen',
  BOOL: 'std_msgs/Bool',
  FAULT_STATUS: 'th_system_msgs/FaultStatus',
  // WP-UI-03: geometry_msgs/Twist (roslib shorthand "geometry_msgs/Twist")
  // and std_msgs/String are both built into roslib, no custom message.
  TWIST: 'geometry_msgs/Twist',
  STRING: 'std_msgs/String',
  // WP-TRANSIT-01 §3.1: LimiterStatus は names.json の msgs にも載っている
  //（既存メッセージ型）。best_effort publisher なので subscribeOptions を
  // 使う側で合わせること（この定数は型名だけ）。
  LIMITER_STATUS: 'th_system_msgs/LimiterStatus',
  // P5 / demo-teach-replay: RouteList / RouteStatus は names.json に載っている
  //（P4.5 で /route/catalog・/route/status を endpoints に追加済み）。
  ROUTE_LIST: 'th_system_msgs/RouteList',
  ROUTE_STATUS: 'th_system_msgs/RouteStatus',
  // WS-3 / demo-teach-replay: /route/preview は nav_msgs/Path（names.json の
  // endpoints にある）。
  PATH: 'nav_msgs/Path',
  // WP-UI-06: S-20。PinList / PersonTargets は names.json の msgs に載っている。
  PIN_LIST: 'th_system_msgs/PinList',
  PERSON_TARGETS: 'th_system_msgs/PersonTargets',
  // WP-UI-07: S-21。待機場所宣言済み（home_declarer が latched 配信）。
  BOOL: 'std_msgs/Bool',
  WAIT_CLEAR: 'th_system_msgs/WaitClearStatus',
}

export const SRV_TYPES = {
  UI_TRIGGER: 'th_system_msgs/UiTrigger',
  STD_TRIGGER: 'std_srvs/Trigger',
  // WP-UI-06: S-20。2 点指示 / ピン編集。
  ONSITE_TWO_POINT: 'th_system_msgs/TwoPointPress',
  ONSITE_EDIT_PIN: 'th_system_msgs/EditPin',
  // WP-UI-07: S-21。待機場所宣言 / ピン選択（select_pin はサービス名が
  // names.json に無いため S21Test 側のローカル定数で呼ぶ。型はここ）。
  ONSITE_DECLARE_HOME: 'th_system_msgs/DeclareHome',
  ONSITE_SELECT_PIN: 'th_system_msgs/GoToPanel',
}
