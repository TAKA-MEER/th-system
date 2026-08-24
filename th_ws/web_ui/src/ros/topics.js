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
}

export const SERVICES = {
  SYSTEM_TRIGGER: '/system/trigger',
  // std_srvs/Trigger, not th_system_msgs/UiTrigger -- see ros/useStdTrigger.js.
  SHUTDOWN_PREPARE: '/shutdown/prepare',
  SHUTDOWN_EXECUTE: '/shutdown/execute',
}

export const MSG_TYPES = {
  SYSTEM_STATE: 'th_system_msgs/SystemState',
  PARAMS_STATUS: 'th_system_msgs/ParamsStatus',
  ACTIVE_SCREEN: 'th_system_msgs/ActiveScreen',
  BOOL: 'std_msgs/Bool',
  FAULT_STATUS: 'th_system_msgs/FaultStatus',
}

export const SRV_TYPES = {
  UI_TRIGGER: 'th_system_msgs/UiTrigger',
  STD_TRIGGER: 'std_srvs/Trigger',
}
