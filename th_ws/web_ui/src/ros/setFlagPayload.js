// ros/setFlagPayload.js — /system/set_flag (th_system_msgs/SetFlag) の送信
// ペイロードを作る純粋関数（devModeState.js と同じ「純粋コア＋薄いフック」
// の流儀。React を import しないので node --test で直接試験できる）。
//
// SetFlag.srv の request は `flag` + `value` だけ（`requester` は無い。
// UiTrigger / SetMode には有る）。rosbridge は存在しないフィールドを拒否
// するので、ここに余計なキーを足すと実機で黙って失敗する（2026-10-10）。
// 呼び出し元の識別（'s20-target' 等）は e2e の記録用タグとして残るが、
// 線上には載せない。
export function buildSetFlagRequest(flag, value) {
  return { flag, value }
}
