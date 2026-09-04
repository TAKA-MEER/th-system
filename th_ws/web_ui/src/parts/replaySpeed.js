// parts/replaySpeed.js — 再生速度コントロール（WS-9T）の比率テーブル。
// JSX を含まない純モジュールなので node --test から直接読める
// (ReplaySpeedControl.jsx はここを import する)。
//
// 返すのは「比率」(0..1) であって m/s ではない。実速度への換算は replay_runner が
// replay_cruise_min_mps 〜 cruise_speed_mps の間の補間として持つ
// (route_replay_core.scale_replay_params)。ブラウザに実速度上限を置かない方針は
// jog の speed_presets と同じ (DetailedDesign-wp3.md WP-UI-03 §3.3)。
export const REPLAY_SPEED_RATIOS = { low: 0.35, mid: 0.65, high: 1.0 }
