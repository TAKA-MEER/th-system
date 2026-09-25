// i18n/checks.js — CheckStatus.detail (the `reason` string check_core.py's
// CheckVerdict carries) -> Japanese, for S-30 (WP-UI-08).
//
// Source of truth: th_ws/src/th_maintenance/th_maintenance/check_core.py.
// Reason keys are per-item (judge_estop / judge_motor_samples / judge_imu /
// judge_gyro_unit / judge_lidar); "no_data" and "no_samples" mean different
// things depending on which item produced them, so the lookup is item-scoped
// (checkReasonLabel(item, reason)) rather than a single flat map.
//
// `answer_mismatch` comes from opcheck_runner.py itself (not check_core.py):
// the ESTOP item's verdict is overridden to NG when the operator answers
// "いいえ" (mismatch) to the visual-confirm question.
const BY_ITEM = {
  ESTOP: {
    no_data: '非常停止ボタンの状態が届いていません（配線・GPIO を確認してください）',
    no_press: '押下を検出できませんでした',
    stuck_release: '解除を検出できませんでした（固着の疑い）',
    answer_mismatch: '実物と画面表示が一致しないと回答されました',
  },
  MOTOR: {
    no_samples: '指令を送っても応答がありませんでした',
    sign_mismatch_L: '左輪の指令と実測の符号が逆です（配線・極性を確認してください）',
    sign_mismatch_R: '右輪の指令と実測の符号が逆です（配線・極性を確認してください）',
    no_follow_L: '左輪が指令に追従していません',
    no_follow_R: '右輪が指令に追従していません',
  },
  IMU: {
    no_data: 'IMU のデータが届いていません',
    bias_too_large: '静止時のジャイロ値が大きすぎます',
    needs_calibration: '校正が必要です',
    wz_implausible: 'ジャイロの値が単位不一致の疑いがあります（rad/s ではない可能性）',
    no_samples: '静止観測中にサンプルが集まりませんでした',
  },
  LIDAR: {
    no_data: 'LiDAR のデータが届いていません',
    scan_stale: 'スキャンの周期が乱れています',
    coverage_gap: '無効なビームが連続する範囲があります',
    blind_mismatch: '死角マスクの設定と実際の死角がずれています',
  },
}

export function checkReasonLabel(item, reason) {
  if (!reason) return ''
  return BY_ITEM[item]?.[reason] ?? reason
}
