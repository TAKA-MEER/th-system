#pragma once

// ============================================================
// TH System ESP32 — config.h
// ハードウェア定数・チューニングパラメータ
// ============================================================

// ── デバッグ用シリアル ───────────────────────────────────────
#define SERIAL_BAUD       115200     // Serial.print デバッグ出力用 (UART0)

// ── エンコーダ ───────────────────────────────────────────────
// CuGo v3i エンコーダ: 4096 count/rev (A-B 相)
// 新シールド基板のピン配置。ESP32-test/lib/hw_common/pins.h と同一に揃えてある。
#define ENC_LEFT_A        36
#define ENC_LEFT_B        39
// 右エンコーダも同様にA/B入れ替えで回転方向の符号を反転(実機検証で確認)
#define ENC_RIGHT_A       35
#define ENC_RIGHT_B       34
#define ENC_COUNTS_PER_REV  4096.0f  // 1 回転あたりのカウント数

// ── モータードライバ (Cytron MD10C: DIR + PWM) ────────────────
// 新シールド基板のピン配置。ESP32-test/lib/hw_common/pins.h と同一に揃えてある。
#define MOT_RIGHT_DIR     33
#define MOT_RIGHT_PWM     25
#define MOT_LEFT_DIR      27
#define MOT_LEFT_PWM      26

// PWM チャンネル (ESP32 LEDC)
#define PWM_CH_RIGHT      0
#define PWM_CH_LEFT       1
#define PWM_FREQ_HZ       5000       // 5 kHz
#define PWM_RESOLUTION    8          // 8-bit (0-255)

// 正転方向 (実機で確認し必要なら 0/1 を反転)
#define MOT_RIGHT_FWD     1          // DIR=HIGH で前進
#define MOT_LEFT_FWD      1          // DIR=HIGH で前進 (実機検証で反転を確認)

// ── ロボット寸法 (オドメトリキャリブレーションで上書き) ────────
// ※ esp32_bridge 側でも同値を使用しており両方の変更が必要
#define WHEEL_RADIUS_M    0.019412f    // m  (実測後に calib ツールで調整)
#define WHEEL_BASE_M      0.39f      // m  (実測後に calib ツールで調整)

// ── PID パラメータ ──
// ESP32-test/stage4_timed_distance でのベンチ試験(機体拘束状態)で
// チューニングした値をそのまま転記。ベンチと実ロボットで負荷条件(走行抵抗・
// 重量配分)が異なる可能性があるため、実ロボットでの再検証が必須。
// 詳細: ESP32-test/TEST_LOG.md
#define PID_KP_RIGHT      210.0f
#define PID_KI_RIGHT       80.0f
#define PID_KD_RIGHT        1.0f

#define PID_KP_LEFT       120.0f
#define PID_KI_LEFT        80.0f
#define PID_KD_LEFT         1.0f

// フィードフォワードゲイン [PWM/(m/s)]。PID 単独だと積分が積み上がるまで
// 出力不足になるため、目標速度比例の基準出力を先に与える。
// 実機実測 (2026-07-11): PWM 140 で約 0.45〜0.5 m/s (無負荷寄り) → k ≈ 280。
// 過大にすると PID の補正幅 (ITERM_MAX=100) を超えて長時間オーバーシュートする。
#define PID_KFF           280.0f

#define PID_OUT_MIN      -200.0f    // PWM 出力下限
#define PID_OUT_MAX       200.0f    // PWM 出力上限
#define PID_ITERM_MAX     100.0f    // 積分ワインドアップ防止

// 目標速度のランプ加速度 (m/s^2)。wheel_cmd がステップ変化しても目標速度を
// 徐々にしか動かさないことで、PID比例項の急崩壊による起動直後の振動を防ぐ。
// (ESP32-test/stage4_timed_distance で発見・検証した対策と同一)
#define TARGET_RAMP_ACCEL_MPS2  1.5f

// ── タイマー周期 ─────────────────────────────────────────────
#define CTRL_PERIOD_MS    100        // 制御ループ周期 (ms)
#define WATCHDOG_MS       300        // wheel_cmd 受信タイムアウト → モーター停止

// ── E-Stop 入力 ──────────────────────────────────────────────
// 物理スイッチの別端子を接続
// スイッチ OFF(モーター電断)時の論理レベル:
//   LOW_ACTIVE=true  → GPIO が LOW のとき E-Stop 発動
//   LOW_ACTIVE=false → GPIO が HIGH のとき E-Stop 発動
// 実機の回路に合わせて変更すること
// 新シールド基板ではGPIO34がENC_RIGHT_Bに割り当たったため、E-StopはGPIO32に変更。
#define ESTOP_GPIO        32
#define ESTOP_LOW_ACTIVE  true       // LOW で E-Stop 発動

// ⚠️ ベンチ試験用の一時バイパス（現在有効） ⚠️
// 物理E-Stopスイッチ/外部プルアップ抵抗が未配線だとフローティングで常時
// E-Stop 発動状態になり駆動系試験ができない。
// 物理E-Stopスイッチが未配線のため一時的に有効化中。
// 実機に物理E-Stopスイッチを配線したら必ず無効化(コメントアウト)すること。
// ベンチ試験以外での使用禁止。
#define ESTOP_BENCH_TEST_BYPASS

// ── IMU (DSR1603 / BNO055, I2C) ────────────────────────────────
#define IMU_SDA           21
#define IMU_SCL           22
#define IMU_I2C_ADDR      0x28
