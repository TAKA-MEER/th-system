#pragma once

// ============================================================
// TH System ESP32 — config.h
// ハードウェア定数・チューニングパラメータ
// ============================================================

// ── デバッグ用シリアル ───────────────────────────────────────
#define SERIAL_BAUD       115200     // Serial.print デバッグ出力用 (UART0)

// ── エンコーダ ───────────────────────────────────────────────
// CuGo v3i エンコーダ: 4096 count/rev (A-B 相)
#define ENC_LEFT_A        4
#define ENC_LEFT_B        13
// 右エンコーダも同様にA/B入れ替えで回転方向の符号を反転(実機検証で確認)
#define ENC_RIGHT_A       5
#define ENC_RIGHT_B       14
#define ENC_COUNTS_PER_REV  4096.0f  // 1 回転あたりのカウント数

// ── モータードライバ (Cytron MD10C: DIR + PWM) ────────────────
#define MOT_RIGHT_DIR     25
#define MOT_RIGHT_PWM     26
#define MOT_LEFT_DIR      32
#define MOT_LEFT_PWM      33

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
#define WHEEL_RADIUS_M    0.0391f    // m  (実測後に calib ツールで調整)
#define WHEEL_BASE_M      0.39f      // m  (実測後に calib ツールで調整)

// ── PID パラメータ (初期値 — 実機チューニング必須) ───────────
#define PID_KP_RIGHT      80.0f
#define PID_KI_RIGHT      30.0f
#define PID_KD_RIGHT       8.0f

#define PID_KP_LEFT      100.0f
#define PID_KI_LEFT       30.0f
#define PID_KD_LEFT        8.0f

#define PID_OUT_MIN      -255.0f    // PWM 出力下限
#define PID_OUT_MAX       255.0f    // PWM 出力上限
#define PID_ITERM_MAX     100.0f    // 積分ワインドアップ防止

// ── タイマー周期 ─────────────────────────────────────────────
#define CTRL_PERIOD_MS    100        // 制御ループ周期 (ms)
#define WATCHDOG_MS       300        // wheel_cmd 受信タイムアウト → モーター停止

// ── E-Stop 入力 ──────────────────────────────────────────────
// 物理スイッチの別端子を接続
// スイッチ OFF(モーター電断)時の論理レベル:
//   LOW_ACTIVE=true  → GPIO が LOW のとき E-Stop 発動
//   LOW_ACTIVE=false → GPIO が HIGH のとき E-Stop 発動
// 実機の回路に合わせて変更すること
#define ESTOP_GPIO        34         // 入力専用ピン
#define ESTOP_LOW_ACTIVE  true       // LOW で E-Stop 発動

// ⚠️ ベンチ試験用の一時バイパス（現在有効） ⚠️
// GPIO34は内部プルアップ非対応のため、物理E-Stopスイッチ/外部プルアップ抵抗が
// 未配線だとフローティングで常時 E-Stop 発動状態になり駆動系試験ができない。
// 物理E-Stopスイッチが未配線(GPIO34はGNDに直結)のため一時的に有効化中。
// 実機に物理E-Stopスイッチを配線したら必ず無効化(コメントアウト)すること。
// ベンチ試験以外での使用禁止。
#define ESTOP_BENCH_TEST_BYPASS

// IMU (未実装・将来予約)
// #define IMU_SDA        21
// #define IMU_SCL        22
