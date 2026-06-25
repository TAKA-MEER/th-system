#pragma once

// ============================================================
// TH System ESP32 — config.h
// ハードウェア定数・チューニングパラメータ
// ============================================================

// ── ROS2 設定 ────────────────────────────────────────────────
#define ROS_DOMAIN_ID     10
#define ROS_NODE_NAME     "esp32_node"
#define SERIAL_BAUD       115200     // micro-ROS Agent との通信速度

// ── エンコーダ ───────────────────────────────────────────────
// CuGo v3i エンコーダ: 4096 count/rev (A-B 相)
#define ENC_LEFT_A        4
#define ENC_LEFT_B        5
#define ENC_RIGHT_A       13
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
#define MOT_LEFT_FWD      0          // DIR=LOW  で前進  (左右逆なので反転)

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

// IMU (未実装・将来予約)
// #define IMU_SDA        21
// #define IMU_SCL        22
