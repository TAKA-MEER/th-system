#pragma once

// ============================================================
// ESP32-test / hw_common — ピン定義・ハード定数
//
// th_ws/esp32/src/config.h の実機検証済みの値をそのまま転記している。
// 本体ファームウェア側で配線・極性・寸法を変更した場合はこちらも
// 合わせて更新すること（自動同期はしていない）。
// ============================================================

// ── エンコーダ (CuGo v3i: 4096 count/rev, A-B相) ───────────────
#define ENC_LEFT_A          36
#define ENC_LEFT_B          39
#define ENC_RIGHT_A         35
#define ENC_RIGHT_B         34
#define ENC_COUNTS_PER_REV  4096.0f

// ── モータードライバ (Cytron MD10C: DIR + PWM) ─────────────────
#define MOT_RIGHT_DIR       33
#define MOT_RIGHT_PWM       25
#define MOT_LEFT_DIR        27
#define MOT_LEFT_PWM        26

// PWM チャンネル (ESP32 LEDC)
#define PWM_CH_RIGHT        0
#define PWM_CH_LEFT         1
#define PWM_FREQ_HZ         5000        // 5 kHz
#define PWM_RESOLUTION      8           // 8-bit (0-255)

// 正転方向 (実機検証済み: th_ws/esp32/src/config.h と同値)
#define MOT_RIGHT_FWD       1           // DIR=HIGH で前進
#define MOT_LEFT_FWD        1           // DIR=HIGH で前進

// ── ロボット寸法 ────────────────────────────────────────────
#define WHEEL_RADIUS_M      0.0391f     // m
#define WHEEL_BASE_M        0.39f       // m

// ── PID 初期値 (th_ws/esp32/src/config.h と同一・実機未検証) ──
#define PID_KP_RIGHT        80.0f
#define PID_KI_RIGHT        30.0f
#define PID_KD_RIGHT         8.0f

#define PID_KP_LEFT        100.0f
#define PID_KI_LEFT         30.0f
#define PID_KD_LEFT          8.0f

#define PID_OUT_MIN        -255.0f
#define PID_OUT_MAX         255.0f
#define PID_ITERM_MAX       100.0f

// ── 制御周期 / シリアル ──────────────────────────────────────
#define CTRL_PERIOD_MS      100
#define SERIAL_BAUD         115200
