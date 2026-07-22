// ============================================================
// TH System — ESP32 メインファームウェア
// WiFi + WebSocket (WsLink) + PID 速度制御
// ============================================================
#include <Arduino.h>

#include "config.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "ws_link.h"

// ── タイマー ─────────────────────────────────────────────────
static unsigned long last_cmd_ms = 0;   // ウォッチドッグ用
static unsigned long last_ctrl_ms = 0;

// ── IMU (DSR1603/BNO055) ────────────────────────────────────
// 未実装の個体では init() が false を返し、以降 IMU_DATA 送信をスキップする。
static bool imuPresent = false;

// ── PID ──────────────────────────────────────────────────────
static PID pidRight(PID_KP_RIGHT, PID_KI_RIGHT, PID_KD_RIGHT,
                    PID_OUT_MIN,  PID_OUT_MAX,   PID_ITERM_MAX, PID_KFF);
static PID pidLeft (PID_KP_LEFT,  PID_KI_LEFT,  PID_KD_LEFT,
                    PID_OUT_MIN,  PID_OUT_MAX,   PID_ITERM_MAX, PID_KFF);

// 目標速度 (m/s) — wheel_cmd で受信した最終目標値
static volatile float targetLeft  = 0.0f;
static volatile float targetRight = 0.0f;

// PIDに渡す目標値 (m/s) — TARGET_RAMP_ACCEL_MPS2 で加速度制限した現在値。
// targetLeft/Right へのステップ変化をそのままPIDに渡すと比例項が急崩壊して
// 起動直後に振動するため、ここで滑らかに追従させる。
static float rampLeft  = 0.0f;
static float rampRight = 0.0f;

// value を target に向かって最大 maxStep だけ動かす (加減速どちらも対応)
static float rampToward(float value, float target, float maxStep) {
    if (value < target) return min(value + maxStep, target);
    if (value > target) return max(value - maxStep, target);
    return value;
}

// ── WHEEL_CMD 受信コールバック ────────────────────────────────
static void onWheelCmd(float left, float right) {
    targetLeft  = left;
    targetRight = right;
    last_cmd_ms = millis();
    // 通信テスト用ログ。開発ボード単体での書き込み・通信確認に使う
    // (WsLink::sendEstopHw は毎周期送信されるので、モーター未接続でも
    //  ws_test_server.py 側でこの受信ログと突き合わせて双方向通信を確認できる)
    Serial.printf("[WHEEL_CMD] left=%.3f right=%.3f\n", left, right);
}

// ── WS切断時コールバック: 即座に停止 (ウォッチドッグを待たない) ──
static void onWsDisconnect() {
    Motor::stopAll();
    pidRight.reset();
    pidLeft.reset();
    targetLeft  = 0.0f;
    targetRight = 0.0f;
    rampLeft    = 0.0f;
    rampRight   = 0.0f;
}

// ── 制御タイマーコールバック ─────────────────────────────────
// E-Stop判定・ウォッチドッグ判定・PID・エンコーダ処理は通信方式に依存しない。
static void cbCtrlTimer() {
    unsigned long now = millis();
    float dt = (now - last_ctrl_ms) / 1000.0f;
    last_ctrl_ms = now;
    if (dt <= 0.0f || dt > 1.0f) dt = (float)CTRL_PERIOD_MS / 1000.0f;

    // E-Stop チェック
    bool estopActive = ESTOP_LOW_ACTIVE
                       ? (digitalRead(ESTOP_GPIO) == LOW)
                       : (digitalRead(ESTOP_GPIO) == HIGH);
#ifdef ESTOP_BENCH_TEST_BYPASS
    estopActive = false;
#endif

    // ウォッチドッグ: 最後の wheel_cmd 受信から WATCHDOG_MS 超過でゼロ
    // (WS接続状態に関わらずローカルに独立して動作する)
    bool watchdogTripped = (millis() - last_cmd_ms) > WATCHDOG_MS;

    // ── エンコーダから実速度を計算 ──────────────────────────
    // 停止中(E-Stop/ウォッチドッグ)も読み続ける: 手押し等で車輪が回った
    // 場合もオドメトリに反映させるため。
    float distPerCount = (2.0f * M_PI * WHEEL_RADIUS_M) / ENC_COUNTS_PER_REV;
    long  cntL = Encoder::readAndResetLeft();
    long  cntR = Encoder::readAndResetRight();
    float velL = (float)cntL * distPerCount / dt;   // m/s
    float velR = (float)cntR * distPerCount / dt;   // m/s

    float outR = 0.0f, outL = 0.0f;
    if (estopActive || watchdogTripped) {
        Motor::stopAll();
        pidRight.reset();
        pidLeft.reset();
        targetLeft  = 0.0f;
        targetRight = 0.0f;
        rampLeft    = 0.0f;
        rampRight   = 0.0f;
    } else {
        // ── 目標速度を加速度制限してランプ ──────────────────────
        float rampStep = TARGET_RAMP_ACCEL_MPS2 * dt;
        rampLeft  = rampToward(rampLeft,  targetLeft,  rampStep);
        rampRight = rampToward(rampRight, targetRight, rampStep);

        // ── PID 速度制御 → PWM 正規化 (-1.0 〜 1.0) ────────────
        outR = pidRight.compute(rampRight, velR, dt) / 255.0f;
        outL = pidLeft.compute(rampLeft,  velL, dt) / 255.0f;

        Motor::setRight(outR);
        Motor::setLeft(outL);
    }

    // ── フィードバック送信 (毎周期・停止中も送る) ────────────────
    // これを停止中に止めると、待機中(cmd_vel なし)に ROS 側の
    // safety_monitor が ESP32_DISCONNECTED を誤検知し、esp32_bridge の
    // odom/TF も途絶して Nav2 が動けなくなる(実機検証で判明)。
    // 「切断検知」は通信途絶そのもので判定されるべきで、フィードバック送信を
    // 止めることを安全機構にしてはいけない。
    WsLink::sendWheelFeedback(velL, velR);

    // ── IMU 送信 (未実装個体はスキップ) ───────────────────────────
    if (imuPresent) {
        float qw, qx, qy, qz, wx, wy, wz, ax, ay, az;
        uint8_t calibStatus;
        Imu::read(qw, qx, qy, qz, wx, wy, wz, ax, ay, az, calibStatus);
        WsLink::sendImuData(qw, qx, qy, qz, wx, wy, wz, ax, ay, az, calibStatus);
    }

    // デバッグ用一時ログ: 原因切り分け後に削除すること
    static int dbgCounter = 0;
    if (++dbgCounter >= 5) {
        dbgCounter = 0;
        Serial.printf("[DBG] estop=%d watchdog=%d tgtL=%.2f tgtR=%.2f rmpL=%.2f rmpR=%.2f velL=%.3f velR=%.3f outL=%.3f outR=%.3f\n",
                      estopActive, watchdogTripped, targetLeft, targetRight, rampLeft, rampRight, velL, velR, outL, outR);
    }

    // E-Stop 状態を送信 (毎周期)
    WsLink::sendEstopHw(estopActive);
}

// ── setup / loop ─────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(300);  // シリアルモニタが接続待ちの間にログを取りこぼさないための猶予
    Serial.println();
    Serial.println("================================================");
    Serial.println("TH System ESP32 Firmware (WebSocket)");
    Serial.print("Build: "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);
    Serial.println("書き込み確認: このバナーが表示されていればフラッシュ成功");
    Serial.println("================================================");

    // E-Stop ピン (入力専用 GPIO34 は内蔵プルアップなし)
    pinMode(ESTOP_GPIO, INPUT);

    Encoder::init();
    Motor::init();
    Motor::stopAll();

    imuPresent = Imu::init();
    Serial.printf("[IMU] DSR1603(BNO055) %s\n",
                  imuPresent ? "検出・初期化OK" : "未検出(IMU_DATA送信をスキップ)");

    WsLink::onWheelCmd(onWheelCmd);
    WsLink::onDisconnect(onWsDisconnect);
    WsLink::init();

    last_cmd_ms  = millis();
    last_ctrl_ms = millis();
}

void loop() {
    WsLink::loop();  // 非ブロッキング: WiFi/WebSocket 送受信・自動再接続

    if ((millis() - last_ctrl_ms) >= CTRL_PERIOD_MS) {
        cbCtrlTimer();
    }
}
