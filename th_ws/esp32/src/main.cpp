// ============================================================
// TH System — ESP32 メインファームウェア
// micro-ROS (Serial transport) + PID 速度制御
// ============================================================
#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/bool.h>

#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"

// ── micro-ROS オブジェクト ────────────────────────────────────
static rcl_node_t            node;
static rclc_support_t        support;
static rcl_allocator_t       allocator;
static rclc_executor_t       executor;

// Sub: /wheel_cmd  (Float64MultiArray: [left_speed, right_speed] m/s)
static rcl_subscription_t    sub_wheel_cmd;
static std_msgs__msg__Float64MultiArray  msg_wheel_cmd;
static double                wheel_cmd_data[2] = {0.0, 0.0};

// Pub: /wheel_feedback  (Float64MultiArray: [left_speed, right_speed] m/s)
static rcl_publisher_t       pub_wheel_feedback;
static std_msgs__msg__Float64MultiArray  msg_feedback;
static double                feedback_data[2] = {0.0, 0.0};

// Pub: /estop_hw  (Bool: true = E-Stop 発動中)
static rcl_publisher_t       pub_estop_hw;
static std_msgs__msg__Bool   msg_estop;

// ── タイマー ─────────────────────────────────────────────────
static rcl_timer_t           ctrl_timer;
static unsigned long         last_cmd_ms = 0;   // ウォッチドッグ用
static unsigned long         last_ctrl_ms = 0;

// ── PID ──────────────────────────────────────────────────────
static PID pidRight(PID_KP_RIGHT, PID_KI_RIGHT, PID_KD_RIGHT,
                    PID_OUT_MIN,  PID_OUT_MAX,   PID_ITERM_MAX);
static PID pidLeft (PID_KP_LEFT,  PID_KI_LEFT,  PID_KD_LEFT,
                    PID_OUT_MIN,  PID_OUT_MAX,   PID_ITERM_MAX);

// ── 状態 ─────────────────────────────────────────────────────
enum class AgentState { SEARCHING, CONNECTED };
static AgentState agentState = AgentState::SEARCHING;

// 目標速度 (m/s)
static volatile float targetLeft  = 0.0f;
static volatile float targetRight = 0.0f;

// ── デバッグ: RCCHECK/RCSOFTCHECK がエラーを握りつぶすとサイレント障害が
//    追えないため、オンボードLED点滅で可視化する。
//    (UART0はmicro-ROS通信専用でSerial.printを使えないための代替策)
//    点滅回数 = rcl_ret_t の下1桁 (0は10回扱い)。原因切り分けが終わったら
//    元の (void)rc; 版に戻して良い。
#define DEBUG_LED_PIN 2
static void blinkRclError(rcl_ret_t code) {
    static bool ledReady = false;
    if (!ledReady) { pinMode(DEBUG_LED_PIN, OUTPUT); ledReady = true; }
    int n = (int)(code % 10);
    if (n <= 0) n = 10;
    for (int i = 0; i < n; i++) {
        digitalWrite(DEBUG_LED_PIN, HIGH); delay(120);
        digitalWrite(DEBUG_LED_PIN, LOW);  delay(120);
    }
    delay(700);
}

// ── マクロ: エラー検出時に LED 点滅 ──────────────────────────
#define RCCHECK(fn)     { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) blinkRclError(rc); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) blinkRclError(rc); }

// ── wheel_cmd コールバック ────────────────────────────────────
static void cbWheelCmd(const void* msgin) {
    const auto* m = (const std_msgs__msg__Float64MultiArray*)msgin;
    if (m->data.size >= 2) {
        targetLeft  = (float)m->data.data[0];
        targetRight = (float)m->data.data[1];
    }
    last_cmd_ms = millis();
}

// ── 制御タイマーコールバック ─────────────────────────────────
static void cbCtrlTimer(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
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
    bool watchdogTripped = (millis() - last_cmd_ms) > WATCHDOG_MS;

    if (estopActive || watchdogTripped) {
        Motor::stopAll();
        pidRight.reset();
        pidLeft.reset();
        targetLeft  = 0.0f;
        targetRight = 0.0f;
    } else {
        // ── エンコーダから実速度を計算 ──────────────────────────
        float distPerCount = (2.0f * M_PI * WHEEL_RADIUS_M) / ENC_COUNTS_PER_REV;
        long  cntL = Encoder::readAndResetLeft();
        long  cntR = Encoder::readAndResetRight();
        float velL = (float)cntL * distPerCount / dt;   // m/s
        float velR = (float)cntR * distPerCount / dt;   // m/s

        // ── PID 速度制御 → PWM 正規化 (-1.0 〜 1.0) ────────────
        float outR = pidRight.compute(targetRight, velR, dt) / 255.0f;
        float outL = pidLeft.compute(targetLeft,  velL, dt) / 255.0f;

        Motor::setRight(outR);
        Motor::setLeft(outL);

        // ── フィードバック発行 ───────────────────────────────────
        feedback_data[0] = velL;
        feedback_data[1] = velR;
        msg_feedback.data.data = feedback_data;
        msg_feedback.data.size = 2;
        RCSOFTCHECK(rcl_publish(&pub_wheel_feedback, &msg_feedback, nullptr));
    }

    // E-Stop 状態を発行 (毎周期)
    msg_estop.data = estopActive;
    RCSOFTCHECK(rcl_publish(&pub_estop_hw, &msg_estop, nullptr));
}

// ── micro-ROS エンティティ生成 ───────────────────────────────
static bool createEntities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

    RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));

    // Subscriber: /wheel_cmd
    msg_wheel_cmd.data.data     = wheel_cmd_data;
    msg_wheel_cmd.data.size     = 2;
    msg_wheel_cmd.data.capacity = 2;
    RCCHECK(rclc_subscription_init_default(
        &sub_wheel_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "wheel_cmd"));

    // Publisher: /wheel_feedback
    msg_feedback.data.data     = feedback_data;
    msg_feedback.data.size     = 2;
    msg_feedback.data.capacity = 2;
    RCCHECK(rclc_publisher_init_default(
        &pub_wheel_feedback, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "wheel_feedback"));

    // Publisher: /estop_hw
    RCCHECK(rclc_publisher_init_default(
        &pub_estop_hw, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "estop_hw"));

    // Timer: 制御ループ
    RCCHECK(rclc_timer_init_default(
        &ctrl_timer, &support,
        RCL_MS_TO_NS(CTRL_PERIOD_MS),
        cbCtrlTimer));

    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_wheel_cmd, &msg_wheel_cmd, cbWheelCmd, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &ctrl_timer));

    last_cmd_ms  = millis();
    last_ctrl_ms = millis();
    return true;
}

// ── micro-ROS エンティティ破棄 ───────────────────────────────
static void destroyEntities() {
    rcl_subscription_fini(&sub_wheel_cmd,    &node);
    rcl_publisher_fini(&pub_wheel_feedback,  &node);
    rcl_publisher_fini(&pub_estop_hw,        &node);
    rcl_timer_fini(&ctrl_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ── setup / loop ─────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);

    // E-Stop ピン (入力専用 GPIO34 は内蔵プルアップなし)
    pinMode(ESTOP_GPIO, INPUT);

    Encoder::init();
    Motor::init();
    Motor::stopAll();

    // micro-ROS: シリアルトランスポート (micro_ros_arduino 2.x API)
    set_microros_transports();
    delay(500);
}

void loop() {
    switch (agentState) {

    case AgentState::SEARCHING:
        // Agent が応答するまでポーリング
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
            createEntities();
            agentState = AgentState::CONNECTED;
        } else {
            Motor::stopAll();
            delay(100);
        }
        break;

    case AgentState::CONNECTED:
        // Agent 生存確認 (100ms タイムアウト × 1 回)
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
            // Agent 切断 → エンティティ破棄してリセット待ち
            destroyEntities();
            Motor::stopAll();
            pidRight.reset();
            pidLeft.reset();
            targetLeft  = 0.0f;
            targetRight = 0.0f;
            agentState  = AgentState::SEARCHING;
        } else {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            // rclc executor がタイマーを wait set に含めない場合の保険:
            // cbCtrlTimer 冒頭で last_ctrl_ms を更新するため二重呼び出しは自動防止される
            if ((millis() - last_ctrl_ms) >= CTRL_PERIOD_MS) {
                cbCtrlTimer(nullptr, 0LL);
            }
        }
        break;
    }
}
