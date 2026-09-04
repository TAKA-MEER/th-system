// ============================================================
// safety_monitor — 安全監視ノード (C++)
//
// DetailedDesign-wp2.md `WP-SAFE-01`。判定ロジックは
// th_safety/safety_monitor_core.hpp（ROS2 非依存）に置き、このファイルは
// ROS2 との配線（sub/pub/timer/service）だけを行う
// （CLAUDE.md「追従ロジックの二層構造」と同じ思想）。
//
// 監視対象（enabled_targets に入っているものだけ。F-5・O-7）:
//   lidar        /scan タイムアウト                          → LIDAR_LOST (RECOVERABLE)
//   esp32        /esp32/wheel_feedback タイムアウト          → ESP32_DISCONNECTED (RECOVERABLE)
//   person       /person/targets タイムアウト                → PERSON_TRACKER_LOST (RECOVERABLE)
//   limiter      /safety/limiter_status タイムアウト         → LIMITER_DEAD (CRITICAL)
//   mux          /cmd_vel_muxed ⇄ /cmd_vel の双方向途絶      → MUX_DEAD (CRITICAL)
//   runaway      /cmd_vel と /esp32/wheel_feedback の乖離    → DRIVE_RUNAWAY (CRITICAL)
//   state        /system/state タイムアウト・不整合          → STATE_INCONSISTENT (CRITICAL)
//   firmware     /safety/firmware_flags の bypass_active ビット → ESTOP_BYPASS_ACTIVE (CRITICAL)
//
// UI 非常停止（estop_ui）と物理 E-Stop（estop_hw）は enabled_targets の対象外
// （常時監視。ノード自体の中核機能のため段階に依存しない）。
//
// 出力:
//   /safety/estop       (Bool)         10Hz — hw || ui ラッチ
//   /safety/fault_lock  (Bool)         10Hz — LIDAR_LOST || ESP32_DISCONNECTED || severity==CRITICAL (F-2)
//   /safety/fault       (FaultStatus)  変化時
//   /safety/link_quality (LinkQuality) 1Hz×3（esp32/lidar/ui。WP-SAFE-00 で予定されていたが
//                                       未配線だったためこのパケットで統合する）
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <th_system_msgs/msg/fault_status.hpp>
#include <th_system_msgs/msg/wheel_feedback.hpp>
#include <th_system_msgs/msg/person_targets.hpp>
#include <th_system_msgs/msg/limiter_status.hpp>
#include <th_system_msgs/msg/system_state.hpp>
#include <th_system_msgs/msg/link_quality.hpp>

#include "th_safety/safety_monitor_core.hpp"
#include "th_safety/link_quality_core.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

bool isNonZeroTwist(const geometry_msgs::msg::Twist& t, double eps = 1e-4) {
  return std::fabs(t.linear.x) > eps || std::fabs(t.linear.y) > eps ||
         std::fabs(t.angular.z) > eps;
}

}  // namespace

class SafetyMonitor : public rclcpp::Node {
public:
    SafetyMonitor() : Node("safety_monitor"),
        runaway_hold_(0.5) {
        // ── パラメータ ──────────────────────────────────────
        declare_parameter("lidar_timeout_ms",     2000);
        declare_parameter("esp32_timeout_ms",     2000);
        declare_parameter("person_timeout_ms",    2500);
        declare_parameter("limiter_dead_ms",      250);
        declare_parameter("mux_dead_ms",          500);
        declare_parameter("state_stale_ms",       1500);
        declare_parameter("runaway_ratio",        1.5);
        declare_parameter("runaway_hold_ms",      500);
        // WS-9O (2026-09-04): 重大フォルトの発火に課す保持時間。
        // 監視ループ (check_period_ms) 自身が一時的に遅れると、複数の入力が同じ
        // 判定周期で同時にタイムアウト超過に見える。実機ログで ESP32_DISCONNECTED と
        // LIMITER_DEAD が同じ 1ms に発火し 26ms 後に両方解除された。フォルトは edge で
        // publish されるので、その 26ms でも active=true は state_manager に届き
        // C-06a (ガード無し) で必ず ESTOP に落ちる。条件が継続したときだけ報告する。
        // タイムアウト値そのもの (limiter_dead_ms 等) は変えない。
        declare_parameter("critical_fault_hold_ms", 300);
        declare_parameter("runaway_zero_threshold", 0.02);
        declare_parameter("estop_ui_lease_ms",    1500);
        declare_parameter("link_quality_window_sec", 30);
        declare_parameter("check_period_ms",      100);
        declare_parameter("startup_grace_sec",    3);
        // 未受信（DDS マッチング未了）を「途絶」と誤検知しないための上限。
        // startup_grace_sec は変えない。これを超えても 1 通も来なければ検知する。
        declare_parameter("startup_deadline_sec", 15);
        // O-7: 既定は空（何も監視しない）。段階ごとに launch から渡す。
        declare_parameter("enabled_targets", std::vector<std::string>{});

        lidar_timeout_  = std::chrono::milliseconds(get_parameter("lidar_timeout_ms").as_int());
        esp32_timeout_  = std::chrono::milliseconds(get_parameter("esp32_timeout_ms").as_int());
        person_timeout_ = std::chrono::milliseconds(get_parameter("person_timeout_ms").as_int());
        limiter_dead_   = std::chrono::milliseconds(get_parameter("limiter_dead_ms").as_int());
        mux_dead_       = std::chrono::milliseconds(get_parameter("mux_dead_ms").as_int());
        state_stale_    = std::chrono::milliseconds(get_parameter("state_stale_ms").as_int());
        runaway_ratio_  = get_parameter("runaway_ratio").as_double();
        runaway_zero_threshold_ = get_parameter("runaway_zero_threshold").as_double();
        runaway_hold_   = th_safety::HoldTimer(get_parameter("runaway_hold_ms").as_int() / 1000.0);
        {
            const double hold = get_parameter("critical_fault_hold_ms").as_int() / 1000.0;
            limiter_dead_hold_  = th_safety::HoldTimer(hold);
            mux_dead_hold_      = th_safety::HoldTimer(hold);
            state_inconsist_hold_ = th_safety::HoldTimer(hold);
        }
        estop_ui_lease_sec_ = get_parameter("estop_ui_lease_ms").as_int() / 1000.0;
        link_quality_window_sec_ = get_parameter("link_quality_window_sec").as_int();
        enabled_targets_ = get_parameter("enabled_targets").as_string_array();
        startup_deadline_sec_ = get_parameter("startup_deadline_sec").as_int();

        // ── Publishers ─────────────────────────────────────
        pub_estop_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/estop", rclcpp::QoS(1).reliable());
        pub_fault_ = create_publisher<th_system_msgs::msg::FaultStatus>(
            "/safety/fault", rclcpp::QoS(5).reliable());
        pub_fault_lock_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/fault_lock", rclcpp::QoS(1).reliable());
        pub_link_quality_ = create_publisher<th_system_msgs::msg::LinkQuality>(
            "/safety/link_quality", rclcpp::QoS(1).best_effort());

        // ── Subscribers ────────────────────────────────────
        sub_estop_hw_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/estop_hw", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                hw_estop_active_ = msg->data;
            });

        // UI 非常停止（旧 /safety/tablet_estop から改名。§6.3: 押下側にラッチする）
        sub_estop_ui_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/estop_ui", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                const double t = now().seconds();
                if (msg->data) {
                    ui_latch_.on_true(t);
                } else {
                    ui_latch_.on_false();
                }
                ui_gap_.push(t);
            });

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr) {
                last_scan_time_ = now();
                lidar_alive_    = true;
                lidar_gap_.push(now().seconds());
            });

        // 試験員追従（PersonStatus → PersonTargets。reuse.md §2.3）
        sub_person_ = create_subscription<th_system_msgs::msg::PersonTargets>(
            "/person/targets", 10,
            [this](const th_system_msgs::msg::PersonTargets::SharedPtr) {
                last_person_time_ = now();
                person_alive_     = true;
            });

        sub_wheel_fb_ = create_subscription<th_system_msgs::msg::WheelFeedback>(
            "/esp32/wheel_feedback", 10,
            [this](const th_system_msgs::msg::WheelFeedback::SharedPtr msg) {
                last_esp32_time_ = now();
                esp32_alive_     = true;
                last_wheel_left_  = msg->left_speed;
                last_wheel_right_ = msg->right_speed;
                esp32_gap_.push(now().seconds());
            });

        // §4.1 新設: limiter_status（重大。WP-SAFE-03 が実装するまで publisher 無し。
        // enabled_targets に "limiter" を入れるまでは監視しない — F-5・O-7）
        sub_limiter_status_ = create_subscription<th_system_msgs::msg::LimiterStatus>(
            "/safety/limiter_status", rclcpp::QoS(1).best_effort(),
            [this](const th_system_msgs::msg::LimiterStatus::SharedPtr) {
                last_limiter_time_ = now();
                limiter_alive_     = true;
            });

        // §4.1 新設: MUX_DEAD の入力（/cmd_vel_muxed は obstacle_limiter=WP-SAFE-03 が
        // 出力を消費する側の topic。twist_mux の remap 先が変わるまで publisher 無し。
        // enabled_targets に "mux" を入れるまでは監視しない — F-5・O-7）
        sub_cmd_vel_muxed_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_muxed", rclcpp::QoS(1).reliable(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_muxed_time_ = now();
                muxed_alive_     = true;
                muxed_last_nonzero_ = isNonZeroTwist(*msg);
            });

        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(1).reliable(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_cmd_time_ = now();
                cmd_alive_     = true;
                cmd_last_nonzero_ = isNonZeroTwist(*msg);
                last_cmd_linear_x_ = msg->linear.x;
            });

        // §4.1 新設: STATE_INCONSISTENT。last_mode_/last_state_ は enabled_targets
        // に関わらず常に更新する（clear_estop_ui のログに使うため）。
        sub_system_state_ = create_subscription<th_system_msgs::msg::SystemState>(
            "/system/state", rclcpp::QoS(1).reliable().transient_local(),
            [this](const th_system_msgs::msg::SystemState::SharedPtr msg) {
                last_state_time_ = now();
                state_alive_     = true;
                last_mode_  = msg->mode;
                last_state_ = msg->state;
            });

        // DEBT-1: ESTOP_HW バイパス検出（safety.md §11.1）
        sub_firmware_flags_ = create_subscription<std_msgs::msg::UInt8>(
            "/safety/firmware_flags", rclcpp::QoS(1).transient_local().reliable(),
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                // bit0 = bypass_active。旧ファーム(FIRMWARE_FLAGS_UNKNOWN=0xFF)も
                // bit0 が立っているため、この 1 本のビット判定だけで両方が
                // 「バイパスされているかもしれない → 重大扱い」の安全側に倒れる
                // (ws_protocol.py の FIRMWARE_FLAGS_UNKNOWN=0xFF と一致)。
                firmware_bypass_active_ = (msg->data & 0x01) != 0;
                firmware_flags_received_ = true;
            });

        // ── clear_estop_ui（§3.2・§6.3.1・N-4） ─────────────
        srv_clear_estop_ui_ = create_service<std_srvs::srv::Trigger>(
            "/safety/clear_estop_ui",
            std::bind(&SafetyMonitor::handleClearEstopUi, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ── 監視タイマー ────────────────────────────────────
        int period = get_parameter("check_period_ms").as_int();
        check_period_sec_ = period / 1000.0;
        check_timer_ = create_wall_timer(
            std::chrono::milliseconds(period),
            std::bind(&SafetyMonitor::checkHealth, this));

        // 1Hz のリンク品質 publish（WP-SAFE-00。Q-1: 判定には使わない診断用）
        link_quality_timer_ = create_wall_timer(
            1s, std::bind(&SafetyMonitor::publishLinkQuality, this));

        rclcpp::Time t0 = now();
        node_start_time_   = t0;
        last_scan_time_    = t0;
        last_person_time_  = t0;
        last_esp32_time_   = t0;
        last_limiter_time_ = t0;
        last_muxed_time_   = t0;
        last_cmd_time_     = t0;
        last_state_time_   = t0;

        int grace_sec = get_parameter("startup_grace_sec").as_int();
        startup_grace_end_ = t0 + rclcpp::Duration(grace_sec, 0);

        RCLCPP_INFO(get_logger(), "safety_monitor 起動 (enabled_targets=%zu 件)",
                    enabled_targets_.size());
    }

private:
    bool targetEnabled(const std::string& target) const {
        return th_safety::is_target_enabled(target, enabled_targets_);
    }

    void checkHealth() {
        rclcpp::Time t = now();
        bool in_grace = (t < startup_grace_end_);

        // ── E-Stop 集約（estop_hw || UI ラッチ。enabled_targets の対象外） ──
        bool estop = hw_estop_active_ || ui_latch_.latched();
        std_msgs::msg::Bool estop_msg;
        estop_msg.data = estop;
        pub_estop_->publish(estop_msg);

        if (estop && !prev_estop_) {
            RCLCPP_WARN(get_logger(), "E-Stop 発動 (hw=%d ui=%d)",
                        hw_estop_active_, ui_latch_.latched());
        } else if (!estop && prev_estop_) {
            RCLCPP_INFO(get_logger(), "E-Stop 解除");
        }
        prev_estop_ = estop;

        if (!in_grace) {
            // ── 回復可能フォルト ──────────────────────────
            if (targetEnabled("lidar")) {
                checkTimeout("LIDAR_LOST", last_scan_time_, lidar_timeout_, t, lidar_alive_);
            }
            if (targetEnabled("esp32")) {
                checkTimeout("ESP32_DISCONNECTED", last_esp32_time_, esp32_timeout_, t, esp32_alive_);
            }
            if (targetEnabled("person")) {
                checkTimeout("PERSON_TRACKER_LOST", last_person_time_, person_timeout_, t, person_alive_);
            }
            // UI_DISCONNECTED は enabled_targets の対象外（常時監視。§4.2）
            updateFaultState("UI_DISCONNECTED",
                              ui_latch_.is_disconnected(t.seconds(), estop_ui_lease_sec_));

            // ── 重大フォルト（§4.1） ──────────────────────
            if (targetEnabled("limiter")) {
                // WS-9O: 単発の誤検知よけに保持時間を課す（checkTimeout をそのまま
                // 使うと 1 周期の遅れで ESTOP に落ちる）。
                bool cond = computeTimeoutFault(last_limiter_time_, limiter_dead_, t,
                                                 limiter_alive_);
                updateFaultState("LIMITER_DEAD",
                                  limiter_dead_hold_.update(cond, check_period_sec_));
            }
            if (targetEnabled("mux")) {
                // MUX_DEAD は checkTimeout を通らない独自判定だが、未受信の
                // 誤検知回避は同じ規則にそろえる（is_timeout_fault 経由）。
                double since_start   = (t - node_start_time_).seconds();
                double mux_dead_sec  = std::chrono::duration<double>(mux_dead_).count();
                bool muxed_stale = th_safety::is_timeout_fault(
                    muxed_alive_, (t - last_muxed_time_).seconds(), mux_dead_sec,
                    since_start, startup_deadline_sec_);
                bool cmd_stale = th_safety::is_timeout_fault(
                    cmd_alive_, (t - last_cmd_time_).seconds(), mux_dead_sec,
                    since_start, startup_deadline_sec_);
                bool mux_dead = th_safety::detect_mux_dead(
                    muxed_stale, muxed_last_nonzero_, cmd_stale, cmd_last_nonzero_);
                // WS-9O: 単発の誤検知よけ
                updateFaultState("MUX_DEAD",
                                  mux_dead_hold_.update(mux_dead, check_period_sec_));
            }
            if (targetEnabled("runaway")) {
                double feedback_abs = std::fabs((last_wheel_left_ + last_wheel_right_) / 2.0);
                double cmd_abs = std::fabs(last_cmd_linear_x_);
                bool condition = th_safety::is_runaway_condition(
                    cmd_abs, feedback_abs, runaway_ratio_, runaway_zero_threshold_);
                bool runaway = runaway_hold_.update(condition, check_period_sec_);
                updateFaultState("DRIVE_RUNAWAY", runaway);
            }
            if (targetEnabled("state")) {
                bool state_stale = (t - last_state_time_) > rclcpp::Duration(state_stale_);
                bool inconsistent = th_safety::detect_state_inconsistent(
                    state_stale, last_mode_, last_state_, th_safety::default_mode_states());
                // WS-9O: 単発の誤検知よけ
                updateFaultState("STATE_INCONSISTENT",
                                  state_inconsist_hold_.update(inconsistent, check_period_sec_));
            }
            if (targetEnabled("firmware")) {
                updateFaultState("ESTOP_BYPASS_ACTIVE",
                                  firmware_flags_received_ && firmware_bypass_active_);
            }
        }

        // F-1: 沈黙禁止。状態変化の有無にかかわらず毎周期発行する。
        publishLock();
    }

    // 通信途絶タイムアウトによるフォルト判定（回復可能・重大 共通）。
    // ever_received=false（起動直後、まだ 1 通も来ていない）の間は
    // startup_deadline_sec を超えるまで途絶扱いしない（is_timeout_fault）。
    bool computeTimeoutFault(const rclcpp::Time& last_time,
                             const std::chrono::milliseconds& timeout,
                             const rclcpp::Time& now_t, bool ever_received) {
        double since_last  = (now_t - last_time).seconds();
        double since_start = (now_t - node_start_time_).seconds();
        double timeout_sec = std::chrono::duration<double>(timeout).count();
        return th_safety::is_timeout_fault(
            ever_received, since_last, timeout_sec, since_start, startup_deadline_sec_);
    }

    void checkTimeout(const std::string& fault_type, const rclcpp::Time& last_time,
                      const std::chrono::milliseconds& timeout, const rclcpp::Time& now_t,
                      bool ever_received) {
        updateFaultState(fault_type,
                          computeTimeoutFault(last_time, timeout, now_t, ever_received));
    }

    // フォルトの edge 検出 + publish。active_faults_ を更新する（fault_lock の合成に使う）。
    void updateFaultState(const std::string& fault_type, bool faulted) {
        bool was_fault = active_faults_.count(fault_type) != 0;
        if (faulted && !was_fault) {
            active_faults_.insert(fault_type);
            RCLCPP_ERROR(get_logger(), "[FAULT] %s (severity=%s)", fault_type.c_str(),
                         th_safety::severity_to_string(th_safety::classify_severity(fault_type)));
            publishFault(true, fault_type);
        } else if (!faulted && was_fault) {
            active_faults_.erase(fault_type);
            RCLCPP_INFO(get_logger(), "[FAULT CLEARED] %s", fault_type.c_str());
            publishFault(false, "NONE");
        }
    }

    void publishFault(bool active, const std::string& type) {
        th_system_msgs::msg::FaultStatus msg;
        msg.header.stamp = now();
        msg.active     = active;
        msg.fault_type = type;
        msg.severity   = th_safety::severity_to_string(
            th_safety::classify_severity(active ? type : "NONE"));
        pub_fault_->publish(msg);
        publishLock();
    }

    // §5.2.1: /safety/fault_lock = LIDAR_LOST || ESP32_DISCONNECTED || severity==CRITICAL
    void publishLock() {
        bool lidar_lost_active = active_faults_.count("LIDAR_LOST") != 0;
        bool esp32_disconnected_active = active_faults_.count("ESP32_DISCONNECTED") != 0;
        bool any_critical = false;
        for (const auto& ft : active_faults_) {
            if (th_safety::classify_severity(ft) == th_safety::Severity::CRITICAL) {
                any_critical = true;
                break;
            }
        }
        std_msgs::msg::Bool lock_msg;
        lock_msg.data = th_safety::compute_fault_lock(
            lidar_lost_active, esp32_disconnected_active, any_critical);
        pub_fault_lock_->publish(lock_msg);
    }

    bool hasCriticalFaultActive() const {
        for (const auto& ft : active_faults_) {
            if (th_safety::classify_severity(ft) == th_safety::Severity::CRITICAL) {
                return true;
            }
        }
        return false;
    }

    // §3.2・§6.3.1（N-4）: UI に依存しない非常停止解除経路
    void handleClearEstopUi(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        auto decision = th_safety::decide_clear_estop_ui(hw_estop_active_, hasCriticalFaultActive());
        response->success = decision.success;
        response->message = decision.message;

        // 受理・拒否のどちらも必ずログに残す（who=cli・時刻・そのときの mode/state）
        if (decision.success) {
            ui_latch_.on_false();
            RCLCPP_INFO(get_logger(),
                "[clear_estop_ui] who=cli success=true mode=%s state=%s message=%s",
                last_mode_.c_str(), last_state_.c_str(), decision.message.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "[clear_estop_ui] who=cli success=false mode=%s state=%s message=%s",
                last_mode_.c_str(), last_state_.c_str(), decision.message.c_str());
        }
    }

    void publishLinkQuality() {
        double t = now().seconds();
        publishOneLinkQuality("esp32", esp32_gap_, t);
        publishOneLinkQuality("lidar", lidar_gap_, t);
        publishOneLinkQuality("ui",    ui_gap_,    t);
    }

    void publishOneLinkQuality(const std::string& link, const th_safety::GapTracker& tracker,
                                double now_sec) {
        auto q = tracker.compute(now_sec, static_cast<double>(link_quality_window_sec_));
        th_system_msgs::msg::LinkQuality msg;
        msg.header.stamp = now();
        msg.link = link;
        msg.p50_ms = static_cast<float>(q.p50_ms);
        msg.p99_ms = static_cast<float>(q.p99_ms);
        msg.max_ms = static_cast<float>(q.max_ms);
        msg.window_sec = q.window_sec;
        pub_link_quality_->publish(msg);
    }

    // ── Publishers ────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_estop_;
    rclcpp::Publisher<th_system_msgs::msg::FaultStatus>::SharedPtr   pub_fault_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_fault_lock_;
    rclcpp::Publisher<th_system_msgs::msg::LinkQuality>::SharedPtr   pub_link_quality_;

    // ── Subscribers ──────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             sub_estop_hw_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             sub_estop_ui_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     sub_scan_;
    rclcpp::Subscription<th_system_msgs::msg::PersonTargets>::SharedPtr sub_person_;
    rclcpp::Subscription<th_system_msgs::msg::WheelFeedback>::SharedPtr sub_wheel_fb_;
    rclcpp::Subscription<th_system_msgs::msg::LimiterStatus>::SharedPtr sub_limiter_status_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       sub_cmd_vel_muxed_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       sub_cmd_vel_;
    rclcpp::Subscription<th_system_msgs::msg::SystemState>::SharedPtr sub_system_state_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr            sub_firmware_flags_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr               srv_clear_estop_ui_;

    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::TimerBase::SharedPtr link_quality_timer_;

    // ── 状態 ─────────────────────────────────────────────
    bool hw_estop_active_ = false;
    bool prev_estop_      = false;
    th_safety::UiEstopLatch ui_latch_;

    rclcpp::Time node_start_time_;
    rclcpp::Time last_scan_time_;
    rclcpp::Time last_person_time_;
    rclcpp::Time last_esp32_time_;
    rclcpp::Time last_limiter_time_;
    rclcpp::Time last_muxed_time_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_state_time_;

    bool lidar_alive_    = false;
    bool esp32_alive_    = false;
    bool person_alive_   = false;
    bool limiter_alive_  = false;
    bool muxed_alive_    = false;
    bool cmd_alive_      = false;
    bool state_alive_    = false;

    bool muxed_last_nonzero_ = false;
    bool cmd_last_nonzero_   = false;
    double last_cmd_linear_x_ = 0.0;
    double last_wheel_left_   = 0.0;
    double last_wheel_right_  = 0.0;

    std::string last_mode_  = "";
    std::string last_state_ = "";

    bool firmware_flags_received_ = false;
    bool firmware_bypass_active_  = false;

    // 現在アクティブなフォルトの集合（fault_lock/clear_estop_ui の判定に使う）
    std::set<std::string> active_faults_;

    rclcpp::Time startup_grace_end_;

    std::chrono::milliseconds lidar_timeout_;
    std::chrono::milliseconds esp32_timeout_;
    std::chrono::milliseconds person_timeout_;
    std::chrono::milliseconds limiter_dead_;
    std::chrono::milliseconds mux_dead_;
    std::chrono::milliseconds state_stale_;
    double runaway_ratio_ = 1.5;
    double runaway_zero_threshold_ = 0.02;
    th_safety::HoldTimer runaway_hold_;
    // WS-9O: 重大フォルトの単発誤検知よけ（回復可能フォルトは一時停止から
    // 正常に再開できるため対象外）。
    th_safety::HoldTimer limiter_dead_hold_{0.0};
    th_safety::HoldTimer mux_dead_hold_{0.0};
    th_safety::HoldTimer state_inconsist_hold_{0.0};
    double estop_ui_lease_sec_ = 1.5;
    double check_period_sec_ = 0.1;
    double startup_deadline_sec_ = 15.0;
    int link_quality_window_sec_ = 30;

    std::vector<std::string> enabled_targets_;

    // リンク品質（診断用。Q-1: 判定には使わない）
    th_safety::GapTracker esp32_gap_;
    th_safety::GapTracker lidar_gap_;
    th_safety::GapTracker ui_gap_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}
