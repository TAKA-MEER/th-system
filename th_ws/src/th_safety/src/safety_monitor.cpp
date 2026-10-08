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
//                （開発モードの項目 lidar_fault が実効の間は出さない。/system/dev_mode
//                  を購読し effective だけを読む。dev_mode_core.hpp・Spec-safety.md §10）
//   esp32        /esp32/wheel_feedback タイムアウト          → ESP32_DISCONNECTED (RECOVERABLE)
//   person       /person/targets タイムアウト                → PERSON_TRACKER_LOST (RECOVERABLE)
//                （§5.5: /system/state.tracker_enabled が false の間は判定せず
//                  既存フォルトを解除。false→true エッジから
//                  person_startup_grace_ms の間も保留。person_report_only=true
//                  の間は FaultStatus／fault_lock を出さず記録だけ）
//   limiter      /safety/limiter_status タイムアウト         → LIMITER_DEAD (CRITICAL)
//   localization /safety/localization_health 途絶・ok==false 継続 → LOCALIZATION_LOST (CRITICAL)
//   mux          /cmd_vel_muxed ⇄ /cmd_vel の双方向途絶 ＋ twist_mux の生存確認
//                （/cmd_vel_muxed の publisher に twist_mux が居るか）→ MUX_DEAD (CRITICAL)
//                （mux_report_only=true の間は FaultStatus／fault_lock を出さず記録だけ。
//                  人物追跡の person_report_only と同じ形）
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
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <th_system_msgs/msg/fault_status.hpp>
#include <th_system_msgs/msg/wheel_feedback.hpp>
#include <th_system_msgs/msg/person_targets.hpp>
#include <th_system_msgs/msg/limiter_status.hpp>
#include <th_system_msgs/msg/localization_health.hpp>
#include <th_system_msgs/msg/system_state.hpp>
#include <th_system_msgs/msg/link_quality.hpp>

#include "th_safety/safety_monitor_core.hpp"
#include "th_safety/link_quality_core.hpp"
#include "th_safety/dev_mode_core.hpp"

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
        // §5.5.2: tracker_enabled の false→true エッジから person 判定を
        // 保留する猶予。既定値は registry.yaml（person_startup_grace_ms）が正。
        // 未計測のため 5000 ms（仮。P-04 で実測）。
        declare_parameter("person_startup_grace_ms", 5000);
        // §5.5.5: true の間は PERSON_TRACKER_LOST が成立しても FaultStatus／
        // fault_lock を出さず内部ログ＋カウンタのみ。false なら従来どおり
        // フォルト。P-04 の本有効化で false にする。
        declare_parameter("person_report_only", true);
        declare_parameter("limiter_dead_ms",      250);
        // WP-SAFE-05: /safety/localization_health（1 Hz）の途絶判定。
        // 既定値は registry.yaml（localization_topic_timeout_ms）が正。
        declare_parameter("localization_topic_timeout_ms", 5000);
        declare_parameter("mux_dead_ms",          500);
        // SG-A11（2026-10-08 ユーザー決定）: twist_mux の生存確認。
        // /cmd_vel_muxed の publisher に twist_mux ノードが居ない状態がこの時間
        // 続いたら死んだとみなす（入力が無くて出力が黙っているだけの停止中でも
        // 検知できる）。起動直後は publisher 未発見のまま startup_deadline_sec まで待つ。
        // 既定値は registry.yaml（mux_liveness_grace_ms）が正。
        declare_parameter("mux_liveness_grace_ms", 1000);
        // true の間は MUX_DEAD が成立しても FaultStatus／fault_lock を出さず
        // ログ＋カウンタのみ（person_report_only と同じ形）。誤検知がないことを
        // 実機で確かめてから false にする。既定値は registry.yaml が正。
        declare_parameter("mux_report_only", true);
        declare_parameter("state_stale_ms",       1500);
        declare_parameter("runaway_ratio",        1.5);
        // Spec-safety.md §3.5.4（W-06 の⑤）: 発進・停止直後の追従遅れ
        // （最大 0.7 秒）より長く。既定値は registry.yaml（runaway_hold_ms）が正。
        declare_parameter("runaway_hold_ms",      1000);
        // W-06 の②（Spec-safety.md §3.5.3）: DRIVE_RUNAWAY の実測の鮮度しきい値。
        // 既定値は registry.yaml（runaway_feedback_stale_ms）が正。
        declare_parameter("runaway_feedback_stale_ms", 250);
        // WS-9O (2026-09-04): 重大フォルトの発火に課す保持時間。
        // 監視ループ (check_period_ms) 自身が一時的に遅れると、複数の入力が同じ
        // 判定周期で同時にタイムアウト超過に見える。実機ログで ESP32_DISCONNECTED と
        // LIMITER_DEAD が同じ 1ms に発火し 26ms 後に両方解除された。フォルトは edge で
        // publish されるので、その 26ms でも active=true は state_manager に届き
        // C-06a (ガード無し) で必ず ESTOP に落ちる。条件が継続したときだけ報告する。
        // タイムアウト値そのもの (limiter_dead_ms 等) は変えない。
        declare_parameter("critical_fault_hold_ms", 300);
        // Spec-safety.md §3.5.4（W-06 の③）: 急停止直後の荷重移動による
        // わずかな並進（実測 -0.02〜-0.05 m/s）を停止とみなす。
        // 既定値は registry.yaml（runaway_zero_threshold）が正。
        declare_parameter("runaway_zero_threshold", 0.08);
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
        person_startup_grace_ = std::chrono::milliseconds(
            get_parameter("person_startup_grace_ms").as_int());
        person_report_only_ = get_parameter("person_report_only").as_bool();
        limiter_dead_   = std::chrono::milliseconds(get_parameter("limiter_dead_ms").as_int());
        localization_topic_timeout_ = std::chrono::milliseconds(
            get_parameter("localization_topic_timeout_ms").as_int());
        mux_dead_       = std::chrono::milliseconds(get_parameter("mux_dead_ms").as_int());
        mux_liveness_grace_ = std::chrono::milliseconds(
            get_parameter("mux_liveness_grace_ms").as_int());
        mux_report_only_ = get_parameter("mux_report_only").as_bool();
        state_stale_    = std::chrono::milliseconds(get_parameter("state_stale_ms").as_int());
        runaway_ratio_  = get_parameter("runaway_ratio").as_double();
        runaway_zero_threshold_ = get_parameter("runaway_zero_threshold").as_double();
        runaway_feedback_stale_ = std::chrono::milliseconds(
            get_parameter("runaway_feedback_stale_ms").as_int());
        runaway_hold_   = th_safety::HoldTimer(get_parameter("runaway_hold_ms").as_int() / 1000.0);
        {
            const double hold = get_parameter("critical_fault_hold_ms").as_int() / 1000.0;
            limiter_dead_hold_  = th_safety::HoldTimer(hold);
            localization_dead_hold_ = th_safety::HoldTimer(hold);
            mux_dead_hold_      = th_safety::HoldTimer(hold);
            mux_report_hold_    = th_safety::HoldTimer(hold);
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

        // 開発モード（Spec-safety.md §10）。未受信・鮮度切れ・壊れた JSON は
        // 何も無視しない（dev_mode_core.hpp）。
        sub_dev_mode_ = create_subscription<std_msgs::msg::String>(
            "/system/dev_mode", rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                dev_mode_.received = true;
                dev_mode_.stamp_sec = now().seconds();
                dev_mode_.effective = th_safety::parse_dev_effective(msg->data);
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

        // WP-SAFE-05: localization_health（重大。publisher は localization_health。
        // enabled_targets に "localization" を入れるまでは監視しない — F-5・O-7）
        sub_localization_health_ = create_subscription<th_system_msgs::msg::LocalizationHealth>(
            "/safety/localization_health", rclcpp::QoS(1).reliable(),
            [this](const th_system_msgs::msg::LocalizationHealth::SharedPtr msg) {
                last_localization_time_ = now();
                localization_alive_     = true;
                localization_ok_        = msg->ok;
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
        // §5.5.1: tracker_enabled もここで保持する。false→true の変化時刻を
        // 覚え、§5.5.2 の猶予（person_startup_grace_ms）の起点にする。
        sub_system_state_ = create_subscription<th_system_msgs::msg::SystemState>(
            "/system/state", rclcpp::QoS(1).reliable().transient_local(),
            [this](const th_system_msgs::msg::SystemState::SharedPtr msg) {
                last_state_time_ = now();
                state_alive_     = true;
                last_mode_  = msg->mode;
                last_state_ = msg->state;
                if (msg->tracker_enabled && !tracker_enabled_) {
                    tracker_enable_edge_time_ = now();
                }
                tracker_enabled_ = msg->tracker_enabled;
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
        tracker_enable_edge_time_ = t0;
        last_scan_time_    = t0;
        last_person_time_  = t0;
        last_esp32_time_   = t0;
        last_limiter_time_ = t0;
        last_localization_time_ = t0;
        last_muxed_time_   = t0;
        last_mux_node_time_ = t0;
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
                if (devIgnoreLidarFault(t)) {
                    // 既に立っている LIDAR_LOST も解除する（飛ばすだけでは fault_lock が残る）。
                    updateFaultState("LIDAR_LOST", false);
                } else {
                    checkTimeout("LIDAR_LOST", last_scan_time_, lidar_timeout_, t, lidar_alive_);
                }
            }
            if (targetEnabled("esp32")) {
                checkTimeout("ESP32_DISCONNECTED", last_esp32_time_, esp32_timeout_, t, esp32_alive_);
            }
            if (targetEnabled("person")) {
                // §5.5.1/§5.5.2: tracker OFF の間は判定せず既存フォルトを解除する。
                // ON 直後の猶予中は判定せず何もしない。猶予後は通常の途絶判定。
                // §5.5.5: report_only の間は成立しても FaultStatus／fault_lock を
                // 出さず、スロットル付き WARN とカウンタで記録だけする。
                const double grace_sec =
                    std::chrono::duration<double>(person_startup_grace_).count();
                const double since_enable_sec = (t - tracker_enable_edge_time_).seconds();
                switch (th_safety::person_gate(tracker_enabled_, since_enable_sec, grace_sec)) {
                    case th_safety::PersonGate::SKIP_DISABLED:
                        updateFaultState("PERSON_TRACKER_LOST", false);
                        prev_person_report_lost_ = false;
                        break;
                    case th_safety::PersonGate::SKIP_GRACE:
                        break;
                    case th_safety::PersonGate::CHECK: {
                        const bool lost = computeTimeoutFault(
                            last_person_time_, person_timeout_, t, person_alive_);
                        if (lost && person_report_only_) {
                            if (!prev_person_report_lost_) {
                                ++person_report_only_episodes_;
                                RCLCPP_WARN(
                                    get_logger(),
                                    "[PERSON_TRACKER_LOST 記録だけ %zu 件目] "
                                    "tracker ON 中に /person/targets が途絶 "
                                    "(person_timeout_ms=%lld)。"
                                    "report_only のためフォルトは出さない",
                                    person_report_only_episodes_,
                                    static_cast<long long>(person_timeout_.count()));
                            } else {
                                RCLCPP_WARN_THROTTLE(
                                    get_logger(), *get_clock(), 30000,
                                    "[PERSON_TRACKER_LOST 記録だけ] 途絶が継続中 "
                                    "(これまでの検出 %zu 件)。"
                                    "report_only のためフォルトは出さない",
                                    person_report_only_episodes_);
                            }
                        } else {
                            if (!lost && prev_person_report_lost_) {
                                RCLCPP_INFO(
                                    get_logger(),
                                    "[PERSON_TRACKER_LOST 記録だけ] 途絶が解消 "
                                    "(これまでの検出 %zu 件)",
                                    person_report_only_episodes_);
                            }
                            updateFaultState("PERSON_TRACKER_LOST", lost);
                        }
                        prev_person_report_lost_ = lost;
                        break;
                    }
                }
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
            if (targetEnabled("localization")) {
                // WP-SAFE-05: ①トピック途絶（未受信は startup_deadline まで待つ。
                // computeTimeoutFault の is_timeout_fault 経由。limiter と同じ規則）
                bool topic_dead = computeTimeoutFault(
                    last_localization_time_, localization_topic_timeout_, t,
                    localization_alive_);
                // ②ok==false の継続（1 通でも届いていることが前提。届いていない
                // 間の ng は①が担う）。どちらも WS-9O の保持時間を課す（単発で撃たない）。
                bool ng_held = localization_alive_ && !localization_ok_;
                updateFaultState("LOCALIZATION_LOST",
                                  localization_dead_hold_.update(topic_dead || ng_held,
                                                                  check_period_sec_));
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
                const bool flow_dead = th_safety::detect_mux_dead(
                    muxed_stale, muxed_last_nonzero_, cmd_stale, cmd_last_nonzero_);
                // SG-A11: 生存確認。入力が無く出力が黙っている停止中の死は上の
                // 流れの判定では見えないので、ノードの存在そのものを見る。
                const bool mux_absent = muxNodeAbsent(t, since_start);
                const bool mux_cond = flow_dead || mux_absent;
                if (!mux_report_only_) {
                    // WS-9O: 単発の誤検知よけ
                    updateFaultState("MUX_DEAD",
                                      mux_dead_hold_.update(mux_cond, check_period_sec_));
                } else {
                    // 記録だけ。FaultStatus／fault_lock は出さない（立っている
                    // MUX_DEAD があれば解除する。実行中に記録だけへ戻した場合）。
                    // 保持時間は同じ（誤検知を記録に数えない）。
                    updateFaultState("MUX_DEAD", false);
                    const bool held = mux_report_hold_.update(mux_cond, check_period_sec_);
                    if (held && !prev_mux_report_dead_) {
                        ++mux_report_only_episodes_;
                        RCLCPP_WARN(
                            get_logger(),
                            "[MUX_DEAD 記録だけ %zu 件目] twist_mux の異常を検知 "
                            "(生存確認=%s 流れ=%s)。mux_report_only のためフォルトは出さない",
                            mux_report_only_episodes_,
                            mux_absent ? "twist_mux が居ない" : "居る",
                            flow_dead ? "不整合" : "正常");
                    } else if (held) {
                        RCLCPP_WARN_THROTTLE(
                            get_logger(), *get_clock(), 30000,
                            "[MUX_DEAD 記録だけ] 異常が継続中 (これまでの検出 %zu 件)。"
                            "mux_report_only のためフォルトは出さない",
                            mux_report_only_episodes_);
                    } else if (prev_mux_report_dead_) {
                        RCLCPP_INFO(get_logger(),
                                    "[MUX_DEAD 記録だけ] 異常が解消 (これまでの検出 %zu 件)",
                                    mux_report_only_episodes_);
                    }
                    prev_mux_report_dead_ = held;
                }
            }
            if (targetEnabled("runaway")) {
                // W-06 の②（Spec-safety.md §3.5.3）: 実測が新鮮なときだけ判定する。
                // 古いあいだは凍結する — runaway_hold_.update() を呼ばない（進めも
                // 戻しもしない）し、updateFaultState() も呼ばない（フォルト状態を
                // 変えない）。「古いとき update(false) で保持時間を戻す／
                // updateFaultState(..., false) で解除する」にしない理由:
                // 本物の暴走が起きていて実測が断続的に途切れるとき、保持時間が
                // 永遠に溜まらず暴走を見逃すため。凍結される窓は有限
                // （古いまま esp32_timeout_ms を超えれば ESP32_DISCONNECTED が、
                // ファームのウォッチドッグ 600ms がそれぞれ担う）。
                double feedback_age_sec = (t - last_esp32_time_).seconds();
                double stale_sec =
                    std::chrono::duration<double>(runaway_feedback_stale_).count();
                bool fresh = th_safety::is_runaway_feedback_fresh(
                    esp32_alive_, feedback_age_sec, stale_sec);
                double feedback_abs = std::fabs((last_wheel_left_ + last_wheel_right_) / 2.0);
                double cmd_abs = std::fabs(last_cmd_linear_x_);
                bool condition = th_safety::is_runaway_condition(
                    cmd_abs, feedback_abs, runaway_ratio_, runaway_zero_threshold_);
                std::optional<bool> runaway =
                    th_safety::update_runaway_with_freshness(
                        fresh, condition, runaway_hold_, check_period_sec_);
                if (runaway.has_value()) {
                    updateFaultState("DRIVE_RUNAWAY", *runaway);
                }
                // frozen（nullopt）のときは何もしない（上のコメント参照）。
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

    // SG-A11: /cmd_vel_muxed の publisher に twist_mux ノードが居ないか。
    // 居なくなってから mux_liveness_grace_ms を超えたら真（起動直後に一度も
    // 見えないときは startup_deadline_sec まで待つ。detect_mux_absent）。
    // グラフの問い合わせが失敗したときは「居た」ことにする（不明を死と決めない）。
    bool muxNodeAbsent(const rclcpp::Time& t, double since_start) {
        try {
            const auto pubs = get_publishers_info_by_topic("/cmd_vel_muxed");
            bool present = false;
            for (const auto& info : pubs) {
                if (info.node_name() == kMuxNodeName) { present = true; break; }
            }
            if (present) {
                last_mux_node_time_ = t;
                mux_node_seen_ = true;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "twist_mux の生存確認に失敗 (%s)。今回は判定しない", e.what());
            last_mux_node_time_ = t;
        }
        return th_safety::detect_mux_absent(
            mux_node_seen_, (t - last_mux_node_time_).seconds(),
            std::chrono::duration<double>(mux_liveness_grace_).count(),
            since_start, startup_deadline_sec_);
    }

    // 開発モードの項目 lidar_fault が実効か。切り替わりをログに残す。
    bool devIgnoreLidarFault(const rclcpp::Time& now_t) {
        const bool on = th_safety::dev_item_effective(
            dev_mode_, th_safety::kDevItemLidarFault, now_t.seconds());
        if (on != prev_dev_ignore_lidar_fault_) {
            if (on) {
                RCLCPP_WARN(get_logger(), "開発モード: lidar_fault 有効（LIDAR_LOST を出さない）");
            } else {
                RCLCPP_INFO(get_logger(), "開発モード: lidar_fault 無効（LIDAR_LOST の監視を再開）");
            }
            prev_dev_ignore_lidar_fault_ = on;
        }
        return on;
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

    // SG-A8: フォルトの集約。active_faults_ に何か残っている間は
    // active=true を出し続け、代表は最も重いもの（重大 > 回復。同一階級では
    // 辞書順最小で決定的にする）。全部消えたときだけ active=false, NONE。
    // state_manager は最後のメッセージで上書きするだけなので、この出し方で
    // fault_cleared / estop_resume_prev が正しく効く（C-04・C-09c）。
    std::string representativeFault() const {
        std::string best;
        bool best_critical = false;
        for (const auto& ft : active_faults_) {
            bool crit = (th_safety::classify_severity(ft) == th_safety::Severity::CRITICAL);
            if (best.empty() || (crit && !best_critical) ||
                (crit == best_critical && ft < best)) {
                best = ft;
                best_critical = crit;
            }
        }
        return best;
    }

    // フォルトの edge 検出 + publish。active_faults_ を更新する（fault_lock の合成に使う）。
    //
    // SG-A8: 発生時はそのフォルト自身の型を出すのが基本（単一フォルト由来の
    // 既存試験・購読者の種別判定が曇らないよう）。ただし新しいフォルトが
    // 「いま残っている最も重いもの」より軽いときだけ代表（重い方）を
    // active=true で出し直す。こうしないと最後のメッセージの severity が
    // 軽くなり、state_manager の重大専用ガード（_hw_released_and_no_critical・
    // _no_critical_fault。C-11 系）が重大継続中に通ってしまう（2026-10-05
    // 受け入れ検査）。同じ重さ以上なら自型（回復同士の既存試験を壊さない）。
    // 解消時は残りがあれば代表を active=true で出し続け、全部消えたときだけ
    // active=false, NONE を出す。state_manager は最後のメッセージで上書き
    // するだけなので、この出し方で fault_cleared / estop_resume_prev が正しく
    // 効く（C-04・C-09c）。残存中の代表再送による余計な fault.critical 再発火は、
    // ESTOP/CARRY での latch 抑制（N-3）と C-03 のガードで無害（PAUSE では
    // 同状態への再遷移＋W-1 再送のみ。CARRY では C-06a で ESTOP に戻るが、
    // 重大が継続している以上は安全側であり、既存試験の §7 行4 と同じ扱い）。
    void updateFaultState(const std::string& fault_type, bool faulted) {
        bool was_fault = active_faults_.count(fault_type) != 0;
        if (faulted && !was_fault) {
            // insert 前の代表が新しいフォルトより重い（＝重大が残っているのに
            // 回復が来た）ときだけ代表を出す。階級は Spec-safety.md §3.5 の 2 階級。
            std::string prev_rep = representativeFault();
            const bool heavier_remains =
                !prev_rep.empty() &&
                th_safety::classify_severity(prev_rep) == th_safety::Severity::CRITICAL &&
                th_safety::classify_severity(fault_type) != th_safety::Severity::CRITICAL;
            active_faults_.insert(fault_type);
            RCLCPP_ERROR(get_logger(), "[FAULT] %s (severity=%s)", fault_type.c_str(),
                         th_safety::severity_to_string(th_safety::classify_severity(fault_type)));
            publishFault(true, heavier_remains ? representativeFault() : fault_type);
        } else if (!faulted && was_fault) {
            active_faults_.erase(fault_type);
            RCLCPP_INFO(get_logger(), "[FAULT CLEARED] %s", fault_type.c_str());
            if (active_faults_.empty()) {
                publishFault(false, "NONE");
            } else {
                publishFault(true, representativeFault());
            }
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
    rclcpp::Subscription<th_system_msgs::msg::LocalizationHealth>::SharedPtr sub_localization_health_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       sub_cmd_vel_muxed_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       sub_cmd_vel_;
    rclcpp::Subscription<th_system_msgs::msg::SystemState>::SharedPtr sub_system_state_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr            sub_firmware_flags_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           sub_dev_mode_;

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
    rclcpp::Time last_localization_time_;
    rclcpp::Time last_muxed_time_;
    rclcpp::Time last_mux_node_time_;   // twist_mux が /cmd_vel_muxed の publisher に最後に見えた時刻
    bool mux_node_seen_ = false;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_state_time_;

    bool lidar_alive_    = false;
    bool esp32_alive_    = false;
    bool person_alive_   = false;
    bool limiter_alive_  = false;
    bool localization_alive_ = false;
    bool localization_ok_    = true;
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
    // §5.5.1: /system/state.tracker_enabled の最新値。false の間は person 判定
    // しない。§5.5.2: false→true エッジの時刻。猶予の起点。
    // §5.5.5: report_only 中の記録（途絶エピソード数と直前の途絶有無）。
    bool tracker_enabled_ = false;
    rclcpp::Time tracker_enable_edge_time_;
    bool person_report_only_ = true;
    std::chrono::milliseconds person_startup_grace_{5000};
    size_t person_report_only_episodes_ = 0;
    bool prev_person_report_lost_ = false;

    bool firmware_flags_received_ = false;
    bool firmware_bypass_active_  = false;

    th_safety::DevModeSnapshot dev_mode_;
    bool prev_dev_ignore_lidar_fault_ = false;

    // 現在アクティブなフォルトの集合（fault_lock/clear_estop_ui の判定に使う）
    std::set<std::string> active_faults_;

    rclcpp::Time startup_grace_end_;

    std::chrono::milliseconds lidar_timeout_;
    std::chrono::milliseconds esp32_timeout_;
    std::chrono::milliseconds person_timeout_;
    std::chrono::milliseconds limiter_dead_;
    std::chrono::milliseconds localization_topic_timeout_;
    std::chrono::milliseconds mux_dead_;
    std::chrono::milliseconds mux_liveness_grace_{1000};
    bool mux_report_only_ = true;
    bool prev_mux_report_dead_ = false;
    std::size_t mux_report_only_episodes_ = 0;
    static constexpr const char* kMuxNodeName = "twist_mux";
    std::chrono::milliseconds state_stale_;
    double runaway_ratio_ = 1.5;
    double runaway_zero_threshold_ = 0.08;
    std::chrono::milliseconds runaway_feedback_stale_{250};
    th_safety::HoldTimer runaway_hold_;
    // WS-9O: 重大フォルトの単発誤検知よけ（回復可能フォルトは一時停止から
    // 正常に再開できるため対象外）。
    th_safety::HoldTimer limiter_dead_hold_{0.0};
    th_safety::HoldTimer localization_dead_hold_{0.0};
    th_safety::HoldTimer mux_dead_hold_{0.0};
    th_safety::HoldTimer mux_report_hold_{0.0};   // mux_report_only の間だけ使う（記録の保持時間）
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
