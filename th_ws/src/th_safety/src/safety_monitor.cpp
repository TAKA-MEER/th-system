// ============================================================
// safety_monitor — 安全監視ノード (C++)
//
// 監視対象:
//   ・物理 E-Stop (/safety/estop_hw from ESP32)
//   ・タブレット緊急停止 (/safety/tablet_estop)
//   ・LiDAR /scan  タイムアウト
//   ・試験員追従 /person/status タイムアウト
//   ・ESP32 /esp32/wheel_feedback タイムアウト
//
// 出力:
//   /safety/estop  (Bool)        — twist_mux の lock に使用
//   /safety/fault  (FaultStatus) — mode_manager への通知
//
// 加えて、リンク品質の計測だけを行い publish する（WP-SAFE-00）。
//   /safety/link_quality (LinkQuality) — ESP32/LiDAR/UI の受信間隔 p50/p99/max。
//   診断用であり、フォルト判定には一切使わない（Q-1）。既存のフォルト判定
//   ロジックには触れていない。
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <th_system_msgs/msg/fault_status.hpp>
#include <th_system_msgs/msg/wheel_feedback.hpp>
#include <th_system_msgs/msg/person_status.hpp>
#include <th_system_msgs/msg/link_quality.hpp>
#include <th_system_msgs/msg/active_screen.hpp>

#include <th_safety/link_quality_core.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

class SafetyMonitor : public rclcpp::Node {
public:
    SafetyMonitor() : Node("safety_monitor") {
        // ── パラメータ ──────────────────────────────────────
        declare_parameter("lidar_timeout_ms",     500);
        declare_parameter("esp32_timeout_ms",     500);
        declare_parameter("person_timeout_ms",    500);
        declare_parameter("check_period_ms",      100);
        declare_parameter("startup_grace_sec",    3);
        // 診断用（WP-SAFE-00）。registry.yaml の link_quality_window_sec と
        // 同じ値を safety_monitor.yaml 経由で渡す（R2。generated/ 移行は
        // WP-PARAM-02）。ここで宣言する既定値 30 は yaml 未指定時の保険。
        declare_parameter("link_quality_window_sec", 30);

        lidar_timeout_  = std::chrono::milliseconds(
            get_parameter("lidar_timeout_ms").as_int());
        esp32_timeout_  = std::chrono::milliseconds(
            get_parameter("esp32_timeout_ms").as_int());
        person_timeout_ = std::chrono::milliseconds(
            get_parameter("person_timeout_ms").as_int());
        link_quality_window_sec_ = static_cast<double>(
            get_parameter("link_quality_window_sec").as_int());

        // ── Publishers ─────────────────────────────────────
        pub_estop_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/estop", rclcpp::QoS(1).reliable());
        pub_fault_ = create_publisher<th_system_msgs::msg::FaultStatus>(
            "/safety/fault", rclcpp::QoS(1).reliable());
        // twist_mux の fault lock 入力 (Bool): active=true 時に true を発行
        pub_fault_lock_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/fault_lock", rclcpp::QoS(1).reliable());
        // 診断用（WP-SAFE-00）。フォルト判定には使わない（Q-1）。
        pub_link_quality_ = create_publisher<th_system_msgs::msg::LinkQuality>(
            "/safety/link_quality", rclcpp::QoS(1).best_effort());

        // ── Subscribers ────────────────────────────────────
        // 物理 E-Stop (ESP32 GPIO → esp32_bridge 中継)
        sub_estop_hw_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/estop_hw", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                hw_estop_active_ = msg->data;
            });

        // タブレット緊急停止
        sub_tablet_estop_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/tablet_estop", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                tablet_estop_active_ = msg->data;
            });

        // LiDAR 死活監視
        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr) {
                last_scan_time_ = now();
                lidar_alive_    = true;
                pushGap(lidar_gap_tracker_, last_scan_time_, "lidar");
            });

        // 試験員追従 死活監視
        sub_person_ = create_subscription<th_system_msgs::msg::PersonStatus>(
            "/person/status", 10,
            [this](const th_system_msgs::msg::PersonStatus::SharedPtr) {
                last_person_time_ = now();
                person_alive_     = true;
            });

        // ESP32 フィードバック 死活監視
        sub_wheel_fb_ = create_subscription<th_system_msgs::msg::WheelFeedback>(
            "/esp32/wheel_feedback", 10,
            [this](const th_system_msgs::msg::WheelFeedback::SharedPtr) {
                last_esp32_time_ = now();
                esp32_alive_     = true;
                pushGap(esp32_gap_tracker_, last_esp32_time_, "esp32");
            });

        // UI 死活監視（診断用。WP-SAFE-00）。
        // publisher はまだ存在しない（WP-UI-01 で追加）。受信 0 でも
        // link_quality の publish は続く（Q-2）ので、購読自体は落ちない。
        sub_active_screen_ = create_subscription<th_system_msgs::msg::ActiveScreen>(
            "/ui/active_screen", rclcpp::QoS(5).reliable(),
            [this](const th_system_msgs::msg::ActiveScreen::SharedPtr) {
                pushGap(ui_gap_tracker_, now(), "ui");
            });

        // ── 監視タイマー ────────────────────────────────────
        int period = get_parameter("check_period_ms").as_int();
        check_timer_ = create_wall_timer(
            std::chrono::milliseconds(period),
            std::bind(&SafetyMonitor::checkHealth, this));

        // リンク品質の 1 Hz タイマ（診断用・WP-SAFE-00）。
        // FMEA③: メインループ（監視タイマー）を圧迫しないよう、別の
        // コールバックグループに置く。既存の checkHealth() とは独立に動く。
        link_quality_cbg_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        link_quality_timer_ = create_wall_timer(
            1s,
            std::bind(&SafetyMonitor::publishLinkQuality, this),
            link_quality_cbg_);

        rclcpp::Time t0 = now();
        last_scan_time_   = t0;
        last_person_time_ = t0;
        last_esp32_time_  = t0;

        // 起動直後のタイムアウト誤検知を防ぐため初回は余裕を持たせる
        int grace_sec = get_parameter("startup_grace_sec").as_int();
        startup_grace_end_ = t0 + rclcpp::Duration(grace_sec, 0);

        RCLCPP_INFO(get_logger(), "safety_monitor 起動");
    }

private:
    void checkHealth() {
        rclcpp::Time t = now();
        bool in_grace = (t < startup_grace_end_);

        // ── E-Stop 集約 ────────────────────────────────────
        bool estop = hw_estop_active_ || tablet_estop_active_;
        std_msgs::msg::Bool estop_msg;
        estop_msg.data = estop;
        pub_estop_->publish(estop_msg);

        if (estop && !prev_estop_) {
            RCLCPP_WARN(get_logger(), "E-Stop 発動 (hw=%d tablet=%d)",
                        hw_estop_active_, tablet_estop_active_);
        } else if (!estop && prev_estop_) {
            RCLCPP_INFO(get_logger(), "E-Stop 解除");
        }
        prev_estop_ = estop;

        // ── フォルト判定 ────────────────────────────────────
        if (!in_grace) {
            checkTimeout("LIDAR_LOST", last_scan_time_, lidar_timeout_,
                         lidar_alive_, prev_lidar_fault_, t);
            checkTimeout("ESP32_DISCONNECTED", last_esp32_time_, esp32_timeout_,
                         esp32_alive_, prev_esp32_fault_, t);
            checkTimeout("PERSON_TRACKER_LOST", last_person_time_, person_timeout_,
                         person_alive_, prev_person_fault_, t);
        }

        // fault_lock を毎サイクル発行する（twist_mux timeout=0.5s を防ぐ）
        // /safety/estop と同じく、状態変化の有無にかかわらず継続送信する。
        publishLock();
    }

    void checkTimeout(const std::string& fault_type,
                      const rclcpp::Time& last_time,
                      const std::chrono::milliseconds& timeout,
                      bool& alive_flag,
                      bool& prev_fault,
                      const rclcpp::Time& now_t) {
        double elapsed = (now_t - last_time).seconds();
        double limit   = timeout.count() / 1000.0;
        bool faulted   = (elapsed > limit);
        bool was_fault = prev_fault;
        // publishFault() 内で publishLock() が prev_lidar_fault_/prev_esp32_fault_
        // を読むため、呼び出し前に確定させておく（そうしないと今まさに検知した
        // フォルトがロック信号に反映されず、次の checkHealth() 周期まで
        // 反映が 1 周期分遅れる）。
        prev_fault = faulted;

        if (faulted && !was_fault) {
            // フォルト発生
            RCLCPP_ERROR(get_logger(), "[FAULT] %s (%.2f s)", fault_type.c_str(), elapsed);
            publishFault(true, fault_type);
            alive_flag = false;
        } else if (!faulted && was_fault) {
            // フォルト回復
            RCLCPP_INFO(get_logger(), "[FAULT CLEARED] %s", fault_type.c_str());
            publishFault(false, "NONE");
        }
    }

    void publishFault(bool active, const std::string& type) {
        th_system_msgs::msg::FaultStatus msg;
        msg.active     = active;
        msg.fault_type = type;
        pub_fault_->publish(msg);

        // twist_mux の fault lock (Bool) へも同期発行（統合状態を再送、単発の
        // active では上書きしない。他フォルトが継続中に消えるのを防ぐ）
        publishLock();
    }

    // twist_mux 物理ロックは LIDAR_LOST / ESP32_DISCONNECTED のみを対象とする。
    // PERSON_TRACKER_LOST は走行の物理安全とは無関係(試験員位置に依存する
    // モードでの IDLE 強制は mode_manager 側の判断に委ねる。VISION.md §5,
    // 2026-07-24 決定)なので、ここではロックに含めない。
    void publishLock() {
        std_msgs::msg::Bool lock_msg;
        lock_msg.data = prev_lidar_fault_ || prev_esp32_fault_;
        pub_fault_lock_->publish(lock_msg);
    }

    // ========================================================
    // リンク品質の計測（診断用。WP-SAFE-00）
    //
    // Q-1: フォルト判定には一切使わない。既存のフォルト判定ロジック
    //      （checkTimeout / publishFault / publishLock、上記）には触れていない。
    // FMEA①: GapTracker が例外を投げても safety_monitor を落とさない。
    //         push() 側は既存の死活監視コールバック内で呼ばれるため、
    //         例外はここで飲み込み、既存の死活監視の状態更新には影響させない。
    // ========================================================

    // 各サブスクライバのコールバックから呼ぶ。GapTracker が例外を投げても
    // 呼び出し元（死活監視の状態更新）には一切波及させない。
    void pushGap(th_safety::GapTracker& tracker, const rclcpp::Time& stamp,
                 const char* link_name) {
        try {
            tracker.push(stamp.seconds());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "[link_quality:%s] push 失敗（無視）: %s",
                        link_name, e.what());
        } catch (...) {
            RCLCPP_WARN(get_logger(), "[link_quality:%s] push 失敗（無視・不明な例外）",
                        link_name);
        }
    }

    // 1 Hz タイマから呼ばれる。ESP32 / LiDAR / UI の 3 本を publish する。
    void publishLinkQuality() {
        const double now_sec = now().seconds();
        publishOneLinkQuality("esp32", esp32_gap_tracker_, now_sec);
        publishOneLinkQuality("lidar", lidar_gap_tracker_, now_sec);
        publishOneLinkQuality("ui",    ui_gap_tracker_,    now_sec);
    }

    // link 単位で例外を隔離する。1 本の計算が失敗しても他の 2 本の publish は
    // 継続する（FMEA①「失敗しても publish をスキップするだけにする」）。
    void publishOneLinkQuality(const char* link_name,
                                const th_safety::GapTracker& tracker,
                                double now_sec) {
        try {
            th_safety::Quantiles q = tracker.compute(now_sec, link_quality_window_sec_);

            th_system_msgs::msg::LinkQuality msg;
            msg.header.stamp = now();
            msg.link          = link_name;
            msg.p50_ms        = static_cast<float>(q.p50_ms);
            msg.p99_ms        = static_cast<float>(q.p99_ms);
            msg.max_ms        = static_cast<float>(q.max_ms);
            msg.window_sec    = q.window_sec;
            pub_link_quality_->publish(msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "[link_quality:%s] publish スキップ: %s",
                        link_name, e.what());
        } catch (...) {
            RCLCPP_WARN(get_logger(), "[link_quality:%s] publish スキップ（不明な例外）",
                        link_name);
        }
    }

    // ── Publishers ────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_estop_;
    rclcpp::Publisher<th_system_msgs::msg::FaultStatus>::SharedPtr   pub_fault_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_fault_lock_;
    rclcpp::Publisher<th_system_msgs::msg::LinkQuality>::SharedPtr   pub_link_quality_;

    // ── Subscribers ──────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             sub_estop_hw_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             sub_tablet_estop_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     sub_scan_;
    rclcpp::Subscription<th_system_msgs::msg::PersonStatus>::SharedPtr sub_person_;
    rclcpp::Subscription<th_system_msgs::msg::WheelFeedback>::SharedPtr sub_wheel_fb_;
    rclcpp::Subscription<th_system_msgs::msg::ActiveScreen>::SharedPtr sub_active_screen_;

    rclcpp::TimerBase::SharedPtr check_timer_;

    // リンク品質（診断用。WP-SAFE-00）。既存の監視タイマーとは別の
    // コールバックグループに置く（FMEA③。メインループを圧迫しない）。
    rclcpp::CallbackGroup::SharedPtr link_quality_cbg_;
    rclcpp::TimerBase::SharedPtr     link_quality_timer_;
    th_safety::GapTracker esp32_gap_tracker_;
    th_safety::GapTracker lidar_gap_tracker_;
    th_safety::GapTracker ui_gap_tracker_;
    double link_quality_window_sec_ = 30.0;

    // ── 状態 ─────────────────────────────────────────────
    bool hw_estop_active_    = false;
    bool tablet_estop_active_ = false;
    bool prev_estop_         = false;

    rclcpp::Time last_scan_time_;
    rclcpp::Time last_person_time_;
    rclcpp::Time last_esp32_time_;

    bool lidar_alive_  = false;
    bool esp32_alive_  = false;
    bool person_alive_ = false;

    bool prev_lidar_fault_  = false;
    bool prev_esp32_fault_  = false;
    bool prev_person_fault_ = false;

    rclcpp::Time startup_grace_end_;

    std::chrono::milliseconds lidar_timeout_;
    std::chrono::milliseconds esp32_timeout_;
    std::chrono::milliseconds person_timeout_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}
