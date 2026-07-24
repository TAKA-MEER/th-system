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
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <th_system_msgs/msg/fault_status.hpp>
#include <th_system_msgs/msg/wheel_feedback.hpp>
#include <th_system_msgs/msg/person_status.hpp>

#include <chrono>

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

        lidar_timeout_  = std::chrono::milliseconds(
            get_parameter("lidar_timeout_ms").as_int());
        esp32_timeout_  = std::chrono::milliseconds(
            get_parameter("esp32_timeout_ms").as_int());
        person_timeout_ = std::chrono::milliseconds(
            get_parameter("person_timeout_ms").as_int());

        // ── Publishers ─────────────────────────────────────
        pub_estop_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/estop", rclcpp::QoS(1).reliable());
        pub_fault_ = create_publisher<th_system_msgs::msg::FaultStatus>(
            "/safety/fault", rclcpp::QoS(1).reliable());
        // twist_mux の fault lock 入力 (Bool): active=true 時に true を発行
        pub_fault_lock_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/fault_lock", rclcpp::QoS(1).reliable());

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
            });

        // ── 監視タイマー ────────────────────────────────────
        int period = get_parameter("check_period_ms").as_int();
        check_timer_ = create_wall_timer(
            std::chrono::milliseconds(period),
            std::bind(&SafetyMonitor::checkHealth, this));

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

    // ── Publishers ────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_estop_;
    rclcpp::Publisher<th_system_msgs::msg::FaultStatus>::SharedPtr   pub_fault_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                pub_fault_lock_;

    // ── Subscribers ──────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             sub_estop_hw_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             sub_tablet_estop_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     sub_scan_;
    rclcpp::Subscription<th_system_msgs::msg::PersonStatus>::SharedPtr sub_person_;
    rclcpp::Subscription<th_system_msgs::msg::WheelFeedback>::SharedPtr sub_wheel_fb_;

    rclcpp::TimerBase::SharedPtr check_timer_;

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
