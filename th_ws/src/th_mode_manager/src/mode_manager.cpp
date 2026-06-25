// ============================================================
// mode_manager — システム FSM ノード (C++)
//
// 状態: INIT → IDLE → FOLLOWING ↔ MOVING_TO_PANEL → AT_PANEL
//               ↕          ↕             ↕               ↕
//             MANUAL ←──────────────────────────────────┘
//               ↕
//             ESTOP (どの状態からでも割り込み)
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <th_system_msgs/msg/robot_mode.hpp>
#include <th_system_msgs/msg/fault_status.hpp>
#include <th_system_msgs/srv/set_mode.hpp>

#include <functional>
#include <string>
#include <map>

using RobotMode   = th_system_msgs::msg::RobotMode;
using FaultStatus = th_system_msgs::msg::FaultStatus;
using SetMode     = th_system_msgs::srv::SetMode;

static std::string modeName(uint8_t m) {
    static const std::map<uint8_t, std::string> names = {
        {RobotMode::INIT,            "INIT"},
        {RobotMode::IDLE,            "IDLE"},
        {RobotMode::FOLLOWING,       "FOLLOWING"},
        {RobotMode::MOVING_TO_PANEL, "MOVING_TO_PANEL"},
        {RobotMode::AT_PANEL,        "AT_PANEL"},
        {RobotMode::MANUAL,          "MANUAL"},
        {RobotMode::ESTOP,           "ESTOP"},
    };
    auto it = names.find(m);
    return it != names.end() ? it->second : "UNKNOWN";
}

class ModeManager : public rclcpp::Node {
public:
    ModeManager() : Node("mode_manager"), current_mode_(RobotMode::INIT) {
        // ── Publisher ───────────────────────────────────────
        pub_mode_ = create_publisher<RobotMode>(
            "/robot/mode", rclcpp::QoS(1).reliable().transient_local());

        // ── Subscribers ────────────────────────────────────
        // E-Stop: どの状態からでも ESTOP へ
        sub_estop_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/estop", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    transition(RobotMode::ESTOP, "E-Stop 発動");
                } else if (current_mode_ == RobotMode::ESTOP) {
                    transition(RobotMode::IDLE, "E-Stop 解除 → IDLE");
                }
            });

        // フォルト: FOLLOWING/MOVING_TO_PANEL/MANUAL/AT_PANEL → IDLE
        sub_fault_ = create_subscription<FaultStatus>(
            "/safety/fault", 10,
            [this](const FaultStatus::SharedPtr msg) {
                if (msg->active) {
                    uint8_t cm = current_mode_;
                    if (cm == RobotMode::FOLLOWING      ||
                        cm == RobotMode::MOVING_TO_PANEL ||
                        cm == RobotMode::MANUAL          ||
                        cm == RobotMode::AT_PANEL) {
                        transition(RobotMode::IDLE,
                                   "フォルト検知: " + msg->fault_type);
                    }
                }
            });

        // ── サービス: /mode_manager/set_mode ────────────────
        srv_set_mode_ = create_service<SetMode>(
            "/mode_manager/set_mode",
            std::bind(&ModeManager::handleSetMode, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ── 定期発行タイマー (1 Hz) ─────────────────────────
        pub_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { publishMode(); });

        // ── 初期化完了 → IDLE ───────────────────────────────
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                init_timer_->cancel();
                transition(RobotMode::IDLE, "初期化完了");
            });

        RCLCPP_INFO(get_logger(), "mode_manager 起動");
    }

private:
    // ── 遷移許可マトリクス ────────────────────────────────
    bool isTransitionAllowed(uint8_t from, uint8_t to) {
        // ESTOP はどこからでも OK
        if (to == RobotMode::ESTOP) return true;
        // ESTOP からは IDLE のみ
        if (from == RobotMode::ESTOP)
            return to == RobotMode::IDLE;
        // INIT からは IDLE のみ
        if (from == RobotMode::INIT)
            return to == RobotMode::IDLE;

        switch (from) {
        case RobotMode::IDLE:
            return to == RobotMode::FOLLOWING;

        case RobotMode::FOLLOWING:
            return to == RobotMode::MANUAL          ||
                   to == RobotMode::MOVING_TO_PANEL ||
                   to == RobotMode::IDLE;

        case RobotMode::MOVING_TO_PANEL:
            return to == RobotMode::AT_PANEL ||
                   to == RobotMode::MANUAL   ||
                   to == RobotMode::IDLE;

        case RobotMode::AT_PANEL:
            return to == RobotMode::FOLLOWING ||
                   to == RobotMode::MANUAL    ||
                   to == RobotMode::IDLE;

        case RobotMode::MANUAL:
            return to == RobotMode::FOLLOWING ||
                   to == RobotMode::IDLE;

        default:
            return false;
        }
    }

    void transition(uint8_t target, const std::string& reason) {
        if (current_mode_ == target) return;

        if (!isTransitionAllowed(current_mode_, target)) {
            RCLCPP_WARN(get_logger(),
                "遷移拒否: %s → %s (%s)",
                modeName(current_mode_).c_str(),
                modeName(target).c_str(),
                reason.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "モード遷移: %s → %s [%s]",
            modeName(current_mode_).c_str(),
            modeName(target).c_str(),
            reason.c_str());

        current_mode_ = target;
        publishMode();
    }

    void publishMode() {
        RobotMode msg;
        msg.mode = current_mode_;
        pub_mode_->publish(msg);
    }

    // ── /mode_manager/set_mode サービスハンドラ ──────────
    void handleSetMode(
        const SetMode::Request::SharedPtr  req,
        const SetMode::Response::SharedPtr res)
    {
        // IDLE → FOLLOWING は明示操作のみ許可(タブレットからの要求のみ通す)
        // mode_manager 自身は内部遷移でのみ IDLE→FOLLOWING を禁止するものではないが、
        // 外部からのサービス経由に限り「IDLE→FOLLOWING」を MANUAL からの要求でも許容する
        std::string reason = "set_mode サービス by " + req->requester;
        bool before = isTransitionAllowed(current_mode_, req->requested_mode);

        if (before) {
            transition(req->requested_mode, reason);
            res->success      = true;
            res->message      = "OK";
            res->current_mode = current_mode_;
        } else {
            res->success      = false;
            res->message      = "遷移不可: " +
                                modeName(current_mode_) + " → " +
                                modeName(req->requested_mode);
            res->current_mode = current_mode_;
            RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        }
    }

    // ── メンバ ────────────────────────────────────────────
    uint8_t current_mode_;

    rclcpp::Publisher<RobotMode>::SharedPtr         pub_mode_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
    rclcpp::Subscription<FaultStatus>::SharedPtr         sub_fault_;
    rclcpp::Service<SetMode>::SharedPtr                  srv_set_mode_;
    rclcpp::TimerBase::SharedPtr                         pub_timer_;
    rclcpp::TimerBase::SharedPtr                         init_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManager>());
    rclcpp::shutdown();
    return 0;
}
