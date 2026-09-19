// ============================================================
// jog_gate — 手動ジョグ指令のゲートノード (C++)
//
// DetailedDesign-wp2.md `WP-SAFE-04`。判定ロジックは
// th_safety/jog_gate_core.hpp（ROS2 非依存）に置き、このファイルは
// ROS2 との配線（sub/pub）だけを行う（obstacle_limiter.cpp と同じ書き方）。
//
// /cmd_vel_manual_raw（WebUI が rosbridge 直に publish）を購読し、
// /system/state の鮮度と attributes.yaml の jog 列・除外表で判定して、
// 通すときだけ /cmd_vel_manual（twist_mux priority 30）へそのまま転送する。
// 通さないときは**何も publish しない**（沈黙）。ゼロを撃たない（J-1）。
//
// 入力駆動（/cmd_vel_manual_raw を受けたときだけ判定・転送。固定レートで撃たない）。
//
// トピック・QoS（DetailedDesign-wp2.md WP-SAFE-04 §3.1）:
//   sub /cmd_vel_manual_raw (Twist)       reliable, depth 1 — 10Hz（UI）
//   sub /system/state       (SystemState) reliable + transient_local, depth 1 — 10Hz
//   pub /cmd_vel_manual     (Twist)       reliable, depth 1 — 入力駆動。通さないときは沈黙
//
// attributes.yaml（th_state と同じファイル。J-4）は ament_index_cpp で指す
// th_state の share/config を読む。テストでは ROS パラメータ
// attributes_yaml_path で差し替えられる（詳細設計 §3.3 の「テストで override」）。
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <th_system_msgs/msg/system_state.hpp>

#include "th_safety/jog_gate_core.hpp"

#include <string>

class JogGate : public rclcpp::Node {
public:
    JogGate() : Node("jog_gate") {
        // ── パラメータ（DetailedDesign-wp2.md WP-SAFE-04 §3.3） ───────────
        // state_stale_ms は registry.yaml（value: 1500）由来。生成 yaml
        // （generated/jog_gate.yaml）が配線されるまでは安全側の既定値
        // （/system/state が途絶したら短時間で沈黙）を置く。
        declare_parameter("state_stale_ms", 1500);
        state_stale_sec_ =
            get_parameter("state_stale_ms").as_int() / 1000.0;

        // attributes.yaml のパス。テストではこのパラメータで差し替える
        // （詳細設計 §3.3。既定は th_state の share/config）。
        declare_parameter("attributes_yaml_path", "");
        const std::string attrs_path = get_parameter("attributes_yaml_path").as_string().empty()
            ? th_state_attributes_path()
            : get_parameter("attributes_yaml_path").as_string();

        // J-4: th_state の jog_allowed と同じ attributes.yaml を読む。
        // 読めなければ起動を失敗させる（通しっぱなしにしない。§4.3 の安全側）。
        try {
            attrs_ = th_safety::load_attributes_jog(attrs_path);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(),
                "attributes.yaml を読み込めなかったため起動を失敗させる"
                "（素通しでジョグを通さない）: %s", e.what());
            throw std::runtime_error(
                "jog_gate: failed to load attributes.yaml: " + attrs_path);
        }
        RCLCPP_INFO(get_logger(),
            "attributes.yaml を読み込みました（mode=%zu）: %s",
            attrs_.jog.size(), attrs_path.c_str());

        // ── Publishers ─────────────────────────────────────
        pub_manual_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", rclcpp::QoS(1).reliable());

        // ── Subscribers ────────────────────────────────────
        sub_state_ = create_subscription<th_system_msgs::msg::SystemState>(
            "/system/state", rclcpp::QoS(1).reliable().transient_local(),
            [this](const th_system_msgs::msg::SystemState::SharedPtr msg) {
                state_.received = true;
                state_.stamp_sec = now().seconds();
                state_.mode = msg->mode;
                state_.state = msg->state;
            });

        // 入力駆動。/cmd_vel_manual_raw を受けたときだけ判定して、通すとき
        // だけ転送する。通さないときは何も publish しない（J-1・J-2）。
        sub_raw_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual_raw", rclcpp::QoS(1).reliable(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                if (!jog_passes_now()) {
                    return;  // 沈黙。ゼロを撃たない（J-1）
                }
                // J-3: 速度の大きさは変えない（ゲートであってリミッタではない）
                pub_manual_->publish(*msg);
            });

        RCLCPP_INFO(get_logger(), "jog_gate 起動（state_stale_ms=%ld）",
            static_cast<long>(get_parameter("state_stale_ms").as_int()));
    }

private:
    // /system/state の鮮度と attributes.yaml・除外表で判定する（§4.1 の 3 条件）。
    bool jog_passes_now() {
        th_safety::JogGateStateView st;
        st.received = state_.received;
        st.mode = state_.mode;
        st.state = state_.state;
        st.state_age_sec = state_.received
            ? (now().seconds() - state_.stamp_sec) : 0.0;

        th_safety::JogGateParams p;
        p.state_stale_sec = state_stale_sec_;

        const bool pass = th_safety::jog_passes(st, attrs_, p);
        if (!pass) {
            RCLCPP_DEBUG(get_logger(),
                "jog_gate 閉（mode=%s state=%s received=%d age=%.3fs）→ 沈黙",
                st.mode.c_str(), st.state.c_str(), st.received,
                st.state_age_sec);
        }
        return pass;
    }

    // th_state パッケージの share/config/attributes.yaml へのパス（J-4）。
    static std::string th_state_attributes_path() {
        return ament_index_cpp::get_package_share_directory("th_state")
            + "/config/attributes.yaml";
    }

    // ── パラメータ ────────────────────────────────────────
    double state_stale_sec_ = 0.0;
    th_safety::Attributes attrs_;

    // ── /system/state の最新値（コールバックが保持するだけ） ──
    struct {
        bool received = false;
        double stamp_sec = 0.0;
        std::string mode;
        std::string state;
    } state_;

    // ── Publishers / Subscribers ──────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_manual_;
    rclcpp::Subscription<th_system_msgs::msg::SystemState>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_raw_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<JogGate>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("jog_gate"), "起動失敗: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
