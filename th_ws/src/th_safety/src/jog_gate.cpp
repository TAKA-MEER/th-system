// ============================================================
// jog_gate — 手動ジョグ指令のゲートノード (C++)
//
// DetailedDesign-wp2.md `WP-SAFE-04`。判定ロジックは
// th_safety/jog_gate_core.hpp（ROS2 非依存）に置き、このファイルは
// ROS2 との配線（sub/pub）だけを行う（obstacle_limiter.cpp と同じ書き方）。
//
// /cmd_vel_manual_raw（WebUI が rosbridge 直に publish。-1〜1 の比率）を購読し、
// /system/state の鮮度と attributes.yaml の jog 列・除外表で判定して、
// 通すときだけ比率に手動ジョグ上限（v_jog_max / w_jog_max）を掛けて
// /cmd_vel_manual（twist_mux priority 30）へ出す。
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

        // W-07: 手動ジョグ専用の上限（registry.yaml の v_jog_max / w_jog_max
        // 由来。生成 yaml が配線されるまでは registry と同じ値＝変更前の
        // 実速度を保つ既定値を置く）。
        declare_parameter("v_jog_max", 0.55);
        declare_parameter("w_jog_max", 1.0);
        limits_.v_jog_max = get_parameter("v_jog_max").as_double();
        limits_.w_jog_max = get_parameter("w_jog_max").as_double();

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
        // だけ比率に上限を掛けて転送する。通さないときは何も publish しない（J-1・J-2）。
        sub_raw_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual_raw", rclcpp::QoS(1).reliable(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                if (!jog_passes_now()) {
                    return;  // 沈黙。ゼロを撃たない（J-1）
                }
                // W-07: 入力は -1〜1 の比率。範囲外は ±1 に丸めて上限を掛ける
                // （jog_apply_limits。大きな値を受けても上限を超えない）。
                // linear.y/z・angular.x/y は UI が送らない（常に 0）ので写すだけ。
                th_safety::JogRatio ratio;
                ratio.vx_ratio = msg->linear.x;
                ratio.wz_ratio = msg->angular.z;
                const th_safety::JogCmd cmd =
                    th_safety::jog_apply_limits(ratio, limits_);
                geometry_msgs::msg::Twist out(*msg);
                out.linear.x = cmd.vx;
                out.angular.z = cmd.wz;
                pub_manual_->publish(out);
            });

        RCLCPP_INFO(get_logger(),
            "jog_gate 起動（state_stale_ms=%ld v_jog_max=%.3f w_jog_max=%.3f）",
            static_cast<long>(get_parameter("state_stale_ms").as_int()),
            limits_.v_jog_max, limits_.w_jog_max);
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
    th_safety::JogSpeedLimits limits_;
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
