// ============================================================
// th_esp32_bridge — esp32_bridge ノード (C++)
//
// 役割:
//   1. /cmd_vel (Twist) → /wheel_cmd (Float64MultiArray) 変換
//   2. /wheel_feedback (Float64MultiArray) → /odom + TF 計算
//   3. /estop_hw (Bool) を /safety/estop_hw へ中継
//   4. /esp32/wheel_feedback (WheelFeedback) 発行 (他ノード用)
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <th_system_msgs/msg/wheel_feedback.hpp>

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class Esp32Bridge : public rclcpp::Node {
public:
    Esp32Bridge() : Node("esp32_bridge") {
        // ── パラメータ ──────────────────────────────────────
        declare_parameter("wheel_radius",  0.0391);   // m
        declare_parameter("wheel_base",    0.39);     // m
        declare_parameter("odom_frame",    std::string("odom"));
        declare_parameter("base_frame",    std::string("base_link"));
        declare_parameter("publish_tf",    true);
        declare_parameter("feedback_timeout_ms", 500);

        wheel_radius_ = get_parameter("wheel_radius").as_double();
        wheel_base_   = get_parameter("wheel_base").as_double();
        odom_frame_   = get_parameter("odom_frame").as_string();
        base_frame_   = get_parameter("base_frame").as_string();
        publish_tf_   = get_parameter("publish_tf").as_bool();

        // ── Publishers ─────────────────────────────────────
        // → ESP32 (micro-ROS が購読)
        pub_wheel_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "wheel_cmd", 10);
        // → Nav2 / robot_localization
        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(
            "odom", 10);
        // → 他 ROS2 ノード用
        pub_wheel_feedback_ = create_publisher<th_system_msgs::msg::WheelFeedback>(
            "/esp32/wheel_feedback", 10);
        // → safety_monitor
        pub_estop_hw_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/estop_hw", 10);

        // ── TF broadcaster ─────────────────────────────────
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ── Subscribers ────────────────────────────────────
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Esp32Bridge::cbCmdVel, this, std::placeholders::_1));

        sub_wheel_feedback_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_feedback", 10,
            std::bind(&Esp32Bridge::cbWheelFeedback, this, std::placeholders::_1));

        sub_estop_hw_ = create_subscription<std_msgs::msg::Bool>(
            "estop_hw", 10,
            std::bind(&Esp32Bridge::cbEstopHw, this, std::placeholders::_1));

        // ── フィードバック死活監視タイマー ─────────────────
        int timeout_ms = get_parameter("feedback_timeout_ms").as_int();
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(timeout_ms),
            std::bind(&Esp32Bridge::cbWatchdog, this));

        last_feedback_time_ = now();
        odom_x_ = odom_y_ = odom_yaw_ = 0.0;
        last_odom_time_ = now();

        RCLCPP_INFO(get_logger(), "esp32_bridge 起動 wheel_radius=%.4f wheel_base=%.3f",
                    wheel_radius_, wheel_base_);
    }

private:
    // ── /cmd_vel → /wheel_cmd 変換 ────────────────────────
    void cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v  = msg->linear.x;   // 並進速度 (m/s)
        double w  = msg->angular.z;  // 角速度 (rad/s)
        double vR = v + (w * wheel_base_ / 2.0);
        double vL = v - (w * wheel_base_ / 2.0);

        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {vL, vR};
        pub_wheel_cmd_->publish(cmd);
    }

    // ── /wheel_feedback → /odom + TF ──────────────────────
    void cbWheelFeedback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        last_feedback_time_ = now();
        esp32_alive_ = true;

        double vL = msg->data[0];
        double vR = msg->data[1];
        auto   t  = now();
        double dt = (t - last_odom_time_).seconds();
        last_odom_time_ = t;

        if (dt <= 0.0 || dt > 1.0) return;

        // ── 差動駆動オドメトリ更新 ──────────────────────────
        double v_center = (vL + vR) / 2.0;
        double omega    = (vR - vL) / wheel_base_;
        double dtheta   = omega * dt;

        odom_x_   += v_center * std::cos(odom_yaw_ + dtheta / 2.0) * dt;
        odom_y_   += v_center * std::sin(odom_yaw_ + dtheta / 2.0) * dt;
        odom_yaw_ += dtheta;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, odom_yaw_);

        // ── /odom 発行 ───────────────────────────────────────
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = t;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;
        odom.pose.pose.position.x    = odom_x_;
        odom.pose.pose.position.y    = odom_y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x  = v_center;
        odom.twist.twist.angular.z = omega;

        // 共分散 (暫定値 — 実機キャリブ後に調整)
        odom.pose.covariance[0]  = 0.01;
        odom.pose.covariance[7]  = 0.01;
        odom.pose.covariance[35] = 0.05;
        odom.twist.covariance[0]  = 0.01;
        odom.twist.covariance[35] = 0.05;

        pub_odom_->publish(odom);

        // ── TF: odom → base_link ─────────────────────────────
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp    = t;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id  = base_frame_;
            tf.transform.translation.x = odom_x_;
            tf.transform.translation.y = odom_y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tf);
        }

        // ── /esp32/wheel_feedback (カスタム型) 発行 ────────────
        th_system_msgs::msg::WheelFeedback fb;
        fb.header.stamp = t;
        fb.header.frame_id = base_frame_;
        fb.left_speed  = vL;
        fb.right_speed = vR;
        pub_wheel_feedback_->publish(fb);
    }

    // ── E-Stop 中継 ───────────────────────────────────────
    void cbEstopHw(const std_msgs::msg::Bool::SharedPtr msg) {
        pub_estop_hw_->publish(*msg);
    }

    // ── フィードバック死活監視 ────────────────────────────
    void cbWatchdog() {
        double elapsed = (now() - last_feedback_time_).seconds();
        double timeout = get_parameter("feedback_timeout_ms").as_int() / 1000.0;
        if (esp32_alive_ && elapsed > timeout) {
            RCLCPP_WARN(get_logger(), "ESP32 wheel_feedback タイムアウト (%.1f s)", elapsed);
            esp32_alive_ = false;
            // safety_monitor が /esp32/wheel_feedback の途絶を別途検知するため
            // ここでは警告のみ（二重処理を避ける）
        }
    }

    // ── メンバ ────────────────────────────────────────────
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  pub_wheel_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           pub_odom_;
    rclcpp::Publisher<th_system_msgs::msg::WheelFeedback>::SharedPtr pub_wheel_feedback_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               pub_estop_hw_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr             sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr      sub_wheel_feedback_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                   sub_estop_hw_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    double wheel_radius_, wheel_base_;
    std::string odom_frame_, base_frame_;
    bool publish_tf_;

    double odom_x_, odom_y_, odom_yaw_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_feedback_time_;
    bool esp32_alive_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Esp32Bridge>());
    rclcpp::shutdown();
    return 0;
}
