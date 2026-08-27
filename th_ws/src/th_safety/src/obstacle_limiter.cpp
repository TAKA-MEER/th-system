// ============================================================
// obstacle_limiter — 最終段の速度リミッタノード (C++)
//
// DetailedDesign-wp2.md `WP-SAFE-03`。判定ロジックは
// th_safety/obstacle_limiter_core.hpp（ROS2 非依存）に置き、このファイルは
// ROS2 との配線（sub/pub/timer）と、ROS の世界からコアの入力型への変換
// だけを行う（CLAUDE.md「追従ロジックの二層構造」と同じ思想。
// safety_monitor.cpp と同じ書き方）。
//
// このパケットの範囲は本ノードのみ。launch 配線・twist_mux.yaml の改名・
// REGISTRY_NODES への登録は別パケット（obstacle_limiter が動く前に配線を
// 変えると /cmd_vel を誰も出さなくなり起動不能になるため）。したがって
// 現時点では registry.yaml 由来の生成 yaml（th_ws/data/generated/
// obstacle_limiter.yaml）が存在しない。ここで declare_parameter に与える
// 既定値は、それが配線されるまでの安全側フォールバック
// （速度上限系はすべて 0.0 = 動かない）であり、実運用値ではない。
//
// トピック・QoS（DetailedDesign-wp2.md WP-SAFE-03 §3.1）:
//   sub /cmd_vel_muxed   (Twist)        reliable, depth 1
//   sub /scan            (LaserScan)    SensorDataQoS（生。/scan_filtered ではない）
//   sub /system/state    (SystemState)  reliable + transient_local, depth 1
//   sub /cmd_vel_manual  (Twist)        reliable, depth 1（値は使わず鮮度だけ見る）
//   sub /safety/estop    (Bool)         reliable
//   sub /safety/fault_lock (Bool)       reliable
//   pub /cmd_vel               (Twist)         reliable, depth 1 — 20Hz固定・沈黙禁止
//   pub /safety/limiter_status (LimiterStatus) best_effort, depth 1 — 20Hz（heartbeat兼用）
//
// TF（DetailedDesign-names.md §1.2・wp2.md §3.4）: base_link<-laser_link の
// 固定変換を起動時に 1 度だけ取得して保持する（20Hz では TF を引かない）。
// 取得に失敗したら起動を失敗させる（wp2.md §3.4 が明記。素通しで動かさない）。
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

#include <th_system_msgs/msg/system_state.hpp>
#include <th_system_msgs/msg/limiter_status.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "th_safety/obstacle_limiter_core.hpp"
#include "th_safety/obstacle_limiter_params.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {

// SystemState.zone ("IN"/"OUT"/"NA") → Zone。未知の値・空文字は NA に倒す
// （§3.3: NA は常に停止側の政策なので、値が読めない場合の安全側と一致する）。
th_safety::Zone zoneFromString(const std::string& z) {
  if (z == "IN") return th_safety::Zone::IN;
  if (z == "OUT") return th_safety::Zone::OUT;
  return th_safety::Zone::NA;
}

const char* sourceClassToString(th_safety::SourceClass c) {
  return c == th_safety::SourceClass::MANUAL ? "MANUAL" : "AUTO";
}

}  // namespace

class ObstacleLimiter : public rclcpp::Node {
public:
    ObstacleLimiter() : Node("obstacle_limiter") {
        // ── パラメータ（DetailedDesign-wp2.md WP-SAFE-03 §3.3） ───────────
        // 速度上限・距離系は registry.yaml では derived/measured（value: null
        // または実測値）。生成 yaml が未配線の間は安全側（0.0 = 動かない）を
        // 既定にする。given 値（registry.yaml に固定値がある行）はその値を
        // 既定にしておく（配線後に上書きされるので実害は無い）。
        declare_parameter("obstacle_floor_distance_m", 0.0);
        declare_parameter("hysteresis_band_m", 0.0);
        declare_parameter("brake_accel_mps2", 1.18);  // registry.yaml 実測値（WP-MEAS-01）
        declare_parameter("obstacle_cone_half_width_rad", 0.5);
        declare_parameter("obstacle_cone_half_width_reverse_rad", 0.6);
        declare_parameter("w_max", 0.6);
        declare_parameter("w_align_max", 0.3);

        declare_parameter("manual_joy_timeout", 1.0);  // 単位: 秒（registry.yaml のパラメータ名のまま）
        declare_parameter("state_stale_ms", 1500);
        declare_parameter("muxed_stale_ms", 200);
        declare_parameter("scan_stale_ms", 300);
        declare_parameter("lock_stale_ms", 500);

        declare_parameter("blind_calibrated", true);
        // blind_angle_ranges: 空配列の**扱いは 2026-08-27 に訂正した**。
        //
        // 当初（b8d8e98）は「rclcpp が空の DOUBLE_ARRAY から型推論できない」
        // ことが原因と考え、ParameterDescriptor で型を明示すれば直ると判断した。
        // これは誤りだった。Docker 実起動で ParameterDescriptor を付けた後も
        // 同じ「parameter_value_from failed for parameter 'blind_angle_ranges':
        // No parameter value set」で起動失敗することを実測で確認した。
        // 本当の原因は **ROS2 Humble の rcl_yaml_param_parser が空配列の
        // parameter override をそもそも解決できない**ことで、
        // declare_parameter 側の型推論の問題ではない（素の
        // tf2_ros::static_transform_publisher に空配列 1 個だけの params
        // ファイルを渡しても同じ例外で落ちることを実測で確認済み。
        // ParameterDescriptor の有無は無関係）。
        //
        // **本当の対処は生成側**（th_bringup/launch/params_generation.py）
        // にある: sanitize_node_params() が空配列のキーを丸ごと落とし、
        // かつて存在した reshape_blind_angles()（空配列を明示的に書き戻す
        // 処理）は撤去した。したがって生成 yaml に blind_angle_ranges が
        // 載るのは実際に値がある（死角がある）ときだけで、ここの override
        // が空配列になることはもう無い。
        //
        // ここでの ParameterDescriptor は**主たる対処ではなく多層防御**として
        // 残す——将来、生成経路以外（手動の --params-file 上書き等）から
        // このパラメータへ空配列 override が渡る事態が万一起きても、型が
        // 明示されていれば少なくとも `declare_parameter` の意図は読み取れる
        // （それでも override が空配列である限り起動失敗は避けられないことに
        // 変わりはない。空配列を override として渡さないことが唯一の正しい
        // 対処）。既定値は空配列（＝死角マスクなし）。マスクしすぎて障害物が
        // 見えなくなるより、マスクせず広く見える方が安全側。
        rcl_interfaces::msg::ParameterDescriptor blind_angle_ranges_desc;
        blind_angle_ranges_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
        declare_parameter("blind_angle_ranges", std::vector<double>{}, blind_angle_ranges_desc);

        // 速度上限の名前→数値表（§3.3.1・「確認済みの事実①」）。
        // v_reverse は ObstacleLimiterParams::v_reverse と表の両方に使う
        // （コアが後退時のキャップに使う値と、AT_PANEL 等が名前で指す値は
        // 同じ registry.yaml の v_reverse なので、パラメータの実体は 1 つ）。
        declare_parameter("v_max", 0.0);
        declare_parameter("v_slow", 0.0);
        declare_parameter("v_reverse", 0.0);
        declare_parameter("v_jog_panel", 0.0);
        // v_check / v_calib / v_leash: 2026-08-27 に registry.yaml の consumers へ
        // obstacle_limiter を追加した（SystemState.speed_limit が "v_check" /
        // "v_calib" / "v_leash" を運びうる。attributes.yaml の OPCHECK / CALIB /
        // LEASH の speed_limit がそれぞれこの名前を指すため、resolve_speed_limit_name()
        // が解決するには表に載っている必要がある）。これで生成 yaml
        // （th_ws/data/generated/obstacle_limiter.yaml）に値が載るようになった。
        // 数値の実体は registry.yaml の責務なので、ここでは他の v_* と同じく
        // 既定値を安全側（0.0 = 停止）にするだけにする（実運用値をコードに
        // 手で書き写さない。書き写すと registry を変えても追従しない——
        // auto_brake が names.md / zones.py / state_manager.py の3箇所で
        // 食い違った N-14 と同じ失敗パターンになる）。
        declare_parameter("v_check", 0.0);
        declare_parameter("v_calib", 0.0);
        declare_parameter("v_leash", 0.0);

        params_.obstacle_floor_distance_m = get_parameter("obstacle_floor_distance_m").as_double();
        params_.hysteresis_band_m = get_parameter("hysteresis_band_m").as_double();
        params_.brake_accel_mps2 = get_parameter("brake_accel_mps2").as_double();
        params_.obstacle_cone_half_width_rad = get_parameter("obstacle_cone_half_width_rad").as_double();
        params_.obstacle_cone_half_width_reverse_rad =
            get_parameter("obstacle_cone_half_width_reverse_rad").as_double();
        params_.v_reverse = get_parameter("v_reverse").as_double();
        params_.w_max = get_parameter("w_max").as_double();
        params_.w_align_max = get_parameter("w_align_max").as_double();
        params_.manual_joy_timeout_sec = get_parameter("manual_joy_timeout").as_double();
        params_.state_stale_sec = get_parameter("state_stale_ms").as_int() / 1000.0;
        params_.muxed_stale_sec = get_parameter("muxed_stale_ms").as_int() / 1000.0;
        params_.scan_stale_sec  = get_parameter("scan_stale_ms").as_int() / 1000.0;
        params_.lock_stale_sec  = get_parameter("lock_stale_ms").as_int() / 1000.0;
        params_.blind_calibrated = get_parameter("blind_calibrated").as_bool();
        params_.blind_angle_ranges_deg = th_safety::flat_to_range_pairs(
            get_parameter("blind_angle_ranges").as_double_array());

        speed_limit_table_ = {
            {"v_max", get_parameter("v_max").as_double()},
            {"v_slow", get_parameter("v_slow").as_double()},
            {"v_reverse", params_.v_reverse},
            {"v_jog_panel", get_parameter("v_jog_panel").as_double()},
            {"v_check", get_parameter("v_check").as_double()},
            {"v_calib", get_parameter("v_calib").as_double()},
            {"v_leash", get_parameter("v_leash").as_double()},
        };

        // ── TF: 起動時に base_link<-laser_link を取得して保持する
        //    （names.md §1.2・wp2.md WP-SAFE-03 §3.4。20Hz では引かない）。
        //    取得に失敗したら起動を失敗させる（wp2.md §3.4 が明記。素通しにしない）。
        //
        // 有界のリトライ（launch 配線時に追加。実装管理者からの指示、2026-08-27）:
        // launch では robot_state_publisher と同時に起動されるため、xacro の
        // Command 実行（数百ms〜数秒かかりうる）が終わって最初の /tf_static が
        // 届く前に本ノードが TF を引きに行く起動レースがある。単発の
        // lookupTransform(timeout) だけに頼ると、Docker実測で「約8msでリトライ
        // 無しに例外を投げた」事例が確認されている（tf2_ros::Buffer の内部の
        // ブロッキング待ちが期待通りに機能しない環境があるということ）。
        // そのため呼び出し側で canTransform() を明示的にポーリングするループに
        // し、実際に一定間隔でリトライすることを保証する。
        // 待ち時間: 全体 tf_lookup_timeout_sec（既定 10.0 秒。xacro 起動 + DDS
        // discovery の遅延を吸収する余裕を持たせた値。5 秒では Docker の重い
        // 起動時に不足する可能性があるため従来の既定値から広げた）を
        // tf_lookup_poll_interval_sec（既定 0.2 秒）刻みでポーリングする。
        // 「待っても取れなければ起動失敗」なので wp2.md §3.4 の要求
        // （素通しで動かさない）は変わらず満たす。
        const double tf_timeout_sec = declare_parameter("tf_lookup_timeout_sec", 10.0);
        const double tf_poll_interval_sec = declare_parameter("tf_lookup_poll_interval_sec", 0.2);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        // spin_thread=true: このリスナ専用の実行スレッドを持たせる。main() の
        // executor がまだ spin していない起動シーケンス中でも /tf, /tf_static
        // のコールバックが処理される。
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

        bool tf_ready = false;
        std::string tf_error;
        int tf_attempts = 0;
        const auto tf_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(tf_timeout_sec));
        do {
            ++tf_attempts;
            tf_error.clear();
            if (tf_buffer_->canTransform(
                    "base_link", "laser_link", tf2::TimePointZero, &tf_error)) {
                tf_ready = true;
                break;
            }
            if (std::chrono::steady_clock::now() >= tf_deadline) {
                break;
            }
            RCLCPP_WARN(get_logger(),
                "base_link<-laser_link TF 未取得（%d 回目、%.1fs 刻みでリトライ継続）: %s",
                tf_attempts, tf_poll_interval_sec, tf_error.c_str());
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(tf_poll_interval_sec)));
        } while (std::chrono::steady_clock::now() < tf_deadline);

        if (!tf_ready) {
            RCLCPP_FATAL(get_logger(),
                "起動時の base_link<-laser_link TF 取得に %.1f 秒(%d 回試行)待っても"
                "失敗した。素通しでは動かさず起動を失敗させる（wp2.md WP-SAFE-03 §3.4）: %s",
                tf_timeout_sec, tf_attempts, tf_error.c_str());
            throw std::runtime_error(
                "obstacle_limiter: base_link<-laser_link TF unavailable at startup");
        }
        try {
            laser_to_base_ = tf_buffer_->lookupTransform(
                "base_link", "laser_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            // canTransform 直後の lookupTransform が失敗するのは通常起きない
            // （TF バッファから消える猶予はあるが、直前に確認したばかりのため）。
            // 想定外の状態として同じくFATALで起動失敗にする（素通ししない）。
            RCLCPP_FATAL(get_logger(),
                "canTransform は成功したが直後の lookupTransform が失敗した（想定外）: %s",
                ex.what());
            throw std::runtime_error(
                "obstacle_limiter: base_link<-laser_link TF lookup failed "
                "immediately after a successful canTransform");
        }
        const double yaw = tf2::getYaw(laser_to_base_.transform.rotation);
        if (std::fabs(yaw) > 1e-6) {
            // 既知の制約: obstacle_limiter_core は前方/後退の判定コーン方向を
            // laser_link 基準の 0 / pi に固定しており、base_link<-laser_link の
            // 回転成分を補正する入力を持たない（現在の URDF は
            // th_description/urdf/th_robot.urdf.xacro の laser_joint が rpy=0
            // ＝並進のみなので、今は影響が無い）。回転付きの搭載に変わった
            // ときはコア側の拡張が必要になる（このノードの改修だけでは足りない）。
            RCLCPP_WARN(get_logger(),
                "laser_link は base_link に対し yaw=%.4f rad 回転している構成を検知したが、"
                "現在の obstacle_limiter_core は回転補正を行わない（既知の制約）", yaw);
        } else {
            RCLCPP_INFO(get_logger(), "base_link<-laser_link TF 取得完了 (yaw=%.4f rad)", yaw);
        }

        // ── Publishers ─────────────────────────────────────
        pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(1).reliable());
        pub_limiter_status_ = create_publisher<th_system_msgs::msg::LimiterStatus>(
            "/safety/limiter_status", rclcpp::QoS(1).best_effort());

        // ── Subscribers（コールバックは最新値を保持するだけ。判定はしない） ──
        sub_muxed_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_muxed", rclcpp::QoS(1).reliable(),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                muxed_.received = true;
                muxed_.stamp_sec = now().seconds();
                muxed_.value.linear_x = msg->linear.x;
                muxed_.value.angular_z = msg->angular.z;
            });

        sub_manual_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_manual", rclcpp::QoS(1).reliable(),
            [this](const geometry_msgs::msg::Twist::SharedPtr) {
                // §3.2: MANUAL/AUTO の分類には値を使わず鮮度だけを見る。
                manual_.received = true;
                manual_.stamp_sec = now().seconds();
            });

        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_.received = true;
                scan_.stamp_sec = now().seconds();
                scan_.geometry.angle_min = msg->angle_min;
                scan_.geometry.angle_increment = msg->angle_increment;
                scan_.geometry.num_ranges = msg->ranges.size();
                scan_.ranges.assign(msg->ranges.begin(), msg->ranges.end());
            });

        sub_state_ = create_subscription<th_system_msgs::msg::SystemState>(
            "/system/state", rclcpp::QoS(1).reliable().transient_local(),
            [this](const th_system_msgs::msg::SystemState::SharedPtr msg) {
                state_.received = true;
                state_.stamp_sec = now().seconds();
                state_.mode = msg->mode;
                state_.jog_active = msg->jog_active;
                state_.zone = zoneFromString(msg->zone);
                state_.auto_brake = msg->auto_brake;
                latest_speed_limit_name_ = msg->speed_limit;
            });

        sub_estop_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/estop", rclcpp::QoS(1).reliable(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                estop_.received = true;
                estop_.stamp_sec = now().seconds();
                estop_.value = msg->data;
            });

        sub_fault_lock_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/fault_lock", rclcpp::QoS(1).reliable(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                fault_lock_.received = true;
                fault_lock_.stamp_sec = now().seconds();
                fault_lock_.value = msg->data;
            });

        // ── 20Hz 固定タイマ（§3.1・§3.4.2。入力が全部 stale でも沈黙しない） ──
        timer_ = create_wall_timer(50ms, std::bind(&ObstacleLimiter::tick, this));

        RCLCPP_INFO(get_logger(), "obstacle_limiter 起動");
    }

private:
    void tick() {
        th_safety::ObstacleLimiterInputs in;
        in.now_sec = now().seconds();
        in.muxed = muxed_;
        in.manual = manual_;
        in.scan = scan_;
        in.state = state_;
        in.estop = estop_;
        in.fault_lock = fault_lock_;

        const auto resolved = th_safety::resolve_speed_limit_name(
            latest_speed_limit_name_, speed_limit_table_);
        if (resolved.unknown) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "SystemState.speed_limit の名前 '%s' がパラメータ表に無い。"
                "安全側(0.0=停止)に倒す", latest_speed_limit_name_.c_str());
        }
        // N-15: /system/state.speed_limit は state_manager の
        // combine_speed_limits()（th_state/th_state/zones.py）が「画面由来」と
        // 「モード由来」のうち厳しい方をすでに合成した 1 本の名前として届く
        // （SystemState.msg のフィールドコメントは N-13 時点のままで
        // 「画面由来」としか書いていないが、実装は N-15 で合成済みに変わって
        // いる — .msg 側のコメント更新漏れ。実装報告に記載）。
        // obstacle_limiter_core は screen_limit_mps / mode_limit_mps の 2 系統を
        // min() で合成する設計だが、このノードが持つ信号は合成済みの 1 本しか
        // 無いため、同じ解決値を両方に渡す（min(x,x)=x で結果は変わらない）。
        // 片方だけを +infinity にする案も検討したが、「実際には 1 系統しか
        // 届いていない」という事実を隠すより、同じ値を渡して意味的に
        // 「両方とも合成済みの上限」と明示する方が読み手に誤解が少ないと判断した。
        in.screen_limit_mps = resolved.value_mps;
        in.mode_limit_mps = resolved.value_mps;

        const auto out = core_.update(in, params_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = out.out.linear_x;
        cmd.angular.z = out.out.angular_z;
        pub_cmd_vel_->publish(cmd);

        th_system_msgs::msg::LimiterStatus status;
        status.header.stamp = now();
        status.alive = true;
        status.action = th_safety::limiter_action_to_string(out.action);
        status.in_linear = static_cast<float>(in.muxed.value.linear_x);
        status.out_linear = static_cast<float>(out.out.linear_x);
        status.nearest_obstacle_m = static_cast<float>(out.nearest_obstacle_m);
        status.source_class = sourceClassToString(out.source_class);
        status.applied_limit_mps = static_cast<float>(out.applied_limit_mps);
        pub_limiter_status_->publish(status);
    }

    // ── パラメータ ────────────────────────────────────────
    th_safety::ObstacleLimiterParams params_;
    std::map<std::string, double> speed_limit_table_;
    std::string latest_speed_limit_name_ = "stop";  // 未受信の既定は安全側(停止)

    // ── TF（起動時に 1 度だけ取得。20Hz では引かない） ───────────
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped laser_to_base_;

    // ── 判定コア（ROS2 非依存） ───────────────────────────────
    th_safety::ObstacleLimiterCore core_;

    // ── 最新入力（コールバックが保持するだけ。判定は tick() が呼ぶ core_.update() ──
    th_safety::Stamped<th_safety::Twist2D> muxed_;
    th_safety::Stamped<th_safety::Twist2D> manual_;
    th_safety::ScanSnapshot scan_;
    th_safety::SystemStateSnapshot state_;
    th_safety::Stamped<bool> estop_;
    th_safety::Stamped<bool> fault_lock_;

    // ── Publishers ────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<th_system_msgs::msg::LimiterStatus>::SharedPtr pub_limiter_status_;

    // ── Subscribers ──────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_muxed_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_manual_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<th_system_msgs::msg::SystemState>::SharedPtr sub_state_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_fault_lock_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ObstacleLimiter>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("obstacle_limiter"), "起動失敗: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
