#!/usr/bin/env python3
"""
face_planner.py — ROS2 ノード本体
=====================================
face_planner_core.py のロジックを ROS2 に接続する。
FACING モード(超信地旋回のみで正対・展示用)専用。並進は常に 0。

costmap・Nav2・SLAM 地図には依存しない(地図の無いブースでも成立する)。ただし
odom→base_link の TF(EKF が発行)は必要: 自機の自己移動補償(下記)に使う。

自己移動補償(2026-08-07 追加): DR-SPAAM は約2Hz(0.5秒間隔)でしか推論されないため、
/person/status の (x, y) は検知時点の base_link 相対値のまま次の検知まで更新されない。
FACING は検知の合間も旋回し続けるモードのため、これをそのまま使うと「自機が既にどれだけ
向き直したか」を制御ループが把握できず、次の検知が届くまで盲目的に同じ角速度指令を出し
続けて行き過ぎる(オーバーシュート → 逆方向へ補正 → 発振)。follow_planner_mapless.py と
同じ方式(検知時に odom 絶対座標へ固定し、毎周期その時点の自己位置で相対座標へ引き直す)
で補償する。TF が取れない間は補償前の生値にフォールバックする(follow_planner_mapless.py
と同じ縮退動作)。

担当:
  - /person/status  購読 → ベアリング入力
  - /robot/mode     購読 → FACING 中のみ動作
  - TF (odom→base_link) → 自己移動補償
  - /cmd_vel_retreat パブリッシュ(linear.x は常に 0)
  - /follow/status  パブリッシュ(planner="face"。follow_planner / follow_planner_mapless と
    共有するトピックのため、自モードでない間は黙る規律を同様に守る)
"""

import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from th_system_msgs.msg import RobotMode, PersonStatus, FollowStatus

import tf2_ros

from th_planning.face_planner_core import FaceParams, FaceCore
from th_planning.follow_planner_core import to_absolute, to_relative


class FacePlanner(Node):
    def __init__(self):
        super().__init__("face_planner")
        self._declare_all_params()
        p = self._load_params()
        self._core = FaceCore(p)
        self.add_on_set_parameters_callback(self._on_set_params)
        self._current_mode = RobotMode.IDLE
        self._person_pos    = None   # (x, y) 検知時点の base_link 相対値
        self._person_abs    = None   # 検知時点の odom 絶対座標(自己移動補償用)
        self._confidence    = 0.0
        self._person_lost   = True
        self._lost_since: float | None = None

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_retreat = self.create_publisher(Twist, "/cmd_vel_retreat", 10)
        self._pub_status  = self.create_publisher(FollowStatus, "/follow/status", 10)
        self.create_subscription(RobotMode,    "/robot/mode",    self._cb_mode,   10)
        self.create_subscription(PersonStatus, "/person/status", self._cb_person, 10)

        # 状態 publish の間引き用。follow_planner / follow_planner_mapless と同じ方針
        # (変化時 + 1Hz のハートビート)。3 プランナで /follow/status を共有するため、
        # 自モードでない間は黙り、離脱直後の 1 回だけ INACTIVE を出す。
        self._last_status_key  = None
        self._last_status_time = 0.0
        self._status_active    = False
        hz = self.get_parameter("update_rate_hz").value
        self._timer = self.create_timer(1.0 / hz, self._loop)
        self.get_logger().info("face_planner 起動(超信地旋回のみ・展示用)")

    STATUS_HEARTBEAT_SEC = 1.0

    def _publish_status(self, state: str, reason: str, person_distance: float = -1.0):
        key = (state, reason)
        now = self.get_clock().now().nanoseconds * 1e-9
        if key == self._last_status_key and (now - self._last_status_time) < self.STATUS_HEARTBEAT_SEC:
            return
        self._last_status_key  = key
        self._last_status_time = now

        msg = FollowStatus()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = "base_link"
        msg.planner          = "face"
        msg.state            = state
        msg.reason           = reason
        msg.person_distance  = float(person_distance)
        msg.linear_x         = 0.0
        msg.has_escape_angle = False
        msg.escape_angle     = 0.0
        self._pub_status.publish(msg)

    def _declare_all_params(self):
        D = self.declare_parameter
        D("k_ang",                      2.0)
        D("w_max_rad_s",                0.5)
        D("max_angular_accel_rad_s2",   3.0)
        D("max_angular_decel_rad_s2",   6.0)
        D("angle_tolerance_rad",       0.05)
        D("min_confidence",             0.5)
        D("lost_debounce_sec",          0.3)
        D("update_rate_hz",            10.0)
        D("odom_frame",           "odom")
        D("base_frame",           "base_link")

    def _load_params(self) -> FaceParams:
        g = self.get_parameter
        return FaceParams(
            k_ang                     = g("k_ang").value,
            w_max_rad_s                = g("w_max_rad_s").value,
            max_angular_accel_rad_s2   = g("max_angular_accel_rad_s2").value,
            max_angular_decel_rad_s2   = g("max_angular_decel_rad_s2").value,
            angle_tolerance_rad        = g("angle_tolerance_rad").value,
            min_confidence              = g("min_confidence").value,
            lost_debounce_sec           = g("lost_debounce_sec").value,
            update_rate_hz               = g("update_rate_hz").value,
        )

    def _on_set_params(self, params):
        # face_planner_core.py の計算式で除数として使われる値 (update_rate_hz) に
        # 0 以下を許すとゼロ除算で FACING 中にノードがクラッシュするため拒否する
        # (follow_planner_mapless.py の同ガードと同じ理由)。
        for p in params:
            if p.name in self._POSITIVE_PARAMS and p.value <= 0:
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} は正の値である必要があります")
        for p in params:
            if hasattr(self._core.params, p.name):
                setattr(self._core.params, p.name, p.value)
        return SetParametersResult(successful=True)

    _POSITIVE_PARAMS = frozenset({
        "k_ang", "w_max_rad_s", "max_angular_accel_rad_s2", "max_angular_decel_rad_s2",
        "angle_tolerance_rad", "min_confidence", "lost_debounce_sec",
        "update_rate_hz",
    })

    def _cb_mode(self, msg: RobotMode):
        prev = self._current_mode
        self._current_mode = msg.mode
        if prev == RobotMode.FACING and msg.mode != RobotMode.FACING:
            self._core.reset()
            self._stop()

    def _cb_person(self, msg: PersonStatus):
        was_lost = self._person_lost
        self._person_lost = msg.is_lost

        if msg.is_lost:
            if not was_lost:
                self._lost_since = self.get_clock().now().nanoseconds * 1e-9
            return

        self._person_pos = (msg.position.x, msg.position.y)
        self._confidence  = msg.confidence
        # 受信時点の自己位置で odom 絶対座標に固定する。DR-SPAAM は約2Hzでしか
        # 更新されないため、base_link 相対のまま次の検知まで保持すると、その間
        # 旋回し続ける FACING では「どれだけ向き直したか」を見失い、次の検知まで
        # 同じ角速度を出し続けて行き過ぎる(follow_planner_mapless.py と同じ理由)。
        pose = self._robot_pose_odom()
        self._person_abs = to_absolute(pose, self._person_pos) if pose is not None else None
        self._lost_since = None

    def _robot_pose_odom(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self.get_parameter("odom_frame").value,
                self.get_parameter("base_frame").value,
                rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().debug(
                f"odom TF 取得失敗: {e}", throttle_duration_sec=2.0)
            return None
        trans = tf.transform.translation
        rot   = tf.transform.rotation
        yaw   = 2.0 * math.atan2(rot.z, rot.w)
        return (trans.x, trans.y, yaw)

    def _stop(self):
        self._pub_retreat.publish(Twist())

    def _loop(self):
        if self._current_mode != RobotMode.FACING:
            # 自モードでない間は黙る(follow_planner_mapless.py の同じ箇所を参照)。
            # モードを抜けた直後の 1 回だけ INACTIVE を出して残留状態を解除する。
            if self._status_active:
                self._status_active = False
                self._publish_status("INACTIVE", "inactive")
            return
        self._status_active = True

        now = self.get_clock().now().nanoseconds * 1e-9
        lost_elapsed = (now - self._lost_since) if self._lost_since is not None else 0.0

        if self._person_pos is None:
            # まだ一度も検知していない
            self._stop()
            self._publish_status("STOPPED", "person_lost")
            return

        # 自己移動補償: 検知間隔中も旋回し続けるため、固定した odom 絶対位置を
        # 現在の自己位置で相対座標に引き直す。TF が取れない間は生値にフォール
        # バックする(follow_planner_mapless.py と同じ縮退動作)。
        px, py = self._person_pos
        robot_pose_odom = self._robot_pose_odom()
        compensated = self._person_abs is not None and robot_pose_odom is not None
        if compensated:
            px, py = to_relative(robot_pose_odom, self._person_abs)

        out = self._core.update(
            person_x=px, person_y=py, confidence=self._confidence,
            is_lost=self._person_lost, lost_elapsed_sec=lost_elapsed)

        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = out.angular_z
        # ロスト・低confidence中も含め毎周期明示発行する。直前の非ゼロ指令が
        # /cmd_vel_retreat に残ったまま停止しない事態を防ぐため
        # (follow_planner_mapless.py と同じ理由)。
        self._pub_retreat.publish(cmd)

        # TF が取れず自己移動補償なしの生値で旋回した周期は reason を上書きする。
        # 補償の有無を見た目上区別できないと、実機チューニング中に
        # 「補償あり/なしのどちらの挙動を見ているか」を誤認しかねないため。
        reason = out.reason
        if not compensated and out.reason in ("tracking", "aligned"):
            reason = "no_odom"

        state = "STOPPED" if out.reason in ("person_lost", "low_confidence") else \
                ("ALIGNED" if out.reason == "aligned" else "TRACKING")
        self._publish_status(state, reason, person_distance=math.hypot(px, py))


def main(args=None):
    rclpy.init(args=args)
    node = FacePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
