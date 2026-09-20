#!/usr/bin/env python3
"""localization_health.py — localization_health_core を ROS2 に繋ぐ薄いノード（WP-SAFE-05）。

自己位置推定の健全性を 1 Hz で `/safety/localization_health`
（LocalizationHealth）に publish する。止まったこと自体が異常の合図になる
（LimiterStatus と同じ流儀。heartbeat 兼用）。

判定の中身は `localization_health_core.evaluate()`（A・C）と
`detect_jump()`（B′。ROS2 非依存の純粋関数）が持つ。
このノードは次だけを行う:

  1. `map→odom` TF のスタンプと値（x, y, yaw）を集める
     （非ブロッキング。route_recorder.py と同じ流儀）
  2. 推定ノードの有無を `get_node_names()` で照合する（connectivity_checker.py と同じ流儀）
  3. 比較周期（jump_window_ms）ごとに `evaluate()`＋`detect_jump()` を呼び、
     結果を publish する。publish 周期と比較周期は同一（B′ の定義どおり）
  4. ok/reason の変化をログへ出す（あとから「いつ見失ったか」を追えるようにする）

publisher の置き場の決定（brief-SAFE-05 が委任）: th_state の新ノード。
slam_control.py ではなく新設する。理由: (1) slam_control は地図操作の仲介で
あり健全性監視の責務ではない。TF 鮮度を見るために tf2 依存を足すと責務が肥大化する
(2) th_config_manager は新設計で廃止予定（names.md §1）。そこへ新ノードを
足さない (3) connectivity_checker と同じ「1 Hz・evaluate・状態トピック」の
形にでき、th_state 内で対称になる。
"""
import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from th_system_msgs.msg import LocalizationHealth

from th_state.localization_health_core import (
    REASON_JUMP,
    Params,
    TransformSample,
    detect_jump,
    evaluate,
)


def _yaw_from_quat(q) -> float:
    """クォータニオンから yaw [rad] を返す（平面移動だけ見る。B′）。

    slam_control.py の _yaw_from_quat と同じ式（重複だがノード間で共有
    モジュールを新設するほどの規模ではない。同ファイルのコメント参照）。
    """
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationHealthNode(Node):

    def __init__(self, **kwargs):
        # kwargs はそのまま rclpy.node.Node へ渡す（テストが parameter_overrides=[...] を
        # 注入できるようにする。connectivity_checker.py / state_manager.py と同じ流儀）。
        super().__init__('localization_health', **kwargs)

        # --- R2: 裸の数値を書かない。既定値なしで宣言し、外部から必ず渡す ---
        # 生成ファイル（GENERATED_DIR/localization_health.yaml。registry.yaml 由来）。
        self.declare_parameter('localization_stale_ms', Parameter.Type.INTEGER)
        self.declare_parameter('localization_warmup_ms', Parameter.Type.INTEGER)
        self.declare_parameter('localization_expected_nodes', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('jump_window_ms', Parameter.Type.INTEGER)
        self.declare_parameter('jump_translation_m', Parameter.Type.DOUBLE)
        self.declare_parameter('jump_rotation_rad', Parameter.Type.DOUBLE)

        self._boot_ms = self._now_ms()
        self._last_transform_ms = None
        # B′ の比較対象（前 tick の値）。None の間は jump を出さない
        # （初回・TF 不連続の直後。brief-SAFE-05B「誤発火させないための条件」）。
        self._prev_sample = None
        self._prev_ok = None
        self._prev_reason = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        status_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(LocalizationHealth, '/safety/localization_health',
                                          status_qos)

        # 比較周期ごとに比べる（B′ の定義）。publish 周期と同一。
        period_s = self.get_parameter('jump_window_ms').value / 1000.0
        self.create_timer(period_s, self._on_timer)
        self.get_logger().info('localization_health 起動')

    # ------------------------------------------------------------
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _poll_transform(self):
        """最新の map→odom の（スタンプms, x, y, yaw）を返す。取れなければ None。

        非ブロッキング（route_recorder.py と同じ流儀。timeout を渡すと単一
        スレッド executor 上で固まる）。
        """
        try:
            tf = self._tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except Exception:
            return None
        stamp = tf.header.stamp
        t = tf.transform.translation
        return (stamp.sec * 1000 + stamp.nanosec // 1_000_000,
                float(t.x), float(t.y),
                _yaw_from_quat(tf.transform.rotation))

    def _params(self) -> Params:
        return Params(
            stale_ms=self.get_parameter('localization_stale_ms').value,
            warmup_ms=self.get_parameter('localization_warmup_ms').value,
            expected_nodes=tuple(self.get_parameter('localization_expected_nodes').value or ()),
            jump_window_ms=self.get_parameter('jump_window_ms').value,
            jump_translation_m=self.get_parameter('jump_translation_m').value,
            jump_rotation_rad=self.get_parameter('jump_rotation_rad').value,
        )

    # ------------------------------------------------------------
    def _on_timer(self):
        now_ms = self._now_ms()
        sample = self._poll_transform()
        if sample is not None:
            self._last_transform_ms = sample[0]

        p = self._params()
        base = evaluate(
            now_ms=now_ms,
            boot_ms=self._boot_ms,
            last_transform_ms=self._last_transform_ms,
            present_nodes=self.get_node_names(),
            p=p,
        )

        # 優先順位は node_down ＞ stale ＞ jump（core の docstring 参照）。
        # jump は base が ok のときだけ評価する（構造で保証）。
        ok, reason = base.ok, base.reason
        if sample is not None:
            curr = TransformSample(t_ms=sample[0], x=sample[1], y=sample[2],
                                   yaw=sample[3])
            in_warmup = now_ms - self._boot_ms < p.warmup_ms
            if base.ok and not in_warmup:
                jump = detect_jump(self._prev_sample, curr, p)
                if jump.is_jump:
                    ok, reason = False, REASON_JUMP
            self._prev_sample = curr
        else:
            # TF 不連続（読み直し等）の直後は jump を出さない。次 tick から
            #  anchor を取り直す。stale 側の aging は _last_transform_ms が
            # 担う（ここでは触らない）。
            self._prev_sample = None

        msg = LocalizationHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ok = ok
        msg.reason = reason
        msg.transform_age_sec = float(base.transform_age_sec)
        msg.node_present = base.node_present
        self._pub.publish(msg)

        if ok != self._prev_ok or reason != self._prev_reason:
            if ok:
                self.get_logger().info('localization ok（復帰）')
            else:
                self.get_logger().warn(
                    f'localization ng: reason={reason} '
                    f'age={base.transform_age_sec:.1f}s '
                    f'node_present={base.node_present}')
            self._prev_ok = ok
            self._prev_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
