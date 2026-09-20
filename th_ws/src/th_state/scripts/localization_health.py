#!/usr/bin/env python3
"""localization_health.py — localization_health_core を ROS2 に繋ぐ薄いノード（WP-SAFE-05）。

自己位置推定の健全性を 1 Hz で `/safety/localization_health`
（LocalizationHealth）に publish する。止まったこと自体が異常の合図になる
（LimiterStatus と同じ流儀。heartbeat 兼用）。

判定の中身は `localization_health_core.evaluate()`（ROS2 非依存の純粋関数）が持つ。
このノードは次だけを行う:

  1. `map→odom` TF のスタンプを集める（非ブロッキング。route_recorder.py と同じ流儀）
  2. 推定ノードの有無を `get_node_names()` で照合する（connectivity_checker.py と同じ流儀）
  3. 1 Hz で `evaluate()` を呼び、結果を publish する
  4. ok/reason の変化をログへ出す（あとから「いつ見失ったか」を追えるようにする）

publisher の置き場の決定（brief-SAFE-05 が委任）: th_state の新ノード。
slam_control.py ではなく新設する。理由: (1) slam_control は地図操作の仲介で
あり健全性監視の責務ではない。TF 鮮度を見るために tf2 依存を足すと責務が肥大化する
(2) th_config_manager は新設計で廃止予定（names.md §1）。そこへ新ノードを
足さない (3) connectivity_checker と同じ「1 Hz・evaluate・状態トピック」の
形にでき、th_state 内で対称になる。
"""
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from th_system_msgs.msg import LocalizationHealth

from th_state.localization_health_core import Params, evaluate

_PUBLISH_PERIOD_S = 1.0  # LimiterStatus（20 Hz）ではなく 1 Hz。途絶判定は
# localization_topic_timeout_ms（1 Hz の 5 周期 = 5 s）が吸収する。


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

        self._boot_ms = self._now_ms()
        self._last_transform_ms = None
        self._prev_ok = None
        self._prev_reason = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        status_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(LocalizationHealth, '/safety/localization_health',
                                          status_qos)

        self.create_timer(_PUBLISH_PERIOD_S, self._on_timer)
        self.get_logger().info('localization_health 起動')

    # ------------------------------------------------------------
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _poll_transform_ms(self):
        """最新の map→odom のスタンプ [ms] を返す。取れなければ None。

        非ブロッキング（route_recorder.py と同じ流儀。timeout を渡すと単一
        スレッド executor 上で固まる）。
        """
        try:
            tf = self._tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
        except Exception:
            return None
        stamp = tf.header.stamp
        return stamp.sec * 1000 + stamp.nanosec // 1_000_000

    def _params(self) -> Params:
        return Params(
            stale_ms=self.get_parameter('localization_stale_ms').value,
            warmup_ms=self.get_parameter('localization_warmup_ms').value,
            expected_nodes=tuple(self.get_parameter('localization_expected_nodes').value or ()),
        )

    # ------------------------------------------------------------
    def _on_timer(self):
        stamp_ms = self._poll_transform_ms()
        if stamp_ms is not None:
            self._last_transform_ms = stamp_ms

        report = evaluate(
            now_ms=self._now_ms(),
            boot_ms=self._boot_ms,
            last_transform_ms=self._last_transform_ms,
            present_nodes=self.get_node_names(),
            p=self._params(),
        )

        msg = LocalizationHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ok = report.ok
        msg.reason = report.reason
        msg.transform_age_sec = float(report.transform_age_sec)
        msg.node_present = report.node_present
        self._pub.publish(msg)

        if report.ok != self._prev_ok or report.reason != self._prev_reason:
            if report.ok:
                self.get_logger().info('localization ok（復帰）')
            else:
                self.get_logger().warn(
                    f'localization ng: reason={report.reason} '
                    f'age={report.transform_age_sec:.1f}s '
                    f'node_present={report.node_present}')
            self._prev_ok = report.ok
            self._prev_reason = report.reason


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
