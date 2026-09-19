#!/usr/bin/env python3
"""connectivity_checker.py — connectivity_core を ROS2 に繋ぐ薄いノード（WP-STATE-03）。

「リンクしている」ではなく「実際にデータが行き来していること」を確かめて
`evt.link_ok` を出す（DetailedDesign-state.md §12.1・§12.2。この一行要旨は
DetailedDesign-wp1.md WP-STATE-03 §0 のとおり）。

判定の中身は `connectivity_core.evaluate()`（ROS2 非依存の純粋関数）が持つ。
このノードは DetailedDesign-wp1.md WP-STATE-03 §4.2 のとおり、次だけを行う。

  1. 4項目の入力（受信間隔・スキャン点数・起動済みノード一覧）を集める
  2. 1 Hz で `evaluate()` を呼ぶ
  3. `all_ok()` かつ **物理 E-Stop が押されていない**とき `evt.link_ok` を
     **立ち上がりで1回だけ**（L-3）`/system/event` へ出す（L-2）
  4. `restart_control_stack` を「実行できる」ようにしておく（§12.3・L-4）

`link_wait_timeout_ms` の管理（`sys.link_timeout` の生成）は `th_state`
（state_manager.py）側の責務であり、このノードには無い（§3.3 の注記）。
"""
import os
import signal

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from th_system_msgs.msg import StateEvent, WheelFeedback

from th_state.connectivity_core import Params, evaluate

_EVALUATE_PERIOD_S = 1.0  # §4.2: 1 Hz


class ConnectivityChecker(Node):

    def __init__(self, **kwargs):
        # kwargs はそのまま rclpy.node.Node へ渡す（テストが parameter_overrides=[...] を
        # 注入できるようにする。state_manager.py と同じ流儀）。
        super().__init__('connectivity_checker', **kwargs)

        # --- R2: 裸の数値を書かない。既定値なしで宣言し、外部から必ず渡す ---
        self.declare_parameter('esp32_alive_timeout_ms', Parameter.Type.INTEGER)
        self.declare_parameter('scan_expected_points', Parameter.Type.INTEGER)
        self.declare_parameter('required_nodes', Parameter.Type.STRING_ARRAY)
        # restart_control_stack の打ち切り条件（O-d4。registry.yaml では placeholder のまま。
        # §11 既知の負債の判断: 現時点では th_state 側からこの effect を配送する経路が
        # まだ無い（state_manager.py の _EFFECT_DESTINATIONS には宛先として書かれているが、
        # 実際の配送は未実装。WP-STATE-02 §11）。このパケットの §2 にも配送経路の節は
        # 挙げられていないので、自動起動の配線は対象外とする（R1）。ここでは
        # 「実行する能力」（このファイル末尾の restart_control_stack()）だけを用意する。
        self.declare_parameter('restart_max_count', Parameter.Type.INTEGER)
        self.declare_parameter('restart_wait_ms', Parameter.Type.INTEGER)
        # sim: DetailedDesign-wp1.md WP-STATE-03 §3.3 の表には無い、このノード独自の
        # 追加パラメータ（§8 Gazebo シナリオを満たすための拡張。判断は完了報告に明記）。
        # Gazebo には esp32_bridge が居ないため、ESP32 の2項目と required_nodes を除外する。
        self.declare_parameter('sim', False)

        self._last_fb_ms = None
        self._last_cmd_ms = None
        self._last_scan_ms = None
        self._scan_points = 0
        self._hw_estop = False
        # L-2 のフェイルセーフ既定: /safety/estop_hw を一度も受け取っていない間は
        # 「押されているか分からない」＝不合格として扱う（§6.2「判定できない項目は
        # 不合格」）。初期値 False のままだと、購読が繋がる前に疎通が揃った瞬間に
        # evt.link_ok を出してしまい、押しっぱなしの機体がそのまま運用に入る。
        self._estop_seen = False
        self._link_ok_gate_prev = False   # L-3: 立ち上がり検出用のラッチ
        self._restart_count = 0           # restart_control_stack の実行回数（FMEA②）

        self._setup_io()

        if self.get_parameter('sim').value:
            self.get_logger().warn(
                'sim=true: Gazebo には esp32_bridge が居ないため、ESP32 の2項目'
                '（esp32_feedback/esp32_loopback）と required_nodes の判定を除外する（§8）')

    # ------------------------------------------------------------
    def _setup_io(self):
        wf_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            WheelFeedback, '/esp32/wheel_feedback', self._on_wheel_feedback, wf_qos)
        self.create_subscription(
            WheelFeedback, '/esp32/wheel_cmd_speed', self._on_wheel_cmd_speed, wf_qos)

        scan_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, scan_qos)

        bool_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/safety/estop_hw', self._on_estop_hw, bool_qos)

        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub_event = self.create_publisher(StateEvent, '/system/event', event_qos)

        self.create_timer(_EVALUATE_PERIOD_S, self._on_timer)

    # ------------------------------------------------------------
    # 入力の収集（§3.1: 受信間隔は「受信した時刻」で見る。メッセージのヘッダ
    # スタンプではない。WiFi 遅延を受信間隔の悪化としてそのまま拾うため）。
    # ------------------------------------------------------------
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _on_wheel_feedback(self, msg):
        self._last_fb_ms = self._now_ms()

    def _on_wheel_cmd_speed(self, msg):
        self._last_cmd_ms = self._now_ms()

    def _on_scan(self, msg):
        self._last_scan_ms = self._now_ms()
        self._scan_points = len(msg.ranges)

    def _on_estop_hw(self, msg):
        self._hw_estop = bool(msg.data)
        self._estop_seen = True

    # ------------------------------------------------------------
    def _params(self) -> Params:
        return Params(
            esp32_alive_timeout_ms=self.get_parameter('esp32_alive_timeout_ms').value,
            scan_expected_points=self.get_parameter('scan_expected_points').value,
            required_nodes=tuple(self.get_parameter('required_nodes').value or ()),
            sim=bool(self.get_parameter('sim').value),
        )

    def _present_node_names(self):
        # §3.2: サービスを新設せず rclpy の get_node_names() で照合する。
        return self.get_node_names()

    # ------------------------------------------------------------
    # 責務2・3: 1 Hz で evaluate() を呼び、L-2/L-3 に従って evt.link_ok を出す
    # ------------------------------------------------------------
    def _on_timer(self):
        p = self._params()
        report = evaluate(
            now_ms=self._now_ms(),
            last_fb_ms=self._last_fb_ms,
            last_cmd_ms=self._last_cmd_ms,
            last_scan_ms=self._last_scan_ms,
            scan_points=self._scan_points,
            present_nodes=self._present_node_names(),
            p=p,
        )

        if not report.nodes and report.missing_nodes:
            self.get_logger().warn(f'必須ノードが不足している: {report.missing_nodes}')

        # L-2: 物理 E-Stop 押下中は evt.link_ok を出さない（CL-B-6・§12.6）。
        gate = report.all_ok() and self._estop_seen and not self._hw_estop

        # L-3: 立ち上がりで1回だけ。10 Hz(このタイマ自体は1Hzだが)で撃たない。
        if gate and not self._link_ok_gate_prev:
            self._publish_link_ok()
        self._link_ok_gate_prev = gate

    def _publish_link_ok(self):
        msg = StateEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.event = 'evt.link_ok'
        msg.source_node = self.get_name()
        msg.arg_json = ''
        self._pub_event.publish(msg)

    # ------------------------------------------------------------
    # restart_control_stack（DetailedDesign-wp1.md WP-STATE-03 §1・§4.2・§12.3・L-4）。
    #
    # 呼び出し元が無い（このパケットのスコープ外。ファイル冒頭のコメント参照）ため、
    # 現時点ではどこからも自動では呼ばれない。「実行する能力」を用意するだけ。
    # ------------------------------------------------------------
    def restart_control_stack(self) -> bool:
        """PC 側の ROS2 ノード群のプロセスグループへ SIGTERM を送る（L-4）。

        機体の電源には一切触れない。SIGTERM の対象は
        「connectivity_checker 自身が属するプロセスグループ」
        （= 同じ launch から起動された PC 側 ROS2 ノード群。§12.3「launch のプロセス
        グループへ SIGTERM」）。`restart_max_count` を超えたら何もしない
        （FMEA②: 無限ループの打ち切り）。戻り値は実際に SIGTERM を送ったかどうか。
        """
        max_count = self.get_parameter('restart_max_count').value
        if max_count is None or self._restart_count >= max_count:
            self.get_logger().error(
                f'restart_control_stack: restart_max_count ({max_count}) に達したため中止')
            return False
        self._restart_count += 1
        pgid = os.getpgrp()
        self.get_logger().warn(
            f'restart_control_stack: PC側プロセスグループ (pgid={pgid}) へ SIGTERM '
            f'（{self._restart_count}/{max_count} 回目）')
        os.killpg(pgid, signal.SIGTERM)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ConnectivityChecker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
