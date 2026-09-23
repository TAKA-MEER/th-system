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
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                     QoSHistoryPolicy)

from std_msgs.msg import Bool

from th_system_msgs.msg import LocalizationHealth, SystemState

from th_state.localization_health_core import (
    REASON_INACTIVE,
    REASON_JUMP,
    Params,
    TransformSample,
    check_planned_restart,
    detect_jump,
    evaluate,
    is_localization_in_use,
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
        # O-e3: 計画的な再起動の上限・再起動後の猶予（既定値なしで宣言し、
        # 外部から必ず渡す。R2）
        self.declare_parameter('localization_restart_max_ms', Parameter.Type.INTEGER)
        self.declare_parameter('localization_post_restart_grace_ms', Parameter.Type.INTEGER)

        self._boot_ms = self._now_ms()
        self._last_transform_ms = None
        # B′ の比較対象（前 tick の値）。None の間は jump を出さない
        # （初回・TF 不連続の直後。brief-SAFE-05B「誤発火させないための条件」）。
        self._prev_sample = None
        self._prev_ok = None
        self._prev_reason = None
        # O-e3: /slam_control/estimator_restarting の受信状態。時刻はすべて
        # 自分の時計（publisher が死んで true のまま止まっても上限で救うため）。
        # edge（False→True／True→False）でのみ時刻を更新する。
        self._restart_last = None
        self._restart_true_ms = None
        self._restart_false_ms = None
        # WP-SAFE-05修正: /system/state の最新値。まだ一度も受け取っていない間は
        # None（＝監視しない。起動中は INIT なので同じ）。
        self._mode = None
        self._state = None
        # WP-SAFE-05追加修正: ESTOP 中は止まる直前のモードに従う
        # （Spec-safety.md §3.5.0）ためのラッチ。state_manager が ESTOP に
        # 入るときに記録する。
        self._prev_mode = None
        self._prev_state = None
        # 監視外⇔監視中の切り替わりと、監視外の間の実際の判定の変化をログに
        # 残すための前回値。
        self._prev_in_use = None
        self._prev_actual = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        status_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(LocalizationHealth, '/safety/localization_health',
                                          status_qos)

        # O-e3: slam_control が示す計画的な再起動の通知を受ける。発行側の
        # QoS（RELIABLE + TRANSIENT_LOCAL + depth 1）に合わせる。後から起動
        # しても直近値が届く（再起動の最中にこちらが起動した場合に要る）。
        restart_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/slam_control/estimator_restarting', self._on_restarting,
            restart_qos)

        # WP-SAFE-05修正: 使っていない間は監視しない（Spec-safety.md §3.5.0）。
        # QoS は publisher（state_manager.py の state_qos）に合わせる:
        # depth 1・RELIABLE・TRANSIENT_LOCAL・KEEP_LAST。ここを間違えると
        # 受信できない＝ずっと「未受信＝監視しない」になり、全モードで監視が
        # 黙って切れる。
        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(SystemState, '/system/state',
                                 self._on_system_state, state_qos)

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

    def _on_restarting(self, msg: Bool) -> None:
        """O-e3: 再起動通知の edge を自分の時計で記録する。"""
        now_ms = self._now_ms()
        value = bool(msg.data)
        if value and self._restart_last is not True:
            self._restart_true_ms = now_ms    # False→True（初回 True 含む）
        if not value and self._restart_last is True:
            self._restart_false_ms = now_ms   # True→False
        self._restart_last = value

    def _on_system_state(self, msg: SystemState) -> None:
        """WP-SAFE-05修正: /system/state の mode/state を保持するだけ。
        判定への反映は _on_timer が行う（次の周期で効く）。"""
        self._mode = msg.mode
        self._state = msg.state
        self._prev_mode = msg.prev_mode
        self._prev_state = msg.prev_state

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

        # O-e3: 計画的な再起動の保留判定を先に見る。上限超過の故障もここで
        # 出す（起動時猶予より優先する）。保留中は B′ の前回値を捨て、
        # 再起動をまたいだ比較をしない（立て直し前後の map→odom は別物）。
        # transform_age_sec／node_present は生きた値を載せ続ける。
        # ここでは publish せず実際の判定（actual）だけを決め、配信の差し替え
        # （モードゲート）は最後にまとめて行う。再起動中でも監視外なら inactive
        # が効くようにするため。
        restart = check_planned_restart(
            now_ms, self._restart_last,
            self._restart_true_ms, self._restart_false_ms,
            self.get_parameter('localization_restart_max_ms').value,
            self.get_parameter('localization_post_restart_grace_ms').value,
        )
        if restart is not None:
            actual_ok, actual_reason = restart
            self._prev_sample = None
        else:
            # 優先順位は node_down ＞ stale ＞ jump（core の docstring 参照）。
            # jump は base が ok のときだけ評価する（構造で保証）。
            actual_ok, actual_reason = base.ok, base.reason
            if sample is not None:
                curr = TransformSample(t_ms=sample[0], x=sample[1], y=sample[2],
                                       yaw=sample[3])
                in_warmup = now_ms - self._boot_ms < p.warmup_ms
                if base.ok and not in_warmup:
                    jump = detect_jump(self._prev_sample, curr, p)
                    if jump.is_jump:
                        actual_ok, actual_reason = False, REASON_JUMP
                self._prev_sample = curr
            else:
                # TF 不連続（読み直し等）の直後は jump を出さない。次 tick から
                #  anchor を取り直す。stale 側の aging は _last_transform_ms が
                # 担う（ここでは触らない）。
                self._prev_sample = None

        # WP-SAFE-05修正: 使っていない間は ok=true・reason=inactive を配信する
        # （Spec-safety.md §3.5.0）。判定（evaluate・再起動・B′・前回値の更新）は
        # 上で毎回すべて行っており、ここでは配信だけ差し替える。監視に入った
        # 次の周期で実際の判定がそのまま出る（入った瞬間に効く）。
        # transform_age_sec／node_present は生の値のまま（_publish が base から
        # 載せる）。B′ の前回値も監視外で更新を続ける（上で更新済み）。
        in_use = is_localization_in_use(self._mode, self._state,
                                          self._prev_mode, self._prev_state)
        if self._prev_in_use is not None and in_use != self._prev_in_use:
            self.get_logger().info(
                f'localization 監視{"開始" if in_use else "終了"}: '
                f'mode={self._mode} state={self._state} '
                f'prev_mode={self._prev_mode} prev_state={self._prev_state}')
        self._prev_in_use = in_use
        actual = (actual_ok, actual_reason)
        if not in_use and actual != self._prev_actual:
            if actual_ok:
                self.get_logger().info(
                    f'localization 監視外: 実際は ok（復帰） reason={actual_reason} '
                    f'age={base.transform_age_sec:.1f}s '
                    f'node_present={base.node_present}')
            else:
                self.get_logger().warn(
                    f'localization 監視外: 実際は ng reason={actual_reason} '
                    f'age={base.transform_age_sec:.1f}s '
                    f'node_present={base.node_present}')
        self._prev_actual = actual

        if in_use:
            self._publish(actual_ok, actual_reason, base)
        else:
            self._publish(True, REASON_INACTIVE, base)

    def _publish(self, ok: bool, reason: str, base) -> None:
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
