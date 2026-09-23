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
import json
import os
import signal

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from th_system_msgs.msg import FaultStatus, StateEvent, SystemState, WheelFeedback

from th_state.connectivity_core import Params, evaluate, should_emit_link_ok
from th_state.dev_log_core import (FaultSnap, StateSnap, TwistSum, fault_changed,
                                   format_line, should_record_cmdvel, state_changed)

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
        # dev_mode 一式: WP-DEV-01A（開発モードの土台）。ROS 側が正本であり、
        # ブラウザの localStorage は見た目専用（Spec-webui.md §5.1）。
        # launch 引数 dev_mode から渡す。実行中の切り替えは標準のパラメータ機構
        #（`ros2 param set connectivity_checker dev_mode true`）で行う。
        # config_manager（/config_manager/set_tunable_params）には乗せない。
        # dev_mode が偽なら個別値に関わらず何も無視しない（既定挙動と同一）。
        # 2026-09-23 ユーザー決定（Spec-safety.md §10）: 開発モードに入っただけでは
        # 通常運用と同じで、必要な項目だけを選んで外す。よって個別項目の既定は偽。
        # 起動時に選ぶときは dev_ignore_at_start（カンマ区切りの項目名。launch 引数
        # dev_ignore から渡す）を使う。空配列の既定は rclpy(Humble) が BYTE_ARRAY と
        # 推論して非空 override を弾くため、文字列にしてある（CLAUDE.md）。
        self.declare_parameter('dev_mode', False)
        for item in self._DEV_ITEMS:
            self.declare_parameter(f'dev_ignore_{item}', False)
        self.declare_parameter('dev_ignore_at_start', '')
        self._apply_dev_ignore_at_start()
        # WP-DEV-01C（ログの選択記録）: 何を記録するかの選択。既存の
        # `dev_ignore_*` と同じ流儀のノードローカルパラメータ
        #（`registry.yaml` には載せない。`sim` と同じ扱い）。
        # 既定は偽（opt-in）。記録は検証のたびに選び直すものであり、
        # 起動引数 `dev_mode` だけでは疎通に要らないため、無視項目のような
        # 「既定真でないと IDLE に届かない」事情が無い。実効記録は
        # dev_mode（マスタ）AND dev_log_<項目> で、OFF なら何も出さない。
        self.declare_parameter('dev_log_state', False)
        self.declare_parameter('dev_log_fault', False)
        self.declare_parameter('dev_log_cmdvel', False)

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
        self._dev_logged_state = None     # 開発モード状態の変化検出用（ログは切り替わり時だけ）
        # WP-DEV-01C: 選択記録の購読最新値（seen）と最終記録値（logged）。
        # OFF・非選択のあいだは logged を進めない。ON＋選択になった瞬間に
        # いまの値を 1 行出す（「何を記録しているか」の立証になる）ため。
        self._seen_state = None
        self._logged_state = None
        self._seen_fault = None
        self._logged_fault = None
        self._seen_cmdvel = None
        self._logged_cmdvel = None
        self._cmdvel_emit_ms = None

        self._setup_io()

        if self.get_parameter('sim').value:
            self.get_logger().warn(
                'sim=true: Gazebo には esp32_bridge が居ないため、ESP32 の2項目'
                '（esp32_feedback/esp32_loopback）と required_nodes の判定を除外する（§8）')
        self._log_dev_state(force=True)

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

        # WP-DEV-01C: 選択記録の購読。判定ロジックには一切触らない（購読専用）。
        # QoS は発行側に合わせる（state=transient_local・fault=reliable depth5・
        # cmd_vel=reliable。後から起動しても state の最新は読める）。
        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(SystemState, '/system/state', self._on_system_state, state_qos)
        fault_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(FaultStatus, '/safety/fault', self._on_fault, fault_qos)
        cmd_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, cmd_qos)

        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub_event = self.create_publisher(StateEvent, '/system/event', event_qos)

        # /system/dev_mode: 開発モードの現在状態（names.md §6.2。WP-DEV-01A）。
        # std_msgs/String に JSON を載せる（/ui/jog_lease が String、
        # /shutdown/prepare が JSON 文字列を返す前例に倣う。新規 .msg は作らない）。
        # transient_local + depth 1 で、後から起動した WebUI・ログも最新を読める。
        dev_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_dev = self.create_publisher(String, '/system/dev_mode', dev_qos)

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

    # WP-DEV-01C: 購読コールバックは最新値の保持だけ（判定・記録は _on_timer）。
    def _on_system_state(self, msg):
        self._seen_state = StateSnap(mode=str(msg.mode), state=str(msg.state))

    def _on_fault(self, msg):
        self._seen_fault = FaultSnap(
            active=bool(msg.active),
            fault_type=str(msg.fault_type),
            severity=str(msg.severity))

    def _on_cmd_vel(self, msg):
        self._seen_cmdvel = TwistSum(v=float(msg.linear.x), w=float(msg.angular.z))

    # ------------------------------------------------------------
    def _params(self) -> Params:
        return Params(
            esp32_alive_timeout_ms=self.get_parameter('esp32_alive_timeout_ms').value,
            scan_expected_points=self.get_parameter('scan_expected_points').value,
            required_nodes=tuple(self.get_parameter('required_nodes').value or ()),
            sim=bool(self.get_parameter('sim').value),
            dev_ignore_link=self._dev_effective()['link'],
        )

    def _present_node_names(self):
        # §3.2: サービスを新設せず rclpy の get_node_names() で照合する。
        return self.get_node_names()

    # ------------------------------------------------------------
    # WP-DEV-01A: 開発モードの状態。**正本はこのノードのパラメータ**であり、
    # ブラウザの localStorage は見た目専用（Spec-webui.md §5.1）。
    # 実効無視 = dev_mode（マスタ） AND dev_ignore_<項目>（項目別選択）。
    # ------------------------------------------------------------
    # link        : 疎通確認（ESP32・LiDAR・必須ノード）を合格扱いにする
    # lidar_fault : safety_monitor が LIDAR_LOST を出さない（/system/dev_mode 経由）
    # scan_stop   : obstacle_limiter が /scan 途絶でも MANUAL を止めない（同上。速度上限は通常と同じ）
    # battery / opcheck / auto_brake : 止めるゲートが as-built に無く、選択の保持・配信のみ
    # dev_mode_core.hpp の kDevItem* と web_ui の DEV_ITEMS と揃えること。
    _DEV_ITEMS = ('link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake')

    # WP-DEV-01C: 記録対象の項目名。`dev_log_<item>` に対応する。
    # dev_log_core.LOG_ITEMS と揃えること。
    _DEV_LOG_ITEMS = ('state', 'fault', 'cmdvel')

    def _apply_dev_ignore_at_start(self) -> None:
        """起動時の項目選択（dev_ignore_at_start='link,lidar_fault' 等）を反映する。

        知らない項目名は警告して捨てる（打ち間違いで黙って効かないのを防ぐ）。
        """
        raw = str(self.get_parameter('dev_ignore_at_start').value or '')
        names = [n.strip() for n in raw.split(',') if n.strip()]
        unknown = [n for n in names if n not in self._DEV_ITEMS]
        if unknown:
            self.get_logger().warn(
                f'dev_ignore_at_start に未知の項目 {unknown} がある（無視する。'
                f'有効な項目: {list(self._DEV_ITEMS)}）')
        known = [n for n in names if n in self._DEV_ITEMS]
        if known:
            self.set_parameters([Parameter(f'dev_ignore_{n}', Parameter.Type.BOOL, True)
                                 for n in known])

    def _dev_selected(self) -> dict:
        return {item: bool(self.get_parameter(f'dev_ignore_{item}').value)
                for item in self._DEV_ITEMS}

    def _dev_effective(self) -> dict:
        master = bool(self.get_parameter('dev_mode').value)
        return {item: (master and sel) for item, sel in self._dev_selected().items()}

    def _dev_log_selected(self) -> dict:
        return {item: bool(self.get_parameter(f'dev_log_{item}').value)
                for item in self._DEV_LOG_ITEMS}

    def _dev_log_effective(self) -> dict:
        master = bool(self.get_parameter('dev_mode').value)
        return {item: (master and sel) for item, sel in self._dev_log_selected().items()}

    def _maybe_log_selections(self) -> None:
        """WP-DEV-01C: 開発モード ON ＋ 選択された対象だけを ROS ログへ出す。

        記録先は ROS のログ（`get_logger().info`）であり、ファイルには書かない
        （理由は dev_log_core.py のモジュール docstring）。
        書式は 1 行 1 イベントの JSON（`dev_log_core.format_line`）。
        先頭の `dev_log: ` は grep 用の目印。

        OFF・非選択のあいだは logged 側を進めない（ON＋選択になった瞬間に
        いまの値を 1 行出す。沈黙のまま選択だけ切り替わると「記録しているか」
        が立証できないため）。
        """
        eff = self._dev_log_effective()
        if not any(eff.values()):
            return
        now_ms = self._now_ms()
        if eff['state'] and self._seen_state is not None:
            if state_changed(self._logged_state, self._seen_state):
                self.get_logger().info(
                    'dev_log: ' + format_line(
                        now_ms, 'state',
                        {'mode': self._seen_state.mode,
                         'state': self._seen_state.state}))
                self._logged_state = self._seen_state
        if eff['fault'] and self._seen_fault is not None:
            if fault_changed(self._logged_fault, self._seen_fault):
                self.get_logger().info(
                    'dev_log: ' + format_line(
                        now_ms, 'fault',
                        {'active': self._seen_fault.active,
                         'fault_type': self._seen_fault.fault_type,
                         'severity': self._seen_fault.severity}))
                self._logged_fault = self._seen_fault
        if eff['cmdvel'] and self._seen_cmdvel is not None:
            if should_record_cmdvel(
                    now_ms, self._cmdvel_emit_ms,
                    self._logged_cmdvel, self._seen_cmdvel):
                self.get_logger().info(
                    'dev_log: ' + format_line(
                        now_ms, 'cmdvel',
                        {'v': self._seen_cmdvel.v, 'w': self._seen_cmdvel.w}))
                self._logged_cmdvel = self._seen_cmdvel
                self._cmdvel_emit_ms = now_ms

    def _dev_state_dict(self) -> dict:
        return {
            'dev_mode': bool(self.get_parameter('dev_mode').value),
            'ignore': self._dev_selected(),
            'effective': self._dev_effective(),
            'estop_hw_known': self._estop_seen,
            'estop_hw_pressed': self._hw_estop,
        }

    def _log_dev_state(self, force: bool = False) -> None:
        """開発モードの ON/OFF と無視項目を切り替わるたびにログへ出す（WP-DEV-01A §5）。

        あとから「その検証は開発モードで通したのか」を追えるようにするため。
        起動時は force=True で必ず出す。
        """
        state = self._dev_state_dict()
        if not force and state == self._dev_logged_state:
            return
        self._dev_logged_state = state
        if not state['dev_mode']:
            self.get_logger().info('dev_mode=false（通常運用。警告の無視なし）')
            return
        ignored = sorted(k for k, v in state['effective'].items() if v)
        self.get_logger().warn(
            f"dev_mode=true: 無視項目={ignored}（選んだ項目だけを外す。未選択の項目は"
            '通常運用と同じ。battery/opcheck/auto_brake は as-built に止めるゲートが無く、'
            '選択状態の保持・配信のみ）')

    def _publish_dev_state(self) -> None:
        msg = String()
        msg.data = json.dumps(self._dev_state_dict(), sort_keys=True)
        self._pub_dev.publish(msg)

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

        effective_link = self._dev_effective()['link']
        # L-2 + DEV-01A: 物理 E-Stop 押下中は dev_mode でも evt.link_ok を出さない
        #（完了条件4）。判定式は connectivity_core.should_emit_link_ok（純粋関数）。
        gate = should_emit_link_ok(report, self._estop_seen, self._hw_estop,
                                   effective_link)

        # L-3: 立ち上がりで1回だけ。10 Hz(このタイマ自体は1Hzだが)で撃たない。
        if gate and not self._link_ok_gate_prev:
            if effective_link and not self._estop_seen:
                self.get_logger().warn(
                    'dev_mode: /safety/estop_hw 未受信（ESP32 不在）のまま evt.link_ok を出す。'
                    '押下が判明した時点で link_ok は止まる')
            self._publish_link_ok()
        self._link_ok_gate_prev = gate

        self._publish_dev_state()
        self._log_dev_state()
        self._maybe_log_selections()

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
