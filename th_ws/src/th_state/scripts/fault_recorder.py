#!/usr/bin/env python3
"""fault_recorder.py — フォルト履歴の記録ノード（Spec-safety.md §3.5.5）。

`/safety/active_faults`（立っているフォルト種別の集合）と `/system/state` を購読し、
フォルト 1 件ずつの履歴を `/system/fault_history` へ publish し、JSON Lines で
ファイルへ追記する。判定は fault_history_core が持つ。

**記録だけ。**購読と publish とファイル書きのほかは何もしない
（`/system/event` を出さない・サービスを呼ばない・他ノードの状態に触れない）。
ファイルが書けなくても警告を出すだけでノードは落ちない。

ファイル（fault_log_dir/fault_log.jsonl）:
  1 件につき最大 2 行。開始時に `ongoing=true` で 1 行、解消して outcome の判定窓が
  過ぎたら確定行（`final=true`）を 1 行。同じ (fault_type, started_at) の行は後のものが
  正。継続中のまま再起動した分は、`ongoing=true` の行だけが残る。
  fault_log_max_bytes を超えたら fault_log.jsonl.1 へ退避（既存の .1 は捨てる）。
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from th_system_msgs.msg import (ActiveFaults, FaultHistory, FaultRecord,
                                SystemState)

from th_state.fault_history_core import FaultHistoryCore

LOG_NAME = 'fault_log.jsonl'


class FaultRecorder(Node):

    def __init__(self, **kwargs):
        super().__init__('fault_recorder', **kwargs)
        # R2: 数値は既定値なしで宣言し、registry 由来の生成 yaml から必ず渡す
        self.declare_parameter('fault_outcome_window_ms', Parameter.Type.INTEGER)
        self.declare_parameter('fault_log_max_bytes', Parameter.Type.INTEGER)
        # 置き場は調整値ではないので既定値を持つ（th_data は th_ws/data のバインドマウント）
        self.declare_parameter('fault_log_dir', '/root/th_data/fault_log')

        window_s = self.get_parameter('fault_outcome_window_ms').value / 1000.0
        self._max_bytes = int(self.get_parameter('fault_log_max_bytes').value)
        self._dir = str(self.get_parameter('fault_log_dir').value)
        self._core = FaultHistoryCore(window_s)

        # 購読の QoS は発行側に合わせる（起動順で取りこぼさない）
        latch_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        # 購読の depth は発行側(1)より深くする: 取り込みが遅れても短いフォルトの
        # 立ち上がり→解消の連続した 2 通を 1 通に潰さない
        sub_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(ActiveFaults, '/safety/active_faults',
                                 self._on_active, sub_qos)
        self.create_subscription(SystemState, '/system/state', self._on_state, sub_qos)
        self._pub = self.create_publisher(FaultHistory, '/system/fault_history', latch_qos)
        self.create_timer(0.1, self._on_timer)

        self._write_warned = False
        self._publish()   # 起動直後の空の履歴（後から開いた画面にも届く）
        self.get_logger().info(
            f'fault_recorder 起動: dir={self._dir} window={window_s}s max_bytes={self._max_bytes}')

    @staticmethod
    def _now() -> float:
        return time.time()

    def _on_active(self, msg) -> None:
        if self._core.on_active(self._now(), list(msg.types), list(msg.severities)):
            self._after_change()
        else:
            self._flush_writes()

    def _on_state(self, msg) -> None:
        if self._core.on_state(self._now(), msg.mode, msg.state, msg.pause_reason):
            self._after_change()

    def _on_timer(self) -> None:
        self._core.tick(self._now())
        self._flush_writes()

    def _after_change(self) -> None:
        self._publish()
        self._flush_writes()

    def _publish(self) -> None:
        out = FaultHistory()
        out.header.stamp = self.get_clock().now().to_msg()
        for r in self._core.records():
            m = FaultRecord()
            m.fault_type = r['fault_type']
            m.started_at = float(r['started_at'])
            m.duration_s = float(r['duration_s'])
            m.ongoing = bool(r['ongoing'])
            m.mode = r['mode']
            m.state = r['state']
            m.severity = r['severity']
            m.outcome = r['outcome']
            out.records.append(m)
        self._pub.publish(out)

    def _flush_writes(self) -> None:
        lines = self._core.drain_writes()
        if not lines:
            return
        try:
            os.makedirs(self._dir, exist_ok=True)
            path = os.path.join(self._dir, LOG_NAME)
            if os.path.exists(path) and os.path.getsize(path) >= self._max_bytes:
                os.replace(path, path + '.1')
            with open(path, 'a', encoding='utf-8') as f:
                for d in lines:
                    f.write(json.dumps(d, ensure_ascii=False) + '\n')
            self._write_warned = False
        except Exception as e:  # 記録は動作に影響させない
            if not self._write_warned:
                self.get_logger().warning(f'フォルト履歴をファイルへ書けない（記録のみ・継続）: {e}')
                self._write_warned = True

    def shutdown_flush(self) -> None:
        self._core.tick(self._now())
        self._flush_writes()


def main(args=None):
    rclpy.init(args=args)
    node = FaultRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_flush()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
