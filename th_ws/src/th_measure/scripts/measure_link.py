#!/usr/bin/env python3
"""measure_link.py — 測定機能（fk-system）との生存確認と `/measure/status`。

正本は docs/plan/detailed/DetailedDesign-measure.md §3.4・§6.1。

  - `/measure/heartbeat` を購読し、版の一致したものだけを数える
    （判定は `th_measure/link_core.py` の純コア）。
  - `/system/event` へ `evt.measure_link_ok` ／ `evt.measure_link_lost`
    （`arg_json` に `{"reason": ...}`）を**変化時と 1 Hz の出し直し**で出す。
    受け手は `th_state` の `state_manager`（最後の値をラッチし、3 秒どちらも
    来なければ途絶とみなす）。
  - `/measure/status` を reliable・transient_local・depth 1 で **10 Hz** に出す。
    材料は §6.1 の表（`/system/state`・TF `map→base_link`・
    `/map_session/status` の `instance_id`・自分の生存判定・
    `/measure/command_state`）。

`/cmd_vel*` へは一切出さない（本ノードは状態の発行だけ）。
"""
import json
import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from std_msgs.msg import String

from th_system_msgs.msg import (MapSessionStatus, MeasureHeartbeat,
                                MeasureStatus, StateEvent, SystemState)

from th_measure.link_core import (INTERFACE_VERSION, LinkTracker,
                                  StatusFields)

_STATUS_PERIOD_S = 0.1   # 10 Hz（§2 の表）
_EVENT_REEMIT_PERIOD_S = 1.0  # 1 Hz の出し直し（§6.1）
_POSE_MAX_AGE_MS = 500   # TF `map→base_link` が 0.5 s 以内に取れたら pose_valid

# `speed_limit` 名 → 数値の引き当てに使う速度パラメータ（§6.1）。
# 生成 yaml が渡す（registry の consumers）。`v_slow` / `v_jog_panel` は
# registry 上 placeholder 導出で生成物に載らないため、既定値は
# obstacle_limiter の宣言既定と同じに倒す（WAIVER(demo) はその側の注記）。
_SPEED_PARAM_DEFAULTS = {
    "v_max": 0.0,
    "v_slow": 0.30,
    "v_jog_panel": 0.10,
    "v_leash": 0.0,
    "v_check": 0.0,
    "v_calib": 0.0,
}


def _yaw_of(quaternion) -> float:
    """四元数から yaw（rad）。"""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MeasureLink(Node):

    def __init__(self, **kwargs):
        # kwargs はそのまま rclpy.node.Node へ渡す（テストから
        # parameter_overrides を注入できるように。state_manager と同じ流儀）。
        super().__init__('measure_link', **kwargs)

        # --- R2: 裸の数値を書かない。registry 由来は生成 yaml が渡す ---
        # measure_link_timeout_ms は INTEGER（registry の値 1000 は整数で載る。
        # rclpy は既定値の型で決まるため、DOUBLE 既定にすると生成 yaml の
        # 整数を受け付けず起動が落ちる。CLAUDE.md）。
        self.declare_parameter('measure_link_timeout_ms', Parameter.Type.INTEGER)
        for name, default in _SPEED_PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

        self._tracker = LinkTracker(
            timeout_ms=int(self.get_parameter('measure_link_timeout_ms').value))
        self._speed_table = {
            name: float(self.get_parameter(name).value)
            for name in _SPEED_PARAM_DEFAULTS
        }

        # 材料（§6.1）: /system/state・/map_session/status・/measure/command_state
        self._sys_mode = ""
        self._sys_state = ""
        self._sys_working = False
        self._sys_pause_reason = ""
        self._sys_speed_limit = "stop"
        self._map_instance_id = ""
        self._pending_command_id = ""
        self._pending_command = ""
        self._reentry_required = False
        self._v_cap_mps = 0.0
        self._w_cap_radps = 0.0

        self._last_evt_key = None    # (ok, reason) の変化検出用
        self._last_evt_ms = None     # 1 Hz の出し直し用

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._setup_io()
        self.create_timer(_STATUS_PERIOD_S, self._on_timer)

    # ------------------------------------------------------------
    def _setup_io(self):
        hb_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(MeasureHeartbeat, '/measure/heartbeat',
                                 self._on_heartbeat, hb_qos)

        latched_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(SystemState, '/system/state',
                                 self._on_system_state, latched_qos)
        self.create_subscription(MapSessionStatus, '/map_session/status',
                                 self._on_map_session, latched_qos)
        # /measure/command_state: state_manager が出す内部トピック（§6.1）。
        self.create_subscription(String, '/measure/command_state',
                                 self._on_command_state, latched_qos)

        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub_event = self.create_publisher(StateEvent, '/system/event', event_qos)
        self._pub_status = self.create_publisher(
            MeasureStatus, '/measure/status', latched_qos)

    # ------------------------------------------------------------
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _on_heartbeat(self, msg):
        self._tracker.on_heartbeat(str(msg.interface_version), self._now_ms())

    def _on_system_state(self, msg):
        self._sys_mode = str(msg.mode)
        self._sys_state = str(msg.state)
        self._sys_working = bool(msg.working)
        self._sys_pause_reason = str(msg.pause_reason)
        self._sys_speed_limit = str(msg.speed_limit)

    def _on_map_session(self, msg):
        self._map_instance_id = str(msg.instance_id)

    def _on_command_state(self, msg):
        try:
            obj = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(obj, dict):
            return
        self._pending_command_id = str(obj.get("pending_command_id") or "")
        self._pending_command = str(obj.get("pending_command") or "")
        self._reentry_required = bool(obj.get("reentry_required") or False)
        try:
            self._v_cap_mps = float(obj.get("v_cap_mps") or 0.0)
        except (TypeError, ValueError):
            self._v_cap_mps = 0.0
        try:
            self._w_cap_radps = float(obj.get("w_cap_radps") or 0.0)
        except (TypeError, ValueError):
            self._w_cap_radps = 0.0

    # ------------------------------------------------------------
    def _pose(self, now_ms: int):
        """TF `map→base_link`。0.5 s 以内に取れたら (True, x, y, yaw)。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException):
            return False, 0.0, 0.0, 0.0
        stamp_ms = (tf.header.stamp.sec * 1000
                    + tf.header.stamp.nanosec // 1_000_000)
        if now_ms - stamp_ms > _POSE_MAX_AGE_MS:
            return False, 0.0, 0.0, 0.0
        return (True, float(tf.transform.translation.x),
                float(tf.transform.translation.y),
                _yaw_of(tf.transform.rotation))

    def _emit_evt_if_needed(self, now_ms: int):
        """`evt.measure_link_ok` ／ `evt.measure_link_lost` を変化時と 1 Hz に出す。"""
        ok = self._tracker.link_ok(now_ms)
        reason = self._tracker.reason(now_ms)
        key = (ok, reason)
        if key == self._last_evt_key and (
                self._last_evt_ms is not None
                and now_ms - self._last_evt_ms < int(_EVENT_REEMIT_PERIOD_S * 1000)):
            return
        msg = StateEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.event = 'evt.measure_link_ok' if ok else 'evt.measure_link_lost'
        msg.source_node = 'measure_link'
        msg.arg_json = json.dumps({"reason": reason})
        self._pub_event.publish(msg)
        self._last_evt_key = key
        self._last_evt_ms = now_ms

    def _on_timer(self):
        now_ms = self._now_ms()
        self._emit_evt_if_needed(now_ms)

        pose_valid, x, y, yaw = self._pose(now_ms)
        fields = StatusFields(
            mode=self._sys_mode,
            state=self._sys_state,
            working=self._sys_working,
            link_ok=self._tracker.link_ok(now_ms),
            pose_valid=pose_valid,
            map_instance_id=self._map_instance_id,
            x=x, y=y, yaw=yaw,
            pending_command_id=self._pending_command_id,
            pending_command=self._pending_command,
            pause_reason=self._sys_pause_reason,
            blocked=(self._sys_state == "BLOCKED"),
            reentry_required=self._reentry_required,
            speed_limit_name=self._sys_speed_limit,
            speed_table=self._speed_table,
            v_cap_mps=self._v_cap_mps,
            w_cap_radps=self._w_cap_radps,
        )
        d = fields.as_dict()
        msg = MeasureStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_version = d["interface_version"]
        msg.mode = d["mode"]
        msg.state = d["state"]
        msg.working = d["working"]
        msg.link_ok = d["link_ok"]
        msg.pose_valid = d["pose_valid"]
        msg.map_instance_id = d["map_instance_id"]
        msg.x = d["x"]
        msg.y = d["y"]
        msg.yaw = d["yaw"]
        msg.pending_command_id = d["pending_command_id"]
        msg.pending_command = d["pending_command"]
        msg.pause_reason = d["pause_reason"]
        msg.blocked = d["blocked"]
        msg.reentry_required = d["reentry_required"]
        msg.v_limit_mps = d["v_limit_mps"]
        msg.w_limit_radps = d["w_limit_radps"]
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MeasureLink()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
