#!/usr/bin/env python3
"""dr_spaam_lifecycle_controller.py — 人物検出（DR-SPAAM）を tracker_enabled で起動/停止

brief-tracker-default-off §3.2:
- /system/state の tracker_enabled を購読する。
- 定期（既定 1s）に DR-SPAAM（/dr_spaam/dr_spaam_ros）の lifecycle 状態（GetState）を
  読み取り、変化と実状のずれを ChangeState（activate / deactivate）で突き合わせる。
  （他ノードによる変更や再起動後のずれを放置しない）
- person_tracker は lifecycle_manager 配下のまま変更しない。DR-SPAAM 自体の
  auto_configure=true（launch 側）により起動直後は INACTIVE で待機し、
  このノードだけが activate を出す。
"""
import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from th_system_msgs.msg import SystemState

from dr_spaam_lifecycle_controller_core import (
    TRANSITION_ACTIVATE,
    TRANSITION_DEACTIVATE,
    TRANSITION_LABELS,
    TRANSITION_NONE,
    desired_lifecycle_transition,
)


def _state_qos() -> QoSProfile:
    # /system/state は TRANSIENT_LOCAL（state_manager の既定。購読開始前の値も受け取る）
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_ALL,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class DrSpaamLifecycleController(Node):
    def __init__(self):
        super().__init__('dr_spaam_lifecycle_controller')
        self.declare_parameter('get_state_service', '/dr_spaam/dr_spaam_ros/get_state')
        self.declare_parameter('change_state_service', '/dr_spaam/dr_spaam_ros/change_state')
        self.declare_parameter('reconcile_period_s', 1.0)

        self._tracker_enabled = False
        self._last_action = TRANSITION_NONE

        self.create_subscription(
            SystemState, '/system/state', self._on_system_state, _state_qos())

        self._get_client = self.create_client(
            GetState, self.get_parameter('get_state_service').value)
        self._change_client = self.create_client(
            ChangeState, self.get_parameter('change_state_service').value)

        self.create_timer(self.get_parameter('reconcile_period_s').value, self._reconcile)

    # ── 購読 /system/state ──────────────────────────────
    def _on_system_state(self, msg: SystemState):
        self._tracker_enabled = bool(msg.tracker_enabled)

    # ── 定期の突き合わせ ─────────────────────────────────
    def _reconcile(self):
        if not self._get_client.service_is_ready():
            self.get_logger().debug('get_state サービス未 ready（起動直後 / 再起動中）')
            return
        future = self._get_client.call_async(GetState.Request())
        future.add_done_callback(self._on_get_state_done)

    def _on_get_state_done(self, future):
        try:
            resp = future.result()
        except Exception as exc:  # ServiceException など
            self.get_logger().warn(f'get_state 失敗: {exc}')
            return
        action = desired_lifecycle_transition(self._tracker_enabled, resp.current_state.id)
        if action == TRANSITION_NONE:
            self._last_action = TRANSITION_NONE
            return
        self._request_transition(action)

    def _request_transition(self, action: int):
        if not self._change_client.service_is_ready():
            self.get_logger().warn(f'change_state サービス未 ready（{TRANSITION_LABELS.get(action, action)}）')
            return
        req = ChangeState.Request()
        req.transition.id = action
        future = self._change_client.call_async(req)
        future.add_done_callback(lambda f, a=action: self._on_change_done(f, a))

    def _on_change_done(self, future, action: int):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f'change_state 失敗: {exc}')
            return
        label = TRANSITION_LABELS.get(action, str(action))
        if not resp.success:
            self.get_logger().warn(f'change_state が success=false（{label}）')
            return
        if action != self._last_action:
            self.get_logger().info(f'人物検出を{label}（tracker_enabled={self._tracker_enabled}）')
        self._last_action = action


def main():
    rclpy.init()
    node = DrSpaamLifecycleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()