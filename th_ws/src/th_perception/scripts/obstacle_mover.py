#!/usr/bin/env python3
"""
obstacle_mover.py
=================
Gazebo 上の "wanderer"(人間に模した円柱) をランダムに歩き回らせるノード。

追従対象 "inspector" とは別物で、回避動作の検証用に置く「動く障害物」。
collision を持つモデルなので LiDAR(/scan) に映り、
  /scan → lidar_filter → /scan_filtered → Nav2 local_costmap(obstacle layer)
の経路でコストマップにマーキングされ、controller_server が回避する。

挙動:
  - 行動範囲 (x/y の min/max) 内でランダムに目標点を選び、move_speed で向かう
  - 目標到達後は pause_sec だけ停止し、次のランダム目標を選ぶ
  - これを繰り返すことで「ランダムに動き回る人」を模擬する

使い方:
  ros2 run th_perception obstacle_mover.py --ros-args \
    -p model_name:=wanderer -p move_speed:=0.5 \
    -p x_min:=-4.0 -p x_max:=4.0 -p y_min:=-3.0 -p y_max:=3.0
"""
import math
import random

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion


def step_toward(cx: float, cy: float, tx: float, ty: float, max_dist: float):
    """
    現在地 (cx, cy) から目標 (tx, ty) へ最大 max_dist だけ進んだ新座標を返す。
    返り値: (new_x, new_y, reached)  reached=目標に到達したか
    """
    dx, dy = tx - cx, ty - cy
    dist = math.hypot(dx, dy)
    if dist <= max_dist or dist < 1e-9:
        return tx, ty, True
    return cx + dx / dist * max_dist, cy + dy / dist * max_dist, False


class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')

        # ── パラメータ ──────────────────────────────────────
        self.declare_parameter('model_name',      'wanderer')
        self.declare_parameter('move_speed',       0.5)    # m/s
        self.declare_parameter('update_rate_hz',  20.0)
        self.declare_parameter('x_min',           -4.0)
        self.declare_parameter('x_max',            4.0)
        self.declare_parameter('y_min',           -3.0)
        self.declare_parameter('y_max',            3.0)
        self.declare_parameter('reach_threshold',  0.2)    # m 目標到達判定
        self.declare_parameter('pause_sec',        1.0)    # s 到達後の停止時間
        self.declare_parameter('z',                0.85)   # m 円柱中心高さ
        self.declare_parameter('seed',            -1)      # <0 で毎回ランダム

        self._model = self.get_parameter('model_name').value
        self._speed = self.get_parameter('move_speed').value
        rate        = self.get_parameter('update_rate_hz').value
        self._xmin  = self.get_parameter('x_min').value
        self._xmax  = self.get_parameter('x_max').value
        self._ymin  = self.get_parameter('y_min').value
        self._ymax  = self.get_parameter('y_max').value
        self._reach = self.get_parameter('reach_threshold').value
        self._pause = self.get_parameter('pause_sec').value
        self._z     = self.get_parameter('z').value
        seed        = int(self.get_parameter('seed').value)
        self._rng   = random.Random(None if seed < 0 else seed)

        self._dt = 1.0 / rate

        # ── 状態（コマンドした位置を内部で保持）────────────
        self._cx = self._rng.uniform(self._xmin, self._xmax)
        self._cy = self._rng.uniform(self._ymin, self._ymax)
        self._tx, self._ty = self._pick_target()
        self._pause_left = 0.0
        self._yaw = 0.0

        # ── Gazebo SetEntityState クライアント ───────────────
        self._cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        if not self._cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '/gazebo/set_entity_state が見つかりません。'
                'Gazebo (libgazebo_ros_state) が起動しているか確認してください。')

        self._timer = self.create_timer(self._dt, self._step)
        self.get_logger().info(
            f'obstacle_mover 起動 model="{self._model}" speed={self._speed:.2f} m/s '
            f'bounds=x[{self._xmin},{self._xmax}] y[{self._ymin},{self._ymax}]')

    def _pick_target(self):
        return (self._rng.uniform(self._xmin, self._xmax),
                self._rng.uniform(self._ymin, self._ymax))

    def _step(self):
        if self._pause_left > 0.0:
            self._pause_left -= self._dt
        else:
            nx, ny, reached = step_toward(
                self._cx, self._cy, self._tx, self._ty, self._speed * self._dt)
            if nx != self._cx or ny != self._cy:
                self._yaw = math.atan2(ny - self._cy, nx - self._cx)
            self._cx, self._cy = nx, ny
            if reached:
                self._pause_left = self._pause
                self._tx, self._ty = self._pick_target()
                self.get_logger().info(
                    f'次の目標 → ({self._tx:.2f}, {self._ty:.2f})',
                    throttle_duration_sec=2.0)

        self._set_pose(self._cx, self._cy, self._yaw)

    def _set_pose(self, x: float, y: float, yaw: float):
        req = SetEntityState.Request()
        state = EntityState()
        state.name = self._model
        state.pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(self._z)),
            orientation=Quaternion(
                x=0.0, y=0.0,
                z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)))
        state.reference_frame = 'world'
        req.state = state
        self._cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
