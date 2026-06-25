#!/usr/bin/env python3
"""
person_mover.py
================
Gazebo の "inspector" シリンダーモデルをプログラムで移動させる。
walk.dae Actor が使えない環境での代替。

/gazebo/set_model_state サービスを呼び出し、
panels.yaml に登録された配電盤前を巡回させる。

使い方:
  ros2 run th_perception person_mover.py --ros-args \
    -p pattern:=patrol    # 配電盤を巡回
    -p pattern:=approach  # ロボットに近づく(退避テスト用)
    -p pattern:=static    # 静止(静止再配置テスト用)
"""
import math
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion


class PersonMover(Node):
    def __init__(self):
        super().__init__('person_mover')

        self.declare_parameter('pattern',       'patrol')
        self.declare_parameter('model_name',    'inspector')
        self.declare_parameter('move_speed',    0.5)    # m/s 移動速度
        self.declare_parameter('pause_sec',     4.0)    # s 各地点での停止時間
        self.declare_parameter('approach_dist', 0.6)    # m 接近テスト時の最接近距離

        self._pattern    = self.get_parameter('pattern').value
        self._model      = self.get_parameter('model_name').value
        self._speed      = self.get_parameter('move_speed').value
        self._pause      = self.get_parameter('pause_sec').value
        self._app_dist   = self.get_parameter('approach_dist').value

        # Gazebo SetEntityState サービスクライアント
        self._cli = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        # サービス待機
        if not self._cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '/gazebo/set_entity_state サービスが見つかりません。'
                'Gazebo が起動しているか確認してください。')
            return

        self.get_logger().info(
            f'person_mover 起動 pattern={self._pattern} model={self._model}')

        # 移動タイマー (20 Hz で位置を更新)
        self._waypoints = self._build_waypoints()
        self._wp_idx    = 0
        self._wp_t      = 0.0
        self._start_t   = time.time()
        self._timer     = self.create_timer(0.05, self._step)

    def _build_waypoints(self):
        """パターンに応じたウェイポイントリストを構築"""
        if self._pattern == 'patrol':
            # 配電盤前を左右に巡回 (panel_room.world に合わせた座標)
            return [
                (-3.0, 2.5, 4.0),   # (x, y, 停止秒)
                ( 0.0, 2.5, 4.0),
                ( 3.0, 2.5, 4.0),
                ( 4.0, 2.5, 2.0),
                ( 3.0, 2.5, 4.0),
                ( 0.0, 2.5, 4.0),
            ]
        elif self._pattern == 'approach':
            # ロボット（原点付近）に近づいて離れるを繰り返す
            return [
                ( 2.0,  0.0, 1.0),
                ( self._app_dist, 0.0, 2.0),
                ( 2.0,  0.0, 1.0),
                ( 3.0,  0.0, 1.0),
            ]
        elif self._pattern == 'static':
            # 固定位置で静止
            return [
                (1.5, 0.0, 999.0),
            ]
        else:
            self.get_logger().warn(f'不明なパターン: {self._pattern}')
            return [(1.5, 0.0, 999.0)]

    def _set_pose(self, x: float, y: float, z: float = 0.9):
        """Gazebo モデルの位置を設定"""
        req = SetEntityState.Request()
        state = EntityState()
        state.name = self._model
        state.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        state.reference_frame = 'world'
        req.state = state
        self._cli.call_async(req)

    def _step(self):
        """ウェイポイント間を補間しながら移動"""
        if not self._waypoints:
            return

        wps = self._waypoints
        n   = len(wps)

        cur_wp  = wps[self._wp_idx % n]
        next_wp = wps[(self._wp_idx + 1) % n]

        cx, cy, pause = cur_wp
        nx, ny, _     = next_wp

        dist = math.hypot(nx - cx, ny - cy)
        travel_t = dist / max(self._speed, 0.01)
        total_t  = travel_t + pause

        if self._wp_t < pause:
            # 現在のウェイポイントで停止
            self._set_pose(cx, cy)
        elif self._wp_t < total_t:
            # 次のウェイポイントへ移動
            prog = (self._wp_t - pause) / max(travel_t, 0.01)
            prog = min(prog, 1.0)
            ix   = cx + (nx - cx) * prog
            iy   = cy + (ny - cy) * prog
            self._set_pose(ix, iy)
        else:
            # 次のウェイポイントへ進む
            self._wp_idx = (self._wp_idx + 1) % n
            self._wp_t   = 0.0
            return

        self._wp_t += 0.05  # タイマー周期 = 50ms


def main(args=None):
    rclpy.init(args=args)
    node = PersonMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
