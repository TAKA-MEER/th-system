"""robot_pose_register_core の純ロジックテスト (REG-2。ROS2 不要)"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.robot_pose_register_core import build_pin_from_robot_pose  # noqa: E402


def make_pin(pid, kind):
    return {'id': pid, 'name': '', 'kind': kind,
            'pose': {'x': 1.0, 'y': 1.0, 'yaw': 0.0}, 'registered_at': 0}


class TestBuildPinFromRobotPose:
    def test_home_first(self):
        pin = build_pin_from_robot_pose('HOME', 1.2346, 2.345, 0.5, [])
        assert pin['id'] == 'home'
        assert pin['kind'] == 'HOME'
        assert pin['name'] == ''
        assert pin['pose']['x'] == 1.235   # 3 桁に丸め
        assert pin['pose']['y'] == 2.345
        assert pin['pose']['yaw'] == 0.5
        assert pin['registered_at'] > 0

    def test_home_replaces_existing(self):
        # HOME は既存があれば置換して 1 個だけになる
        existing = [make_pin('home', 'HOME'), make_pin('panel_1', 'PANEL')]
        pin = build_pin_from_robot_pose('HOME', 5, 6, 1.0, existing)
        assert pin['id'] == 'home'
        assert pin['kind'] == 'HOME'

    def test_panel_serial_number(self):
        # PANEL は既存 PANEL 数 + 1 の連番
        existing = [make_pin('home', 'HOME'), make_pin('panel_1', 'PANEL')]
        pin = build_pin_from_robot_pose('PANEL', 5, 6, 1.0, existing)
        assert pin['id'] == 'panel_2'
        assert pin['kind'] == 'PANEL'

    def test_panel_first(self):
        pin = build_pin_from_robot_pose('PANEL', 5, 6, 1.0, [])
        assert pin['id'] == 'panel_1'

    def test_rounds_coordinates(self):
        pin = build_pin_from_robot_pose('PANEL', 1.23456, 2.34567, 0.12345, [])
        assert pin['pose']['x'] == 1.235
        assert pin['pose']['y'] == 2.346
        assert pin['pose']['yaw'] == 0.123
