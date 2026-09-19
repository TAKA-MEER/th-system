"""
test_odom_core.py
====================
odom_core.py (esp32_bridge のオドメトリ積分・スタンプ再同期の純粋コア) の
単体テスト。ROS2 なし・純粋 Python で実行可能。

満たす仕様: docs/plan/detailed/DetailedDesign-reuse.md §2.6 の抽出範囲。
"""

import math

import pytest

from odom_core import (
    Pose2D, StampResync, integrate_pose, resolve_dt, resync_stamp,
    yaw_to_quaternion_zw,
)


class TestResolveDt:

    def test_valid_dt_used_as_is(self):
        dt, used_esp32 = resolve_dt(0.1, nominal_dt=0.1, dt_max=1.0)
        assert dt == 0.1
        assert used_esp32 is True

    def test_none_falls_back_to_nominal(self):
        """旧形式 (dt フィールドなし) は公称周期にフォールバック"""
        dt, used_esp32 = resolve_dt(None, nominal_dt=0.1, dt_max=1.0)
        assert dt == 0.1
        assert used_esp32 is False

    def test_out_of_range_falls_back(self):
        dt, used_esp32 = resolve_dt(1.5, nominal_dt=0.1, dt_max=1.0)
        assert dt == 0.1
        assert used_esp32 is False

    def test_zero_falls_back(self):
        dt, used_esp32 = resolve_dt(0.0, nominal_dt=0.1, dt_max=1.0)
        assert dt == 0.1
        assert used_esp32 is False

    def test_negative_falls_back(self):
        dt, used_esp32 = resolve_dt(-0.1, nominal_dt=0.1, dt_max=1.0)
        assert dt == 0.1
        assert used_esp32 is False

    def test_boundary_dt_max_is_accepted(self):
        dt, used_esp32 = resolve_dt(1.0, nominal_dt=0.1, dt_max=1.0)
        assert dt == 1.0
        assert used_esp32 is True

    def test_just_over_dt_max_falls_back(self):
        dt, used_esp32 = resolve_dt(1.0001, nominal_dt=0.1, dt_max=1.0)
        assert dt == 0.1
        assert used_esp32 is False


class TestResyncStamp:

    def test_first_call_uses_now(self):
        r = resync_stamp(None, now_sec=100.0, dt=0.1, resync_threshold_sec=2.0)
        assert r == StampResync(100.0, False)

    def test_normal_advance_no_resync(self):
        r = resync_stamp(100.0, now_sec=100.1, dt=0.1, resync_threshold_sec=2.0)
        assert r.stamp_sec == pytest.approx(100.1)
        assert r.resynced is False

    def test_large_skew_resyncs_to_now(self):
        """バースト配送等で実時間から乖離しすぎたら現在時刻に貼り直す"""
        r = resync_stamp(100.0, now_sec=200.0, dt=0.1, resync_threshold_sec=2.0)
        assert r.stamp_sec == 200.0
        assert r.resynced is True

    def test_boundary_skew_not_resynced(self):
        # advanced = 100.1, skew = |102.0 - 100.1| = 1.9 < 2.0 → 貼り直さない
        r = resync_stamp(100.0, now_sec=102.0, dt=0.1, resync_threshold_sec=2.0)
        assert r.stamp_sec == pytest.approx(100.1)
        assert r.resynced is False

    def test_boundary_skew_just_over_resyncs(self):
        # advanced = 100.1, skew = |102.2 - 100.1| = 2.1 > 2.0 → 貼り直す
        r = resync_stamp(100.0, now_sec=102.2, dt=0.1, resync_threshold_sec=2.0)
        assert r.stamp_sec == pytest.approx(102.2)
        assert r.resynced is True

    def test_stamp_can_run_ahead_of_now_within_threshold(self):
        """dt の累積が実時間よりわずかに先行しても貼り直さない（両方向を許容）"""
        r = resync_stamp(100.0, now_sec=99.95, dt=0.1, resync_threshold_sec=2.0)
        assert r.stamp_sec == pytest.approx(100.1)
        assert r.resynced is False


class TestIntegratePose:

    def test_straight_motion(self):
        pose = Pose2D(0.0, 0.0, 0.0)
        new_pose, v_center, omega = integrate_pose(
            pose, v_left=0.1, v_right=0.1, wheel_base=0.39, dt=1.0)
        assert new_pose.x == pytest.approx(0.1)
        assert new_pose.y == pytest.approx(0.0)
        assert new_pose.yaw == pytest.approx(0.0)
        assert v_center == pytest.approx(0.1)
        assert omega == pytest.approx(0.0)

    def test_pure_rotation_in_place(self):
        pose = Pose2D(0.0, 0.0, 0.0)
        wheel_base = 0.39
        new_pose, v_center, omega = integrate_pose(
            pose, v_left=-0.1, v_right=0.1, wheel_base=wheel_base, dt=1.0)
        expected_omega = 0.2 / wheel_base
        assert omega == pytest.approx(expected_omega)
        assert v_center == pytest.approx(0.0)
        # 純回転では中心速度がゼロなので位置は動かない
        assert new_pose.x == pytest.approx(0.0, abs=1e-9)
        assert new_pose.y == pytest.approx(0.0, abs=1e-9)
        assert new_pose.yaw == pytest.approx(expected_omega)

    def test_yaw_offset_direction(self):
        """+y 方向 (yaw=pi/2) を向いた状態で前進すると y だけ増える"""
        pose = Pose2D(0.0, 0.0, math.pi / 2)
        new_pose, _, _ = integrate_pose(
            pose, v_left=0.1, v_right=0.1, wheel_base=0.39, dt=1.0)
        assert new_pose.x == pytest.approx(0.0, abs=1e-9)
        assert new_pose.y == pytest.approx(0.1)

    def test_zero_dt_no_motion(self):
        pose = Pose2D(1.0, 2.0, 0.5)
        new_pose, v_center, omega = integrate_pose(
            pose, v_left=0.2, v_right=0.3, wheel_base=0.39, dt=0.0)
        assert new_pose == pose


class TestYawToQuaternion:

    def test_zero_yaw(self):
        z, w = yaw_to_quaternion_zw(0.0)
        assert z == pytest.approx(0.0)
        assert w == pytest.approx(1.0)

    def test_half_pi_yaw(self):
        z, w = yaw_to_quaternion_zw(math.pi / 2)
        assert z == pytest.approx(math.sin(math.pi / 4))
        assert w == pytest.approx(math.cos(math.pi / 4))

    def test_unit_norm(self):
        for yaw in (0.1, 1.0, -2.5, math.pi, -math.pi):
            z, w = yaw_to_quaternion_zw(yaw)
            assert z * z + w * w == pytest.approx(1.0)
