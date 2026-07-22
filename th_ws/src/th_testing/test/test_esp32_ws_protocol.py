"""
test_esp32_ws_protocol.py
============================
th_esp32_bridge/ws_protocol.py — ESP32 ⇔ esp32_bridge バイナリプロトコル単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_esp32_ws_protocol.py -v
"""

import sys
import os
import pytest

# ── パスを通す(colcon build 前にも直接 pytest できるように)
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'th_esp32_bridge'))
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_esp32_bridge', 'th_esp32_bridge'))

from ws_protocol import (
    WHEEL_CMD, WHEEL_FEEDBACK, ESTOP_HW, IMU_DATA,
    ProtocolError,
    pack_wheel_cmd, unpack_wheel_cmd,
    pack_wheel_feedback, unpack_wheel_feedback,
    pack_estop_hw, unpack_estop_hw,
    pack_imu_data, unpack_imu_data,
    peek_type,
)


# ────────────────────────────────────────────────────────────
# WHEEL_CMD / WHEEL_FEEDBACK 往復テスト
# ────────────────────────────────────────────────────────────
class TestWheelCmdRoundTrip:
    @pytest.mark.parametrize("left,right", [
        (0.0, 0.0),
        (1.5, 1.5),
        (-1.5, -1.5),
        (0.5, -0.5),        # 超信地旋回
        (-0.001, 0.001),    # 微小値
    ])
    def test_round_trip(self, left, right):
        frame = pack_wheel_cmd(left, right)
        out_left, out_right = unpack_wheel_cmd(frame)
        assert out_left == pytest.approx(left, abs=1e-5)
        assert out_right == pytest.approx(right, abs=1e-5)

    def test_frame_size_and_type_tag(self):
        frame = pack_wheel_cmd(1.0, 2.0)
        assert len(frame) == 9
        assert frame[0] == WHEEL_CMD

    def test_wrong_type_tag_rejected_by_wheel_feedback_unpack(self):
        # WHEEL_CMD のフレームを WHEEL_FEEDBACK として読もうとするとエラーになる
        frame = pack_wheel_cmd(1.0, 2.0)
        with pytest.raises(ProtocolError):
            unpack_wheel_feedback(frame)


class TestWheelFeedbackRoundTrip:
    @pytest.mark.parametrize("left,right", [
        (0.0, 0.0),
        (2.0, 2.0),
        (-2.0, -2.0),
    ])
    def test_round_trip(self, left, right):
        frame = pack_wheel_feedback(left, right)
        out_left, out_right = unpack_wheel_feedback(frame)
        assert out_left == pytest.approx(left, abs=1e-5)
        assert out_right == pytest.approx(right, abs=1e-5)

    def test_frame_size_and_type_tag(self):
        frame = pack_wheel_feedback(1.0, 2.0)
        assert len(frame) == 9
        assert frame[0] == WHEEL_FEEDBACK


class TestEstopHwRoundTrip:
    @pytest.mark.parametrize("active", [True, False])
    def test_round_trip(self, active):
        frame = pack_estop_hw(active)
        assert unpack_estop_hw(frame) is active

    def test_frame_size_and_type_tag(self):
        frame = pack_estop_hw(True)
        assert len(frame) == 2
        assert frame[0] == ESTOP_HW


class TestImuDataRoundTrip:
    @pytest.mark.parametrize("quat,gyro,accel,calib", [
        ((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0x00),
        ((0.7071, 0.0, 0.0, 0.7071), (0.1, -0.2, 0.3), (0.5, -0.5, 9.8), 0xFF),
        ((0.0, 1.0, 0.0, 0.0), (-1.5, 1.5, -1.5), (-9.8, 0.0, 0.0), 0b11100100),
    ])
    def test_round_trip(self, quat, gyro, accel, calib):
        qw, qx, qy, qz = quat
        wx, wy, wz = gyro
        ax, ay, az = accel
        frame = pack_imu_data(qw, qx, qy, qz, wx, wy, wz, ax, ay, az, calib)
        out = unpack_imu_data(frame)
        assert out[0:4] == pytest.approx(quat, abs=1e-4)
        assert out[4:7] == pytest.approx(gyro, abs=1e-4)
        assert out[7:10] == pytest.approx(accel, abs=1e-4)
        assert out[10] == calib

    def test_frame_size_and_type_tag(self):
        frame = pack_imu_data(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
        assert len(frame) == 42
        assert frame[0] == IMU_DATA

    def test_wrong_type_tag_rejected_by_imu_data_unpack(self):
        frame = pack_wheel_feedback(1.0, 2.0)
        with pytest.raises(ProtocolError):
            unpack_imu_data(frame)


# ────────────────────────────────────────────────────────────
# 不正入力
# ────────────────────────────────────────────────────────────
class TestMalformedInput:
    def test_empty_bytes_raises(self):
        with pytest.raises(ProtocolError):
            unpack_wheel_cmd(b"")
        with pytest.raises(ProtocolError):
            unpack_estop_hw(b"")
        with pytest.raises(ProtocolError):
            unpack_imu_data(b"")

    def test_wrong_length_raises(self):
        with pytest.raises(ProtocolError):
            unpack_wheel_cmd(bytes([WHEEL_CMD, 0x00, 0x00]))  # 短すぎる
        with pytest.raises(ProtocolError):
            unpack_wheel_cmd(pack_wheel_cmd(1.0, 2.0) + b"\x00")  # 長すぎる
        with pytest.raises(ProtocolError):
            unpack_imu_data(pack_imu_data(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0) + b"\x00")

    def test_unknown_type_tag_raises(self):
        garbage = bytes([0xFF]) + pack_wheel_cmd(1.0, 2.0)[1:]
        with pytest.raises(ProtocolError):
            unpack_wheel_cmd(garbage)

    def test_peek_type_on_empty_bytes_raises(self):
        with pytest.raises(ProtocolError):
            peek_type(b"")

    def test_peek_type_returns_first_byte(self):
        assert peek_type(pack_wheel_cmd(0.0, 0.0)) == WHEEL_CMD
        assert peek_type(pack_wheel_feedback(0.0, 0.0)) == WHEEL_FEEDBACK
        assert peek_type(pack_estop_hw(True)) == ESTOP_HW
        assert peek_type(pack_imu_data(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)) == IMU_DATA
