"""
test_esp32_ws_protocol.py
============================
th_esp32_bridge/ws_protocol.py — ESP32 ⇔ esp32_bridge バイナリプロトコル単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_esp32_ws_protocol.py -v
"""

import struct
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
    unpack_estop_hw_flags, FIRMWARE_FLAGS_UNKNOWN,
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
    def test_round_trip_legacy_without_dt(self, left, right):
        # 旧形式 (9 byte)。ファームウェア未更新の個体から届くフレーム。
        frame = pack_wheel_feedback(left, right)
        out_left, out_right, out_dt = unpack_wheel_feedback(frame)
        assert out_left == pytest.approx(left, abs=1e-5)
        assert out_right == pytest.approx(right, abs=1e-5)
        assert out_dt is None

    @pytest.mark.parametrize("left,right,dt", [
        (0.0, 0.0, 0.1),
        (2.0, 2.0, 0.033),
        (-2.0, -2.0, 0.85),
    ])
    def test_round_trip_with_dt(self, left, right, dt):
        frame = pack_wheel_feedback(left, right, dt)
        out_left, out_right, out_dt = unpack_wheel_feedback(frame)
        assert out_left == pytest.approx(left, abs=1e-5)
        assert out_right == pytest.approx(right, abs=1e-5)
        assert out_dt == pytest.approx(dt, abs=1e-5)

    def test_frame_size_and_type_tag(self):
        # 旧形式と新形式が長さで判別できること。両方受理する必要がある
        # (ブリッジ更新とファームウェア書き込みを同時に行わなくて済むように)。
        assert len(pack_wheel_feedback(1.0, 2.0)) == 9
        assert len(pack_wheel_feedback(1.0, 2.0, 0.1)) == 13
        assert pack_wheel_feedback(1.0, 2.0)[0] == WHEEL_FEEDBACK
        assert pack_wheel_feedback(1.0, 2.0, 0.1)[0] == WHEEL_FEEDBACK

    def test_wheel_cmd_stays_strict_at_9_bytes(self):
        # WHEEL_FEEDBACK を可変長にしたことで WHEEL_CMD 側の長さ検査が
        # 緩んでいないこと (WHEEL_CMD は 9 byte 固定のまま)。
        frame = pack_wheel_feedback(1.0, 2.0, 0.1)
        with pytest.raises(ProtocolError):
            unpack_wheel_cmd(frame)


class TestEstopHwRoundTrip:
    @pytest.mark.parametrize("active", [True, False])
    def test_round_trip(self, active):
        frame = pack_estop_hw(active)
        assert unpack_estop_hw(frame) is active

    def test_frame_size_and_type_tag(self):
        frame = pack_estop_hw(True)
        assert len(frame) == 2
        assert frame[0] == ESTOP_HW


class TestEstopHwFlags:
    """WP-ESP32-01: ESTOP_HW にファーム構成フラグ(flags)を足した3 byte形式。"""

    @pytest.mark.parametrize("active,flags", [
        (True, 0x00),
        (False, 0x00),
        (True, 0x01),   # bypass_active
        (False, 0x01),
        (True, 0xFE),   # 予約ビットが立っていても bit0 だけ意味を持つ
    ])
    def test_unpack_estop_hw_3byte(self, active, flags):
        # ESP32 側 (ws_link.cpp sendEstopHw) が送る新形式そのままの並び。
        frame = struct.pack('<BBB', ESTOP_HW, 1 if active else 0, flags)
        out_active, out_flags = unpack_estop_hw_flags(frame)
        assert out_active is active
        assert out_flags == flags

    def test_unpack_estop_hw_2byte_unknown(self):
        # 旧形式(2 byte・flags なし)。ファームウェア書き込み前の個体から届く
        # フレームで、flags は「不明」として安全側(バイパスの可能性あり)に
        # 倒す (E-1)。estop_active 自体の読み取りは従来どおり正しく行う。
        for active in (True, False):
            frame = pack_estop_hw(active)
            out_active, out_flags = unpack_estop_hw_flags(frame)
            assert out_active is active
            assert out_flags == FIRMWARE_FLAGS_UNKNOWN == 0xFF

    def test_wrong_length_raises(self):
        with pytest.raises(ProtocolError):
            unpack_estop_hw_flags(bytes([ESTOP_HW]))  # 1 byte: 短すぎる
        with pytest.raises(ProtocolError):
            unpack_estop_hw_flags(struct.pack('<BBBB', ESTOP_HW, 1, 0x00, 0x00))  # 4 byte: 長すぎる

    def test_wrong_type_tag_rejected(self):
        frame = struct.pack('<BBB', WHEEL_FEEDBACK, 1, 0x00)
        with pytest.raises(ProtocolError):
            unpack_estop_hw_flags(frame)


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
