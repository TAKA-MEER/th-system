"""
ws_protocol.py — ESP32 ⇔ esp32_bridge WebSocket バイナリプロトコル
====================================================================
ROS2 非依存の pure Python。WebSocket バイナリフレーム 1 通 = メッセージ 1 個。
WebSocket 自体がメッセージ境界を保証するため長さプレフィックスは持たない。

フレーム形式: [1 byte type tag][固定長 payload]  (すべて little-endian)

| type tag           | 方向             | payload                          | 合計サイズ |
| ------------------- | ---------------- | --------------------------------- | ---------- |
| WHEEL_CMD      (0x01) | bridge → ESP32   | float32 left_mps, float32 right_mps | 9 bytes    |
| WHEEL_FEEDBACK (0x02) | ESP32 → bridge   | float32 left_mps, float32 right_mps | 9 bytes    |
| ESTOP_HW       (0x03) | ESP32 → bridge   | uint8 estop_active (0/1)          | 2 bytes    |
| IMU_DATA       (0x04) | ESP32 → bridge   | float32 qw,qx,qy,qz, wx,wy,wz, ax,ay,az, uint8 calib_status | 42 bytes |

ESP32 側 (th_ws/esp32/src/ws_link.h) のバイト配置と必ず一致させること。
"""
import struct

WHEEL_CMD = 0x01
WHEEL_FEEDBACK = 0x02
ESTOP_HW = 0x03
IMU_DATA = 0x04

_WHEEL_STRUCT = struct.Struct('<Bff')   # type tag + left + right
_ESTOP_STRUCT = struct.Struct('<BB')    # type tag + bool
_IMU_STRUCT = struct.Struct('<BffffffffffB')  # type tag + 10 floats + calib_status


class ProtocolError(ValueError):
    """フレームの type tag 不一致・長さ不正など、デコード不能な入力"""


def pack_wheel_cmd(left_mps: float, right_mps: float) -> bytes:
    return _WHEEL_STRUCT.pack(WHEEL_CMD, left_mps, right_mps)


def unpack_wheel_cmd(data: bytes) -> tuple[float, float]:
    return _unpack_wheel(data, WHEEL_CMD)


def pack_wheel_feedback(left_mps: float, right_mps: float) -> bytes:
    return _WHEEL_STRUCT.pack(WHEEL_FEEDBACK, left_mps, right_mps)


def unpack_wheel_feedback(data: bytes) -> tuple[float, float]:
    return _unpack_wheel(data, WHEEL_FEEDBACK)


def pack_estop_hw(estop_active: bool) -> bytes:
    return _ESTOP_STRUCT.pack(ESTOP_HW, 1 if estop_active else 0)


def unpack_estop_hw(data: bytes) -> bool:
    if len(data) != _ESTOP_STRUCT.size:
        raise ProtocolError(
            f"ESTOP_HW: 長さ不正 (expected {_ESTOP_STRUCT.size}, got {len(data)})")
    tag, value = _ESTOP_STRUCT.unpack(data)
    if tag != ESTOP_HW:
        raise ProtocolError(f"ESTOP_HW: type tag 不一致 (expected 0x{ESTOP_HW:02x}, got 0x{tag:02x})")
    return bool(value)


def pack_imu_data(qw: float, qx: float, qy: float, qz: float,
                   wx: float, wy: float, wz: float,
                   ax: float, ay: float, az: float,
                   calib_status: int) -> bytes:
    return _IMU_STRUCT.pack(IMU_DATA, qw, qx, qy, qz, wx, wy, wz, ax, ay, az, calib_status)


def unpack_imu_data(data: bytes) -> tuple[float, float, float, float,
                                           float, float, float,
                                           float, float, float, int]:
    if len(data) != _IMU_STRUCT.size:
        raise ProtocolError(
            f"IMU_DATA: 長さ不正 (expected {_IMU_STRUCT.size}, got {len(data)})")
    tag, qw, qx, qy, qz, wx, wy, wz, ax, ay, az, calib_status = _IMU_STRUCT.unpack(data)
    if tag != IMU_DATA:
        raise ProtocolError(f"IMU_DATA: type tag 不一致 (expected 0x{IMU_DATA:02x}, got 0x{tag:02x})")
    return qw, qx, qy, qz, wx, wy, wz, ax, ay, az, calib_status


def _unpack_wheel(data: bytes, expected_tag: int) -> tuple[float, float]:
    if len(data) != _WHEEL_STRUCT.size:
        raise ProtocolError(
            f"type 0x{expected_tag:02x}: 長さ不正 (expected {_WHEEL_STRUCT.size}, got {len(data)})")
    tag, left, right = _WHEEL_STRUCT.unpack(data)
    if tag != expected_tag:
        raise ProtocolError(
            f"type tag 不一致 (expected 0x{expected_tag:02x}, got 0x{tag:02x})")
    return left, right


def peek_type(data: bytes) -> int:
    """フレーム先頭 1 byte の type tag だけを読む(ディスパッチ用)"""
    if len(data) < 1:
        raise ProtocolError("空フレーム: type tag を読めない")
    return data[0]
