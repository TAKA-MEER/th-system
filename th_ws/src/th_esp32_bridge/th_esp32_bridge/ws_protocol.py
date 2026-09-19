"""
ws_protocol.py — ESP32 ⇔ esp32_bridge WebSocket バイナリプロトコル
====================================================================
ROS2 非依存の pure Python。WebSocket バイナリフレーム 1 通 = メッセージ 1 個。
WebSocket 自体がメッセージ境界を保証するため長さプレフィックスは持たない。

フレーム形式: [1 byte type tag][固定長 payload]  (すべて little-endian)

| type tag           | 方向             | payload                          | 合計サイズ |
| ------------------- | ---------------- | --------------------------------- | ---------- |
| WHEEL_CMD      (0x01) | bridge → ESP32   | float32 left_mps, float32 right_mps | 9 bytes    |
| WHEEL_FEEDBACK (0x02) | ESP32 → bridge   | float32 left_mps, float32 right_mps, float32 dt_sec | 13 bytes |
| ESTOP_HW       (0x03) | ESP32 → bridge   | uint8 estop_active (0/1) [, uint8 flags] | 2 or 3 bytes |
| IMU_DATA       (0x04) | ESP32 → bridge   | float32 qw,qx,qy,qz, wx,wy,wz, ax,ay,az, uint8 calib_status | 42 bytes |

WHEEL_FEEDBACK の dt_sec は 2026-08-06 追加。ESP32 が速度を求めるのに実際に
使った制御周期で、ブリッジ側のオドメトリ積分区間になる。**旧形式 (9 byte、
dt_sec なし) も受理する**ため、ファームウェア書き込み前の個体でもそのまま動く
(その場合ブリッジは公称周期にフォールバックする)。

ESTOP_HW の flags (uint8) は WP-ESP32-01 で追加。ファーム構成フラグ (bit0 =
bypass_active、他ビットは将来拡張用の予約) で、`unpack_estop_hw_flags` で読む。
**旧形式 (2 byte・flags なし) も受理する**が、この場合 flags は
`FIRMWARE_FLAGS_UNKNOWN(0xFF)` を返す。flags が不明ということはバイパスされて
いるかどうかも分からないということなので、安全側 (バイパスの可能性あり) に倒す
判断をここでは行わず、呼び出し側にそのまま「不明」を伝える。

ESP32 側 (th_ws/esp32/src/serial_link.h) のバイト配置と必ず一致させること。
このフレーム自体は 2026-09-05 のシリアル化後も無変更（PAYLOAD としてそのまま
エンベロープに封入されるだけ。th_ws/src/th_esp32_bridge/th_esp32_bridge/
serial_framer.py 参照）。
"""
import struct

WHEEL_CMD = 0x01
WHEEL_FEEDBACK = 0x02
ESTOP_HW = 0x03
IMU_DATA = 0x04

_WHEEL_STRUCT = struct.Struct('<Bff')    # type tag + left + right
# WHEEL_FEEDBACK 拡張版: 上記に ESP32 側の制御周期 dt[s] を足したもの。
# ESP32 は velL = counts * distPerCount / dt で速度を出しており、その dt を
# そのまま載せる。ブリッジ側はこれを積分区間として使う (詳細は下記 unpack)。
_WHEEL_FEEDBACK_DT_STRUCT = struct.Struct('<Bfff')
_ESTOP_STRUCT = struct.Struct('<BB')    # type tag + bool
_ESTOP_FLAGS_STRUCT = struct.Struct('<BBB')  # type tag + bool + flags
_IMU_STRUCT = struct.Struct('<BffffffffffB')  # type tag + 10 floats + calib_status

# 旧形式 ESTOP_HW (2 byte) には flags が乗っていない。「不明」を意味する値として
# 予約する。0xFF は他の flags 値 (bit0 のみ有効・残りは予約) と衝突しない。
FIRMWARE_FLAGS_UNKNOWN = 0xFF


class ProtocolError(ValueError):
    """フレームの type tag 不一致・長さ不正など、デコード不能な入力"""


def pack_wheel_cmd(left_mps: float, right_mps: float) -> bytes:
    return _WHEEL_STRUCT.pack(WHEEL_CMD, left_mps, right_mps)


def unpack_wheel_cmd(data: bytes) -> tuple[float, float]:
    return _unpack_wheel(data, WHEEL_CMD)


def pack_wheel_feedback(left_mps: float, right_mps: float,
                         dt_sec: "float | None" = None) -> bytes:
    if dt_sec is None:
        return _WHEEL_STRUCT.pack(WHEEL_FEEDBACK, left_mps, right_mps)
    return _WHEEL_FEEDBACK_DT_STRUCT.pack(
        WHEEL_FEEDBACK, left_mps, right_mps, dt_sec)


def unpack_wheel_feedback(data: bytes) -> tuple[float, float, "float | None"]:
    """WHEEL_FEEDBACK をデコードし (left, right, dt) を返す。

    長さで新旧を判別する:
      9 byte  … 旧形式 (left/right のみ)。dt は None を返す。
      13 byte … 新形式。ESP32 が速度算出に使った制御周期 dt[s] を含む。

    ブリッジ側は dt が None の場合 feedback_period_ms の公称値へフォールバック
    するため、ファームウェアが未更新でも動作は従来どおり継続する。新形式を
    受けられるようにしておくことで、書き込み前後どちらの個体でも同じブリッジで
    運用できる (書き込みは AP が落ちて SSH が切れる作業のため、同時切替を
    強制しない)。
    """
    if len(data) == _WHEEL_FEEDBACK_DT_STRUCT.size:
        tag, left, right, dt = _WHEEL_FEEDBACK_DT_STRUCT.unpack(data)
        if tag != WHEEL_FEEDBACK:
            raise ProtocolError(
                f"type tag 不一致 (expected 0x{WHEEL_FEEDBACK:02x}, got 0x{tag:02x})")
        return left, right, dt
    left, right = _unpack_wheel(data, WHEEL_FEEDBACK)
    return left, right, None


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


def unpack_estop_hw_flags(data: bytes) -> tuple[bool, int]:
    """ESTOP_HW をデコードし (estop_active, flags) を返す。

    長さで新旧を判別する:
      3 byte … 新形式 (WP-ESP32-01)。ファーム構成 flags (bit0 = bypass_active)
               を含む。予約ビットが立っていてもそのまま返す (意味付けは
               呼び出し側の責務)。
      2 byte … 旧形式 (flags なし)。ファームウェア書き込み前の個体から届く。
               flags は FIRMWARE_FLAGS_UNKNOWN(0xFF) を返す。「不明」＝
               バイパスされているかもしれない、という安全側への倒し方 (E-1)。
    """
    if len(data) == _ESTOP_FLAGS_STRUCT.size:
        tag, active, flags = _ESTOP_FLAGS_STRUCT.unpack(data)
        if tag != ESTOP_HW:
            raise ProtocolError(
                f"ESTOP_HW: type tag 不一致 (expected 0x{ESTOP_HW:02x}, got 0x{tag:02x})")
        return bool(active), flags
    if len(data) == _ESTOP_STRUCT.size:
        active = unpack_estop_hw(data)
        return active, FIRMWARE_FLAGS_UNKNOWN
    raise ProtocolError(
        f"ESTOP_HW: 長さ不正 (expected {_ESTOP_STRUCT.size} or "
        f"{_ESTOP_FLAGS_STRUCT.size}, got {len(data)})")


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
