"""
test_serial_framer.py
======================
th_esp32_bridge/serial_framer.py — シリアル用フレームエンベロープの単体テスト

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_serial_framer.py -v
"""

import os
import sys

import pytest

# ── パスを通す(colcon build 前にも直接 pytest できるように)
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'th_esp32_bridge'))
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_esp32_bridge', 'th_esp32_bridge'))

from serial_framer import (              # noqa: E402
    SYNC1, SYNC2, MAX_PAYLOAD_LEN,
    crc8, encode_frame, SerialFrameDecoder,
)
from ws_protocol import (                # noqa: E402
    pack_wheel_cmd,
    pack_wheel_feedback,
    pack_estop_hw,
    pack_imu_data,
)


# ────────────────────────────────────────────────────────────
# CRC8 テストベクタ
# ────────────────────────────────────────────────────────────
class TestCrc8Vectors:
    @pytest.mark.parametrize("data,expected", [
        (b"", 0x00),
        (bytes([0x00]), 0x00),
        (bytes([0x01, 0x02, 0x03]), 0x48),
        (bytes.fromhex("000102030405060708090a0b0c0d0e0f"), 0x41),
        (bytes.fromhex("aa55aa55aa55aa55"), 0x3f),
    ])
    def test_vectors(self, data, expected):
        assert crc8(data) == expected


# ────────────────────────────────────────────────────────────
# encode_frame
# ────────────────────────────────────────────────────────────
class TestEncodeFrame:
    def test_single_byte_payload(self):
        payload = b"\x01\x02\x03"
        encoded = encode_frame(payload)
        assert encoded[0] == SYNC1
        assert encoded[1] == SYNC2
        assert encoded[2] == 3
        assert encoded[3:6] == payload
        # CRC は [LEN(1byte)] + [payload] に対してのみ
        assert encoded[6] == crc8(bytes([3]) + payload)
        assert len(encoded) == 7  # LEN + 4

    def test_vec_len_of_buffer_size(self):
        # ちょうど上限64のpayloadは成功
        payload64 = bytes(range(64))
        encoded = encode_frame(payload64)
        assert encoded[2] == 64
        assert len(encoded) == 68

    def test_over_limit_raises_value_error(self):
        with pytest.raises(ValueError):
            encode_frame(bytes(65))


# ────────────────────────────────────────────────────────────
# round-trip (ws_protocol フレームをそのまま復元)
# ────────────────────────────────────────────────────────────
class TestRoundTrip:
    @pytest.mark.parametrize("payload", [
        pack_wheel_cmd(1.0, 2.0),
        pack_wheel_cmd(-1.5, 0.5),
        pack_wheel_feedback(1.0, 2.0),              # 旧形式 (dt なし)
        pack_wheel_feedback(1.0, 2.0, 0.033),       # 新形式 (dt あり)
        pack_estop_hw(True),
        pack_estop_hw(False),
        pack_imu_data(1.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0x00),
        pack_imu_data(0.7071, 0.0, 0.0, 0.7071,
                      0.1, -0.2, 0.3,
                      0.5, -0.5, 9.8, 0xFF),
    ])
    def test_round_trip_single(self, payload):
        encoded = encode_frame(payload)
        frames = SerialFrameDecoder().feed(encoded)
        assert frames == [payload]

    def test_round_trip_two_frames_in_one_feed(self):
        p1 = pack_wheel_cmd(1.0, 2.0)
        p2 = pack_imu_data(0.0, 0.0, 0.0, 1.0,
                           0.0, 0.0, 0.0,
                           0.0, 0.0, 9.8, 0x00)
        decoded = SerialFrameDecoder().feed(encode_frame(p1) + encode_frame(p2))
        assert decoded == [p1, p2]


# ────────────────────────────────────────────────────────────
# 分割 feed (1 byte ずつ)
# ────────────────────────────────────────────────────────────
class TestSplitFeed:
    def test_byte_at_a_time(self):
        payload = pack_wheel_feedback(1.5, -1.5, 0.05)
        encoded = encode_frame(payload)
        decoder = SerialFrameDecoder()
        for i in range(len(encoded) - 1):
            result = decoder.feed(bytes([encoded[i]]))
            assert result == []
        result = decoder.feed(bytes([encoded[-1]]))
        assert result == [payload]

    def test_two_bytes_then_rest(self):
        payload = pack_wheel_cmd(1.0, 2.0)
        encoded = encode_frame(payload)
        decoder = SerialFrameDecoder()
        assert decoder.feed(encoded[:2]) == []
        assert decoder.feed(encoded[2:]) == [payload]


# ────────────────────────────────────────────────────────────
# ASCII ノイズ混入 (ブートバナー相当)
# ────────────────────────────────────────────────────────────
class TestAsciiNoise:
    BANNER = b"TH System ESP32 Firmware (Serial)\r\n"

    def test_noise_before_and_after(self):
        payload = pack_wheel_cmd(1.0, 2.0)
        encoded = encode_frame(payload)
        stream = self.BANNER + encoded + self.BANNER
        frames = SerialFrameDecoder().feed(stream)
        assert frames == [payload]

    def test_noise_between_two_frames(self):
        p1 = pack_estop_hw(True)
        p2 = pack_estop_hw(False)
        stream = encode_frame(p1) + self.BANNER + encode_frame(p2)
        frames = SerialFrameDecoder().feed(stream)
        assert frames == [p1, p2]


# ────────────────────────────────────────────────────────────
# 破損フレームはスキップし、後続の正常フレームを壊さない
# ────────────────────────────────────────────────────────────
class TestCorruptedFrame:
    def test_corrupted_payload_then_valid_frame(self):
        valid_payload = pack_wheel_cmd(1.0, 2.0)
        valid_encoded = encode_frame(valid_payload)

        # payload 中の 1 バイトを反転して CRC を破壊する。
        bad = bytearray(encode_frame(pack_wheel_feedback(0.0, 0.0, 0.0)))
        bad[4] ^= 0xFF
        bad = bytes(bad)

        decoder = SerialFrameDecoder()
        # 破損フレームだけでは何も返らない (例外も投げない)。
        assert decoder.feed(bad) == []
        # 続けて正常フレームを feed すると正しくデコードされる。
        assert decoder.feed(valid_encoded) == [valid_payload]

    def test_corrupted_frame_embedded_in_stream(self):
        valid_payload = pack_estop_hw(True)
        bad = bytearray(encode_frame(pack_estop_hw(False)))
        bad[3] ^= 0x01
        stream = bytes(bad) + encode_frame(valid_payload)
        frames = SerialFrameDecoder().feed(stream)
        assert frames == [valid_payload]


# ────────────────────────────────────────────────────────────
# 偽陽性 sync (CRC 不一致の AA 55 ...)
# ────────────────────────────────────────────────────────────
class TestFalsePositiveSync:
    def test_false_sync_before_valid_frame(self):
        valid_payload = pack_wheel_cmd(1.0, 2.0)

        # ブリーフ例示の b"\xAA\x55" + b"\x00"*10 は実はCRCが合ってしまい
        # 空フレームとして解釈される (crc8([0x00])=0x00 のため) ので、ここでは
        # 「CRC が合わない偽 sync」を意図どおり構成する: 実フレームをエンコードし、
        # payload を 1 バイト破損させて CRC を崩す。
        false_sync = bytearray(encode_frame(b"\x00" * 10))
        false_sync[3] ^= 0x7F
        stream = bytes(false_sync) + encode_frame(valid_payload)

        frames = SerialFrameDecoder().feed(stream)
        # 偽 sync でクラッシュせず、最終的に正しいフレームだけが 1 個返る。
        assert frames == [valid_payload]

    def test_overlapping_sync_pattern(self):
        # AA 55 AA 55 ... のように sync が連続・重複する場合、偽 sync を
        # 1 バイトだけ進めて再探索することで正しい sync を拾える
        # (2 バイト分丸ごと捨てると実フレームの頭を跨いでしまう)。
        valid_payload = pack_wheel_feedback(1.0, 2.0, 0.1)
        # 先頭の AA 55 は LEN 位置に AA(0xAA=170>64) を持つため偽と判定され、
        # その後ろの AA 55 が正しい sync として拾われる。
        stream = bytes([SYNC1, SYNC2, SYNC1]) + encode_frame(valid_payload)
        frames = SerialFrameDecoder().feed(stream)
        assert frames == [valid_payload]


# ────────────────────────────────────────────────────────────
# LEN 超過 (65 以上) の不正フレーム
# ────────────────────────────────────────────────────────────
class TestLenOverflow:
    def test_len_over_max_then_valid(self):
        valid_payload = pack_wheel_cmd(1.0, 2.0)
        # LEN バイト = 65 (不正)。sync は連続しており、続きに正常フレームが来る。
        bad = bytes([SYNC1, SYNC2, 65]) + b"\x00" * 65 + b"\x00"
        decoder = SerialFrameDecoder()
        # 例外は投げず、この時点では何も返さない。
        assert decoder.feed(bad) == []
        # 後続の正常フレームは影響を受けずにデコードされる。
        assert decoder.feed(encode_frame(valid_payload)) == [valid_payload]

    def test_len_over_max_embedded_before_valid(self):
        valid_payload = pack_estop_hw(False)
        bad = bytes([SYNC1, SYNC2, 65]) + b"\x00" * 3
        stream = bad + encode_frame(valid_payload)
        frames = SerialFrameDecoder().feed(stream)
        assert frames == [valid_payload]

    def test_max_len_still_decoded(self):
        # LEN = 64 (上限ちょうど) は有効。
        payload = bytes(range(64))
        decoder = SerialFrameDecoder()
        assert decoder.feed(encode_frame(payload)) == [payload]
        assert MAX_PAYLOAD_LEN == 64


# ────────────────────────────────────────────────────────────
# バッファの破棄 (ゴミの連続)
# ────────────────────────────────────────────────────────────
class TestGarbageDiscard:
    def test_pure_garbage_returns_empty(self):
        decoder = SerialFrameDecoder()
        assert decoder.feed(b"\x01\x02\x03garbage!!!") == []
        # バッファは肥大しない。
        assert decoder.buffered_bytes() == b""

    def test_trailing_single_sync_kept(self):
        # 末尾の単独 0xAA は次の feed で 0x55 が続く可能性があるため保持される。
        decoder = SerialFrameDecoder()
        assert decoder.feed(b"junk\xAA") == []
        assert decoder.buffered_bytes() == bytes([SYNC1])
        # 続きの 0x55 が来ると (0xAA 0x55) で再同期される。
        valid_payload = pack_wheel_cmd(1.0, 2.0)
        stream = bytes([SYNC2]) + encode_frame(valid_payload)[2:]
        assert decoder.feed(stream) == [valid_payload]