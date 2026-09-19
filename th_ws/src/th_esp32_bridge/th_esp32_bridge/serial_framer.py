"""
serial_framer.py — シリアル(UART)用フレームエンベロープ
==========================================================
ROS2 非依存の pure Python。asyncio / pyserial 等の外部依存も import しない。

背景 (VISION.md 2026-09-05): ESP32 を無線 WebSocket から USB-UART 直結に変え、
ラズパイの中継が Pi → PC の esp32_bridge (WebSocket サーバー、無変更) へ
クライアント接続する。シリアルは生のバイトストリームでメッセージ境界が無く、
ESP32 のブート時 ASCII バナーも混ざりうるため、ここにエンベロープ層を 1 つ追加し、
その中に既存の ws_protocol フレーム (tag 込み) をそのまま封入する。

エンベロープ形式 (計 LEN + 4 バイト):

    byte 0              : 0xAA        (sync1、固定)
    byte 1              : 0x55        (sync2、固定)
    byte 2              : LEN         (1 byte, unsigned, PAYLOAD のバイト数。上限 64)
    byte 3..3+LEN-1     : PAYLOAD     (ws_protocol フレームそのまま。tag 込み)
    byte 3+LEN          : CRC8        ([LEN byte 1個] + [PAYLOAD] に対して計算)

CRC8 計算対象は「LEN 1バイト + PAYLOAD」のみ。sync 2バイトは含めない。
アルゴリズムはビット単位の CRC-8 (polynomial 0x07, 初期値 0x00)。
ESP32 側ファームウェア (C++) もビット単位で同一のアルゴリズムを使うため、
ここで変更してはならない。
"""

SYNC1 = 0xAA
SYNC2 = 0x55
MAX_PAYLOAD_LEN = 64

__all__ = ["crc8", "encode_frame", "SerialFrameDecoder", "ProtocolError"]


class ProtocolError(ValueError):
    """エンベロープの同期・長さ・CRC が不正な入力 (デコード不能)"""


def crc8(data: bytes) -> int:
    """CRC-8 (poly 0x07, init 0x00)。シリアルエンベロープの検証用。

    ESP32 側 (C++) ファームウェアとビット単位で同一のアルゴリズムを
    使うこと。ここで「もっと良い」CRC に変えると両者が不一致になる。
    """
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def encode_frame(payload: bytes) -> bytes:
    """ws_protocol の完成フレーム (例: pack_wheel_cmd(...) の戻り値) を
    エンベロープ込みのバイト列に変換する。

    len(payload) > 64 なら ValueError を投げる。
    """
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(
            f"payload が長すぎます ({len(payload)} > {MAX_PAYLOAD_LEN})")
    len_byte = len(payload)
    crc = crc8(bytes([len_byte]) + payload)
    return bytes([SYNC1, SYNC2, len_byte]) + payload + bytes([crc])


class SerialFrameDecoder:
    """ストリーミングデコーダ。

    シリアルから届く生バイト列を少しずつ feed() に渡す運用を想定する。
    1 回の feed で完全なフレームが揃うとは限らず、逆に複数フレーム分まとめて
    届くこともある。feed() は完成した有効フレームの **PAYLOAD 部分だけ**
    (エンベロープを剥がした後の ws_protocol フレームそのもの) のリストを返す。
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        """バイト列を追加し、完成した有効フレームの PAYLOAD リストを返す。"""
        self._buf.extend(data)
        frames: list[bytes] = []

        while self._buf:
            # a. 次の sync (0xAA 0x55) の位置を探す。
            #    - 見つかればその先頭 index を返す。
            #    - 見つからなければ、末尾だけが単独 0xAA の場合のみそれを残す
            #      (次の feed で 0x55 が続く可能性) ため、None を返して保持し、
            #      それ以外は全破棄して None を返す。
            sync_idx = self._find_sync()
            if sync_idx is None:
                if len(self._buf) >= 1 and self._buf[-1] == SYNC1:
                    keep = self._buf[-1:]
                    self._buf = bytearray(keep)
                else:
                    self._buf.clear()
                break

            # sync より前のゴミを破棄。
            if sync_idx > 0:
                del self._buf[:sync_idx]

            # b. LEN を読むには最低 3 バイト (sync2 + len1) 必要。
            if len(self._buf) < 3:
                break

            len_byte = self._buf[2]

            # d. LEN が上限 64 を超えていたら偽陽性。sync 2バイトを丸ごと捨てず、
            #    sync1 の次の 1 バイトから探し直す。
            if len_byte > MAX_PAYLOAD_LEN:
                self._buf.pop(0)
                continue

            # c. フレーム全体 (len_byte + 4 バイト) が揃うまで待つ。
            total = len_byte + 4
            if len(self._buf) < total:
                break

            # e. CRC 検証。
            received_crc = self._buf[total - 1]
            computed = crc8(bytes([len_byte]) + bytes(self._buf[3:total - 1]))
            if received_crc == computed:
                payload = bytes(self._buf[3:total - 1])
                del self._buf[:total]
                frames.append(payload)
                continue
            # e. 不一致: 偽陽性として 1 バイトだけ進めて再探索。
            self._buf.pop(0)

        return frames

    def _find_sync(self) -> int | None:
        """先頭からの 0xAA 0x55 探索。

        見つかればその先頭 index を返す。見つからなければ None を返す
        (末尾が単独 0xAA ならそれを保持したいので、feed() 側で末尾を再確認する)。
        """
        n = len(self._buf)
        i = 0
        while i < n:
            if self._buf[i] == SYNC1:
                if i + 1 < n:
                    if self._buf[i + 1] == SYNC2:
                        return i
                    # 0xAA の次が 0x55 でない: この 0xAA は偽。i を進める。
                    i += 1
                else:
                    # i が末尾の単独 0xAA。保持のため None を返す。
                    return None
            else:
                i += 1
        return None

    # -- 内部状態へのアクセサ (テスト・デバッグ用) --------------------
    def buffered_bytes(self) -> bytes:
        return bytes(self._buf)
