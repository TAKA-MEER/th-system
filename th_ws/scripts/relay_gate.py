"""relay_gate.py — pi_serial_relay の「先着 1 台」ゲートの純粋コア（ROS2 非依存）。

Spec-safety.md §6.4「機体を制御する PC は 1 台だけ」:
  - ESP32 への経路は先に接続した PC だけを受け付ける。その PC が接続している間は
    他の PC からの接続を断る（後から来た PC が制御を奪えない）。
  - 接続中の PC から一定時間なにも届かなくなったら手放す。PC 側は止まっていても
    送り続ける（20Hz の再送）ので、届かないのは PC か通信路が死んだときだけ。
    半開きの TCP（Wi-Fi で FIN が届かない）でも必ず空けるため、TCP の切断検知に
    頼らず「受信時刻」で判定する。

このモジュールは rclpy / websockets / serial のどれにも依存しない
（monotonic 時刻を引数で受け取る）ので、ホストの素の pytest で縛れる。
pi_serial_relay.py と同じディレクトリに置き、ラズパイへも一緒にコピーする
(serial_framer.py と同じ配置の扱い)。
"""
from __future__ import annotations


class FirstComeGate:
    """接続中の制御 PC を 1 台だけ受け付けるゲート。

    owner は表示用のアドレス文字列（例 "192.168.5.50"）。時刻は time.monotonic() 由来の
    秒（float）を呼び出し側が渡す。
    """

    def __init__(self) -> None:
        self._owner: str | None = None
        self._last_frame_s: float | None = None

    @property
    def owner(self) -> str | None:
        """現在の制御 PC のアドレス。居なければ None。"""
        return self._owner

    def is_occupied(self) -> bool:
        return self._owner is not None

    def try_acquire(self, owner: str, now_s: float) -> bool:
        """空いていれば所有権を取る（真）。既に居れば断る（偽）。

        同じ PC の再接続でも、前の接続が手放されていない間は待たせる
        （spec §6.4: 同じ PC が通信の切れ目から戻ってくるときも
        この時間だけ待つことになる）。
        """
        if self._owner is not None:
            return False
        self._owner = owner
        self._last_frame_s = now_s
        return True

    def note_frame(self, now_s: float) -> None:
        """クライアントからフレームが届いた時刻を記録する。"""
        if self._owner is not None:
            self._last_frame_s = now_s

    def release(self) -> None:
        self._owner = None
        self._last_frame_s = None

    def check_release_if_idle(self, now_s: float, timeout_s: float) -> bool:
        """無通信が timeout_s を超えたら手放す（手放したら真）。

        接続直後（フレーム未受信）も last_frame_s は acquire 時刻なので、
        接続して何も送らないクライアントでも timeout_s で空く。
        """
        if self._owner is None:
            return False
        if self._last_frame_s is None:
            return False
        if (now_s - self._last_frame_s) > timeout_s:
            self.release()
            return True
        return False
