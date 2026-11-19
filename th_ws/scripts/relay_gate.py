"""relay_gate.py — pi_serial_relay の「先着 1 台」ゲートの純粋コア（ROS2 非依存）。

Spec-safety.md §6.4「機体を制御する PC は 1 台だけ」:
  - ESP32 への経路は先に接続した PC だけを受け付ける。その PC が接続している間は
    他の PC からの接続を断る（後から来た PC が制御を奪えない）。
  - 接続中の PC から一定時間なにも届かなくなったら手放す。PC 側は止まっていても
    送り続ける（20Hz の再送）ので、届かないのは PC か通信路が死んだときだけ。
    半開きの TCP（Wi-Fi で FIN が届かない）でも必ず空けるため、TCP の切断検知に
    頼らず「受信時刻」で判定する。

所有は**接続ごとのトークン**（呼び出し側が渡す一意なオブジェクト）で持つ。
アドレス文字列で持つと、古い接続の後片付けが新しい所有を消してしまう
（無通信で手放す → 新しい PC が取る → 古い接続の finally が release する → 3 台目が入れる）。
release / note_frame は「自分が所有者のときだけ」効く。

表示用のアドレスはトークンとは別に持つ。時刻は time.monotonic() 由来の秒を
呼び出し側が渡す。このモジュールは rclpy / websockets / serial のどれにも
依存しないので、ホストの素の pytest で縛れる。pi_serial_relay.py と同じ
ディレクトリに置き、ラズパイへも一緒にコピーする（serial_framer.py と同じ配置）。
"""
from __future__ import annotations


class FirstComeGate:
    """接続中の制御 PC を 1 台だけ受け付けるゲート。"""

    def __init__(self) -> None:
        self._token: object | None = None
        self._owner_addr: str | None = None
        self._last_frame_s: float | None = None

    @property
    def owner_addr(self) -> str | None:
        """現在の制御 PC の表示用アドレス。居なければ None。"""
        return self._owner_addr

    def is_occupied(self) -> bool:
        return self._token is not None

    def is_owner(self, token: object) -> bool:
        return self._token is not None and self._token is token

    def try_acquire(self, token: object, addr: str, now_s: float) -> bool:
        """空いていれば所有権を取る（真）。既に居れば断る（偽）。

        同じ PC の再接続でも、前の接続が手放されていない間は待たせる
        （spec §6.4: 同じ PC が通信の切れ目から戻ってくるときも
        この時間だけ待つことになる）。
        """
        if self._token is not None:
            return False
        self._token = token
        self._owner_addr = addr
        self._last_frame_s = now_s
        return True

    def note_frame(self, token: object, now_s: float) -> None:
        """所有者からフレームが届いた時刻を記録する（所有者でなければ無視）。"""
        if self.is_owner(token):
            self._last_frame_s = now_s

    def release(self, token: object) -> bool:
        """所有者のときだけ手放す（手放したら真）。古い接続の後片付けが
        新しい所有を消せないための条件。"""
        if not self.is_owner(token):
            return False
        self._token = None
        self._owner_addr = None
        self._last_frame_s = None
        return True

    def check_release_if_idle(self, now_s: float, timeout_s: float) -> bool:
        """無通信が timeout_s を超えたら手放す（手放したら真）。

        接続直後（フレーム未受信）も last_frame_s は acquire 時刻なので、
        接続して何も送らないクライアントでも timeout_s で空く。
        """
        if self._token is None:
            return False
        if self._last_frame_s is None:
            return False
        if (now_s - self._last_frame_s) > timeout_s:
            self._token = None
            self._owner_addr = None
            self._last_frame_s = None
            return True
        return False
