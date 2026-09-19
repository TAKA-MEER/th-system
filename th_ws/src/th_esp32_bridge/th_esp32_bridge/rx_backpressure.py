"""
rx_backpressure.py — esp32_bridge の受信キュー有界化 (ROS2 非依存)
========================================================================
D-2 (実機計測 2026-08 の受信ギャップ切り分け作業。D-1の send_coalescer.py
と同じ背景。原因A/B/Cはまだ未確定):

旧実装は受信キュー (_rx_queue) が maxsize 無しの queue.Queue で、
drain 側 (_drain_rx_queue、50Hz タイマー) も while True で無制限に
取り出していた。drain が何らかの理由 (D-4 のログ書き込みブロック等) で
一時的に止まると、キューは際限なく伸びる。再開したときに 1 回のタイマー
コールバックで溜まった分を丸ごと処理するため、その 1 回の executor
占有時間も際限なく伸びる ── これが「詰まりが自己増幅する」機構である。

対処方針:
  - キューに上限 (maxsize) を設ける。溢れたら**古い方を捨てる**
    (最新性を優先する)。WHEEL_FEEDBACK を落とすと、その分の走行時間が
    オドメトリの積分から欠落する (dt が丸ごと消える) が、無限に詰まって
    ノード全体が機能停止するより、一部のフレームを落として動作を継続する
    方を選ぶ、というトレードオフである。捨てた数は dropped_count に
    積算し、D-3 の計器で後から出せるようにする。
  - 1 回の drain 呼び出しで処理するフレーム数にも上限を設ける。上限に
    達したら残りは次周期 (20ms 後) に持ち越し、1 回のタイマー
    コールバックが有界時間で終わることを保証する。

queue.Queue は「溢れたら古い方を捨てる」操作を持たない (put_nowait は
Full 例外を投げるだけ) ため collections.deque を使う。ただし maxlen 付き
deque (deque(maxlen=N)) は使わない — append 時に上限を超えると古い方を
「黙って」捨てる仕組みのため、捨てた回数 (dropped_count) を数えられない。
D-3 の計器で drop 数をサマリに出す必要があるため、maxlen 無しの
deque + 明示的な popleft() で自前に上限判定と計数を行う (rev: レビュー
指摘で本 docstring と実装の不一致を修正)。ここでは呼び出し側 (asyncio
スレッドの _handle_client と rclpy タイマースレッドの _drain_rx_queue)
からの同時アクセスを明示的な threading.Lock で守る。

ROS2 に依存しないため、ここだけで pytest による検証が完結する。
"""
from __future__ import annotations

import threading
from collections import deque


class BoundedFrameQueue:
    """maxsize 有界・溢れたら最も古いフレームを捨てる受信キュー。"""

    def __init__(self, maxsize: int):
        if maxsize < 1:
            raise ValueError("maxsize は 1 以上にすること")
        self._maxsize = maxsize
        self._dq: "deque[bytes]" = deque()
        self._lock = threading.Lock()
        self.dropped_count = 0

    def push(self, frame: bytes) -> bool:
        """フレームを末尾に追加する。

        既に maxsize に達していれば、追加する前に先頭 (最古) を1件捨てて
        dropped_count を増やす。

        戻り値: このフレーム追加に伴い何か (今回のものではなく既存の
        最古のフレーム) を捨てたら True。
        """
        with self._lock:
            dropped = False
            if len(self._dq) >= self._maxsize:
                self._dq.popleft()
                self.dropped_count += 1
                dropped = True
            self._dq.append(frame)
            return dropped

    def drain(self, max_items: int) -> "list[bytes]":
        """先頭から最大 max_items 件を取り出して返す。

        キューにある件数が max_items 未満なら、あるだけ返す。max_items
        より多く溜まっていた場合、残りはキューに残り次回の drain() で
        取り出される (1回の呼び出しの処理量を有界にするため)。
        """
        out: "list[bytes]" = []
        with self._lock:
            for _ in range(max_items):
                if not self._dq:
                    break
                out.append(self._dq.popleft())
        return out

    def qsize(self) -> int:
        with self._lock:
            return len(self._dq)
