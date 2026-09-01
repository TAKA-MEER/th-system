"""
test_rx_backpressure.py
===========================
rx_backpressure.py (esp32_bridge の受信キュー有界化) の単体テスト。
ROS2 なし・純粋 Python で実行可能。

満たす仕様: 2026-08 実機計測の受信ギャップ切り分け作業 D-2
(キューに maxsize を設け、溢れたら古い方を捨てる。1回の drain で処理する
件数にも上限を設ける)。
"""

import pytest

from rx_backpressure import BoundedFrameQueue


class TestPush:

    def test_push_below_maxsize_does_not_drop(self):
        q = BoundedFrameQueue(maxsize=3)
        assert q.push(b"1") is False
        assert q.push(b"2") is False
        assert q.qsize() == 2
        assert q.dropped_count == 0

    def test_push_at_maxsize_drops_oldest(self):
        q = BoundedFrameQueue(maxsize=2)
        q.push(b"1")
        q.push(b"2")
        dropped = q.push(b"3")  # 満杯 -> "1" を捨てて "3" を追加
        assert dropped is True
        assert q.qsize() == 2
        assert q.dropped_count == 1
        assert q.drain(10) == [b"2", b"3"]  # 最新2件が残る (最古の "1" は消えた)

    def test_repeated_overflow_accumulates_dropped_count(self):
        q = BoundedFrameQueue(maxsize=1)
        for frame in (b"1", b"2", b"3", b"4"):
            q.push(frame)
        assert q.dropped_count == 3  # 1,2,3 が捨てられ "4" だけ残る
        assert q.drain(10) == [b"4"]

    def test_invalid_maxsize_raises(self):
        with pytest.raises(ValueError):
            BoundedFrameQueue(maxsize=0)


class TestDrain:

    def test_drain_returns_all_when_under_limit(self):
        q = BoundedFrameQueue(maxsize=10)
        q.push(b"1")
        q.push(b"2")
        assert q.drain(max_items=5) == [b"1", b"2"]
        assert q.qsize() == 0

    def test_drain_caps_at_max_items_and_leaves_rest_for_next_call(self):
        """1回の drain で処理する件数の上限。溢れた分は次回に持ち越す
        (drain 1回あたりの executor 占有時間を有界にするための仕様)"""
        q = BoundedFrameQueue(maxsize=10)
        for frame in (b"1", b"2", b"3", b"4"):
            q.push(frame)
        first = q.drain(max_items=2)
        assert first == [b"1", b"2"]
        assert q.qsize() == 2  # "3", "4" は次回に持ち越し

        second = q.drain(max_items=2)
        assert second == [b"3", b"4"]
        assert q.qsize() == 0

    def test_drain_empty_queue_returns_empty_list(self):
        q = BoundedFrameQueue(maxsize=10)
        assert q.drain(max_items=5) == []

    def test_fifo_order_preserved(self):
        q = BoundedFrameQueue(maxsize=10)
        for frame in (b"a", b"b", b"c"):
            q.push(frame)
        assert q.drain(max_items=10) == [b"a", b"b", b"c"]
