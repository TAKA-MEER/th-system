"""
test_send_coalescer.py
==========================
send_coalescer.py (esp32_bridge の WHEEL_CMD 送信コアレッシング判定) の
単体テスト。ROS2 なし・純粋 Python で実行可能。

満たす仕様: 2026-08 実機計測の受信ギャップ切り分け作業 D-1
(送信中に次の要求が来たらキューせず保留フレームを上書きする。最新値勝ち)。

TestDriveSendLoop はレビュー (2026-08) で見つかった2つの実バグの回帰テスト:
  - バグ1: send_fn が例外を投げると complete_fn が呼ばれず in_flight が
    固定される (K-1 の永久停止に直結)。
  - バグ2: 旧実装は自分自身を再帰呼び出ししており、輻輳が続くとスタックが
    伸び続けて RecursionError になり得た。
"""
import asyncio
import sys

import pytest

from send_coalescer import (
    INITIAL_STATE, CoalescerState, complete, drive_send_loop, offer,
)


class TestOffer:

    def test_idle_sends_immediately(self):
        """in_flight でなければ即座に送信させる"""
        result = offer(INITIAL_STATE, b"frame1")
        assert result.frame_to_send == b"frame1"
        assert result.state.in_flight is True
        assert result.state.pending is None

    def test_in_flight_holds_as_pending_without_sending(self):
        """in_flight 中の要求はキューせず保留するだけ (即時送信しない)"""
        state = CoalescerState(in_flight=True)
        result = offer(state, b"frame2")
        assert result.frame_to_send is None
        assert result.state.pending == b"frame2"
        assert result.state.in_flight is True

    def test_second_pending_overwrites_first_and_counts_drop(self):
        """in_flight 中に2回要求が来たら、後着が先着を上書きし drop を1つ数える"""
        state = CoalescerState(in_flight=True, pending=b"frame2")
        result = offer(state, b"frame3")
        assert result.frame_to_send is None
        assert result.state.pending == b"frame3"  # 最新値が勝つ
        assert result.state.dropped_count == 1

    def test_latest_value_wins_even_when_zero_overrides_nonzero(self):
        """ロック時のゼロフレームが古い非ゼロ保留フレームを上書きして勝つこと
        (これが安全上求められる正しい挙動)"""
        zero_frame = b"\x00" * 9
        nonzero_frame = b"\x01" * 9
        state = CoalescerState(in_flight=True, pending=nonzero_frame)
        result = offer(state, zero_frame)
        assert result.state.pending == zero_frame

    def test_repeated_overwrites_increment_dropped_count_once_each(self):
        """in_flight 中に3回連続で要求が来たら drop は2回 (1回目の保留は
        即時送信済み扱いではなく、2・3回目それぞれが直前の保留を捨てる)"""
        state = CoalescerState(in_flight=True)
        r1 = offer(state, b"a")
        assert r1.state.dropped_count == 0  # 保留が空だったので drop なし
        r2 = offer(r1.state, b"b")
        assert r2.state.dropped_count == 1  # "a" を捨てて "b" を保留
        r3 = offer(r2.state, b"c")
        assert r3.state.dropped_count == 2  # "b" を捨てて "c" を保留
        assert r3.state.pending == b"c"


class TestComplete:

    def test_no_pending_becomes_idle(self):
        """保留が無ければ in_flight を解除する"""
        state = CoalescerState(in_flight=True, pending=None)
        result = complete(state)
        assert result.frame_to_send is None
        assert result.state.in_flight is False

    def test_pending_is_sent_next_and_stays_in_flight(self):
        """保留があればそれを続けて送信させ、in_flight は True のまま維持する"""
        state = CoalescerState(in_flight=True, pending=b"frame2")
        result = complete(state)
        assert result.frame_to_send == b"frame2"
        assert result.state.in_flight is True
        assert result.state.pending is None

    def test_dropped_count_survives_complete(self):
        """complete() は dropped_count を変更しない (累積カウンタ)"""
        state = CoalescerState(in_flight=True, pending=b"x", dropped_count=5)
        result = complete(state)
        assert result.state.dropped_count == 5


class TestEndToEndSequence:

    def test_burst_of_offers_while_in_flight_only_sends_first_and_last(self):
        """in_flight 中に複数回の offer が来ても、実際に送られるのは
        『送信開始時の1件』と『in_flight 解除直前に残っていた最新の1件』
        だけであることをシーケンス全体で確認する (束になって配送されない)"""
        state = INITIAL_STATE
        sent = []

        r = offer(state, b"f1")
        assert r.frame_to_send == b"f1"
        sent.append(r.frame_to_send)
        state = r.state

        for frame in (b"f2", b"f3", b"f4"):
            r = offer(state, frame)
            assert r.frame_to_send is None  # in_flight 中なのでキューされない
            state = r.state

        assert state.pending == b"f4"
        assert state.dropped_count == 2  # f2, f3 が捨てられた (f4 は未送信でまだ生きている)

        r = complete(state)
        assert r.frame_to_send == b"f4"
        sent.append(r.frame_to_send)
        state = r.state

        r = complete(state)
        assert r.frame_to_send is None
        assert state.in_flight is True  # 直前の complete() 内での状態
        assert r.state.in_flight is False

        assert sent == [b"f1", b"f4"]


class TestDriveSendLoop:

    def test_complete_fn_is_called_even_if_send_fn_raises(self):
        """バグ1の回帰テスト: send_fn が例外を投げても complete_fn は
        必ず呼ばれる (in_flight が固定されたまま復帰不能にならないこと)。
        """
        calls = {"complete": 0}

        def complete_fn():
            calls["complete"] += 1
            return None  # 保留なし = in_flight 解除

        async def send_fn(frame):
            raise RuntimeError("送信失敗 (asyncio.CancelledError 等を模擬)")

        with pytest.raises(RuntimeError):
            asyncio.run(drive_send_loop(b"f1", send_fn, complete_fn))

        assert calls["complete"] == 1, (
            "send_fn が例外を投げても complete_fn は必ず1回は呼ばれ、"
            "in_flight を解放できる状態にしなければならない")

    def test_complete_fn_not_called_when_no_initial_frame(self):
        """frame=None (offer() が『既に in_flight 中』を返した場合) なら
        何もせず即座に戻る。"""
        calls = {"complete": 0}

        def complete_fn():
            calls["complete"] += 1
            return None

        async def send_fn(frame):
            pass

        asyncio.run(drive_send_loop(None, send_fn, complete_fn))
        assert calls["complete"] == 0

    def test_long_pending_chain_does_not_grow_the_call_stack(self):
        """バグ2の回帰テスト: complete_fn が延々と次のフレームを返し
        続けても (輻輳が続き pending が途切れないケースを模擬)、
        再帰ではなくループで処理されるためスタックが伸びないこと。

        recursionlimit を極端に低く設定してもエラーにならないことで、
        実装が反復的であることを直接確認する (再帰実装ならここで
        RecursionError になる)。
        """
        TOTAL_FRAMES = 5000
        calls = {"send": 0, "complete": 0}

        def complete_fn():
            calls["complete"] += 1
            if calls["complete"] >= TOTAL_FRAMES:
                return None
            return b"next"

        async def send_fn(frame):
            calls["send"] += 1

        old_limit = sys.getrecursionlimit()
        sys.setrecursionlimit(80)  # 再帰していたら5000段には到底届かない
        try:
            asyncio.run(drive_send_loop(b"f0", send_fn, complete_fn))
        finally:
            sys.setrecursionlimit(old_limit)

        assert calls["send"] == TOTAL_FRAMES
        assert calls["complete"] == TOTAL_FRAMES

    def test_send_fn_receives_each_frame_in_order(self):
        """送信されるフレームの並びが complete_fn の返す順序どおりで
        あること (ロジックの健全性確認)。"""
        received = []
        frames = [b"f1", b"f2", b"f3"]
        remaining = list(frames[1:])

        def complete_fn():
            if remaining:
                return remaining.pop(0)
            return None

        async def send_fn(frame):
            received.append(frame)

        asyncio.run(drive_send_loop(frames[0], send_fn, complete_fn))
        assert received == frames
