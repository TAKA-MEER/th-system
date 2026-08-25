"""
test_send_coalescer.py
==========================
send_coalescer.py (esp32_bridge の WHEEL_CMD 送信コアレッシング判定) の
単体テスト。ROS2 なし・純粋 Python で実行可能。

満たす仕様: 2026-08 実機計測の受信ギャップ切り分け作業 D-1
(送信中に次の要求が来たらキューせず保留フレームを上書きする。最新値勝ち)。
"""

from send_coalescer import INITIAL_STATE, CoalescerState, offer, complete


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
