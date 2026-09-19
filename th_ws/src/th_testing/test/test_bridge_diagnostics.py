"""
test_bridge_diagnostics.py
==============================
bridge_diagnostics.py (esp32_bridge の実行時計器) の単体テスト。
ROS2 なし・純粋 Python で実行可能。

満たす仕様: 2026-08 実機計測の受信ギャップ切り分け作業 D-3
(drain タイマー間隔・drain 処理時間・WHEEL_FEEDBACK 到着間隔の窓内最大値、
frames/drop/invalid の累積カウント、1Hz サマリの機械可読フォーマット)。
"""

from bridge_diagnostics import (
    INITIAL_DIAG_STATS, format_summary, record_drain, record_feedback_gap,
    record_invalid_frame, record_timer_tick, reset_window,
)


class TestRecordTimerTick:

    def test_tracks_max_across_calls(self):
        stats = record_timer_tick(INITIAL_DIAG_STATS, 20.0)
        stats = record_timer_tick(stats, 45.0)
        stats = record_timer_tick(stats, 30.0)
        assert stats.timer_interval_max_ms == 45.0

    def test_does_not_affect_other_fields(self):
        stats = record_timer_tick(INITIAL_DIAG_STATS, 20.0)
        assert stats.frames_total == 0
        assert stats.drain_duration_max_ms == 0.0


class TestRecordDrain:

    def test_updates_duration_max_and_frame_counts(self):
        stats = record_drain(INITIAL_DIAG_STATS, duration_ms=5.0, n_frames=3)
        assert stats.drain_duration_max_ms == 5.0
        assert stats.frames_total == 3
        assert stats.frames_since_window == 3

    def test_frame_counts_accumulate_across_calls(self):
        stats = record_drain(INITIAL_DIAG_STATS, duration_ms=5.0, n_frames=3)
        stats = record_drain(stats, duration_ms=2.0, n_frames=1)
        assert stats.frames_total == 4
        assert stats.frames_since_window == 4
        assert stats.drain_duration_max_ms == 5.0  # 2.0 は5.0を超えないので変化なし

    def test_zero_frames_still_updates_duration(self):
        """空の drain (フレームが無かった呼び出し) でも所要時間は記録する"""
        stats = record_drain(INITIAL_DIAG_STATS, duration_ms=1.5, n_frames=0)
        assert stats.drain_duration_max_ms == 1.5
        assert stats.frames_total == 0


class TestRecordFeedbackGap:

    def test_tracks_max(self):
        stats = record_feedback_gap(INITIAL_DIAG_STATS, 120.0)
        stats = record_feedback_gap(stats, 80.0)
        stats = record_feedback_gap(stats, 4106.0)
        assert stats.feedback_gap_max_ms == 4106.0


class TestRecordInvalidFrame:

    def test_accumulates(self):
        stats = record_invalid_frame(INITIAL_DIAG_STATS)
        stats = record_invalid_frame(stats)
        assert stats.invalid_frames_total == 2


class TestResetWindow:

    def test_resets_window_maxes_and_window_count_only(self):
        stats = INITIAL_DIAG_STATS
        stats = record_timer_tick(stats, 100.0)
        stats = record_drain(stats, duration_ms=50.0, n_frames=7)
        stats = record_feedback_gap(stats, 999.0)
        stats = record_invalid_frame(stats)

        reset = reset_window(stats)

        assert reset.timer_interval_max_ms == 0.0
        assert reset.drain_duration_max_ms == 0.0
        assert reset.feedback_gap_max_ms == 0.0
        assert reset.frames_since_window == 0
        # 累積カウントはリセットしない (D-3 の窓の扱いの核心)
        assert reset.frames_total == 7
        assert reset.invalid_frames_total == 1

    def test_reset_then_record_starts_fresh_window(self):
        stats = record_timer_tick(INITIAL_DIAG_STATS, 500.0)
        stats = reset_window(stats)
        stats = record_timer_tick(stats, 25.0)
        assert stats.timer_interval_max_ms == 25.0  # 前の窓の500.0を引きずらない


class TestFormatSummary:

    def test_contains_grep_prefix(self):
        line = format_summary(INITIAL_DIAG_STATS, coalescer_dropped_total=0,
                               rx_queue_dropped_total=0)
        assert line.startswith("esp32_bridge_diag:")

    def test_is_machine_readable_key_value_csv(self):
        stats = record_timer_tick(INITIAL_DIAG_STATS, 42.5)
        stats = record_drain(stats, duration_ms=3.2, n_frames=9)
        stats = record_feedback_gap(stats, 4106.0)
        stats = record_invalid_frame(stats)

        line = format_summary(stats, coalescer_dropped_total=2, rx_queue_dropped_total=5)

        fields = dict(
            part.strip().split("=", 1)
            for part in line.split("esp32_bridge_diag:", 1)[1].split(",")
        )
        assert fields["timer_interval_max_ms"] == "42.5"
        assert fields["drain_duration_max_ms"] == "3.2"
        assert fields["feedback_gap_max_ms"] == "4106.0"
        assert fields["frames_since_window"] == "9"
        assert fields["frames_total"] == "9"
        assert fields["coalescer_dropped_total"] == "2"
        assert fields["rx_queue_dropped_total"] == "5"
        assert fields["invalid_frames_total"] == "1"
