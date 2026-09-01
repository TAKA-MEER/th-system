"""
bridge_diagnostics.py — esp32_bridge の実行時計器 (ROS2 非依存)
========================================================================
D-3 (実機計測 2026-08 の受信ギャップ切り分け作業。D-1 / D-2 と同じ背景):

実機で /esp32/wheel_feedback の受信ギャップが 600ms 超で 3回・最悪4106ms
観測された。原因候補は A) TCP 再送、B) ESP32 側ループ停止、
C) esp32_bridge (Python) 側の詰まり、の3つに絞られているが未確定
(無線区間の健全性は ICMP 3000発・損失0で別途確認済み)。D-1・D-2は
Cが発生し得るとすれば影響しうる構造上の欠陥を取り除くものだが、
それ自体はCが実際に起きていたことの証明にはならない。

このモジュールはA/B/Cを次の実機日に一発で確定させるための計器である:

  - drain タイマー (_drain_rx_queue、50Hz=公称20ms周期) の実際の呼び出し
    間隔の最大値。もしROS側で数秒のギャップが観測された瞬間にこの値が
    50ms未満のままなら、rclpyのexecutorはその間ちゃんと回っていた
    ことになりCは否定され、原因はAかBに絞られる。逆にこの値が跳ねて
    いればCが確定する。
  - drain処理自体(キューから取り出してhandle_frameする一連の処理)の
    所要時間の実測最大値。D-2のキュー溢れと合わせて「詰まりの自己増幅」
    (バックログを一度に処理して次の詰まりを招く)が起きていないかを見る。
  - WHEEL_FEEDBACK到着間隔の実測最大値。上の2値と突き合わせて、跳ねが
    ESP32側(B)かブリッジ側(C)かを切り分ける材料にする。
  - D-1 (送信コアレッシング)・D-2 (受信キュー) それぞれのdrop数。

窓の扱い: 累積カウント (frames_total, invalid_frames_total) はノード
起動からリセットしない。一方 *_max 系 (timer_interval_max_ms 等) は
サマリ出力 (1Hz) のたびに0へ戻す ── 累積最大値にすると最初の一度きりの
スパイクに値が貼りついたまま埋もれてしまい、20分ぶんのログを後から見ても
「いつ」跳ねたか分からなくなる。ウィンドウ最大値にすることで、1Hzログの
各行そのものが「その1秒間に何が起きたか」を表すタイムライン(grepで
追える形式)になる。
"""
from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class DiagStats:
    # ── 累積 (リセットしない) ──────────────────────────
    frames_total: int = 0
    invalid_frames_total: int = 0

    # ── 直近ウィンドウの最大値・件数 (サマリ出力のたびに reset_window() で
    #    0 へ戻す) ──────────────────────────────────────
    timer_interval_max_ms: float = 0.0
    drain_duration_max_ms: float = 0.0
    feedback_gap_max_ms: float = 0.0
    frames_since_window: int = 0


INITIAL_DIAG_STATS = DiagStats()


def record_timer_tick(stats: DiagStats, interval_ms: float) -> DiagStats:
    """drain タイマーが実際に呼ばれた間隔 [ms] を記録する。

    50Hz タイマーなら公称20msだが、executorが詰まっているとこれが
    伸びる。Cの確定/否定に直結する値 (モジュール docstring 参照)。
    """
    return replace(stats, timer_interval_max_ms=max(stats.timer_interval_max_ms, interval_ms))


def record_drain(stats: DiagStats, duration_ms: float, n_frames: int) -> DiagStats:
    """1回の drain 呼び出しの所要時間 [ms] と処理フレーム数を記録する。"""
    return replace(
        stats,
        drain_duration_max_ms=max(stats.drain_duration_max_ms, duration_ms),
        frames_total=stats.frames_total + n_frames,
        frames_since_window=stats.frames_since_window + n_frames)


def record_feedback_gap(stats: DiagStats, gap_ms: float) -> DiagStats:
    """WHEEL_FEEDBACK の到着間隔 [ms] を記録する。"""
    return replace(stats, feedback_gap_max_ms=max(stats.feedback_gap_max_ms, gap_ms))


def record_invalid_frame(stats: DiagStats) -> DiagStats:
    """不正フレーム/未知 type tag を受けた回数を記録する (D-4のログ絞りと対)。"""
    return replace(stats, invalid_frames_total=stats.invalid_frames_total + 1)


def reset_window(stats: DiagStats) -> DiagStats:
    """サマリ出力後に窓最大値・窓件数だけ0に戻す (累積カウントは維持する)。"""
    return replace(
        stats,
        timer_interval_max_ms=0.0,
        drain_duration_max_ms=0.0,
        feedback_gap_max_ms=0.0,
        frames_since_window=0)


def format_summary(stats: DiagStats, coalescer_dropped_total: int,
                    rx_queue_dropped_total: int) -> str:
    """1Hz サマリログの本文を作る。

    key=value のカンマ区切り (機械可読)。固定の接頭辞 `esp32_bridge_diag:`
    を付けて実機ログから grep しやすくする。coalescer_dropped_total
    (D-1) / rx_queue_dropped_total (D-2) はそれぞれの純粋コアが持つ
    累積カウンタをそのまま渡す想定 (呼び出し側=ノードで読み出す)。
    """
    return (
        "esp32_bridge_diag: "
        f"timer_interval_max_ms={stats.timer_interval_max_ms:.1f}, "
        f"drain_duration_max_ms={stats.drain_duration_max_ms:.1f}, "
        f"feedback_gap_max_ms={stats.feedback_gap_max_ms:.1f}, "
        f"frames_since_window={stats.frames_since_window}, "
        f"frames_total={stats.frames_total}, "
        f"coalescer_dropped_total={coalescer_dropped_total}, "
        f"rx_queue_dropped_total={rx_queue_dropped_total}, "
        f"invalid_frames_total={stats.invalid_frames_total}")
