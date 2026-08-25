"""
send_coalescer.py — esp32_bridge の WHEEL_CMD 送信コアレッシング (ROS2 非依存)
========================================================================
D-1 (実機計測 2026-08 で /esp32/wheel_feedback の受信ギャップが 600ms 超で
3回、最悪 4106ms 観測された件の切り分け作業。原因は A) TCP 再送、
B) ESP32 側ループ停止、C) esp32_bridge 側の詰まり、の3つに絞られているが
未確定):

旧実装は asyncio.run_coroutine_threadsafe() で WHEEL_CMD 送信コルーチンを
20Hz キープアライブ + /cmd_vel 受信のたびに無制限に投げていた。ESP32 が
一時的に受信できなくなり TCP の送信ウィンドウが閉じると conn.send() が
待ち状態になり、その間に積まれたコルーチンが順番待ちのまま溜まっていく。
ウィンドウが開いた瞬間、溜まった送信が順に flush され、**数秒前の古い
車輪指令が束で ESP32 に配送される**。

WHEEL_CMD は「最新値だけが意味を持つ」信号 (ラッチではなく毎周期送る
速度指令) なので、これは性能問題ではなく安全問題である: ロック解除の
瞬間に古い非ゼロ指令が復活しかねない (twist_mux.yaml のロック解除時に
似た事象が K-2 として既に一度発覚している。keepalive_core.py 参照)。

対処方針: 送信が in-flight の間に次の送信要求が来ても追加でコルーチンを
キューせず、「保留フレーム」を最新値で上書きする。in-flight の送信が
完了した時点で、保留フレームがあればそれを送る。これにより同時に
存在する送信は高々「実行中 1 件 + 保留 1 件」に有界化され、古い指令が
束になって配送されることがなくなる。ロック中のゼロフレームが保留中の
古い非ゼロフレームを上書きするのは意図した挙動 (最新値が常に勝つ)。

保留フレームを上書きで捨てた回数は dropped_count として state に積算する
(D-3 の計器で 1Hz サマリに出す)。

状態遷移だけを純粋関数として持つ。実際の asyncio.run_coroutine_threadsafe()
呼び出しとロック (rclpy publisher は任意スレッドからの同時呼び出しが
保証されないため、状態そのものは呼び出し側でロックして守る必要がある)
は esp32_bridge.py (ノード側) が行う。
"""
from __future__ import annotations

from dataclasses import dataclass, replace


@dataclass(frozen=True)
class CoalescerState:
    """送信コアレッシングの状態。

    in_flight    : 現在 ESP32 への送信が進行中 (complete() 未着火) かどうか。
    pending      : in_flight 中に上書きされた最新フレーム。None は保留なし。
    dropped_count: pending を上書きで (＝一度も送信されずに) 捨てた回数の累積。
    """
    in_flight: bool = False
    pending: "bytes | None" = None
    dropped_count: int = 0


INITIAL_STATE = CoalescerState()


@dataclass(frozen=True)
class OfferResult:
    """offer() の戻り値。

    frame_to_send: 今すぐ送信すべきフレーム。None なら送信せず保留のみ
                   (既に別の送信が in-flight のため)。
    state        : 更新後の状態。
    """
    frame_to_send: "bytes | None"
    state: CoalescerState


def offer(state: CoalescerState, frame: bytes) -> OfferResult:
    """新しい送信要求を提示する。

    in_flight でなければ即座に送信を開始してよい (frame をそのまま返し、
    in_flight=True にする)。in_flight 中なら追加でキューせず pending を
    上書きする (直前の pending が既にあれば、それは一度も送られずに
    捨てられるので dropped_count を増やす)。
    """
    if not state.in_flight:
        return OfferResult(frame, replace(state, in_flight=True, pending=None))
    dropped = state.dropped_count + (1 if state.pending is not None else 0)
    return OfferResult(None, replace(state, pending=frame, dropped_count=dropped))


@dataclass(frozen=True)
class CompletionResult:
    """complete() の戻り値。

    frame_to_send: 保留フレームがあればそれ (in_flight は True のまま継続、
                   呼び出し側はこれを続けて送信すること)。無ければ None
                   (in_flight=False に戻り、次の offer() を待つ)。
    state        : 更新後の状態。
    """
    frame_to_send: "bytes | None"
    state: CoalescerState


def complete(state: CoalescerState) -> CompletionResult:
    """直前の送信が完了した通知。保留フレームがあれば引き続き送信させる。"""
    if state.pending is not None:
        return CompletionResult(state.pending, replace(state, pending=None))
    return CompletionResult(None, replace(state, in_flight=False))
