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


async def drive_send_loop(frame: "bytes | None", send_fn, complete_fn) -> None:
    """offer() が返した最初のフレームから、保留が尽きるまで送信し続ける。

    レビュー指摘 (2026-08) で見つかった2つの実バグをここで修正する。
    ノード側 (esp32_bridge.py の旧 _send_and_continue) は次のように
    自分自身を再帰呼び出ししていた:

        await conn.send(frame)
        result = complete(state)
        if result.frame_to_send is not None:
            await self._send_and_continue(result.frame_to_send)  # 再帰

    バグ2 (無限に伸びうる再帰): Python に末尾呼び出し最適化は無い。
    TCP 送信ウィンドウが閉じ続ける間 (D-1 がまさに対処対象にしている
    輻輳状態そのもの) に、complete() のたびに新しい保留フレームが
    割り込み続けると呼び出しごとにスタックが1段ずつ伸び、最悪
    RecursionError で送信が永久停止しかねない。ここでは while ループに
    置き換え、1呼び出しあたりのスタック深さを一定にした。

    バグ1 (complete_fn が呼ばれずに終わる経路): 送信側 (send_fn) が
    想定外の例外を投げると、その例外が complete_fn の呼び出しより先に
    伝播してしまい、状態が「送信中」のまま固定される (以後の送信要求は
    永久に保留へ積まれるだけになり、WHEEL_CMD 送信が二度と行われなく
    なる — K-1 違反。ノード再起動でしか復帰しない恒久故障で、しかも
    ログに何も出ない)。ここでは send_fn がどんな例外を投げても
    **コアレッシング状態を idle (in_flight=False) へ戻してから再送出する**
    ことを保証する (下の except 節のコメント参照)。

    注意: 「complete_fn を 1 回呼べばよい」ではない。complete() は保留が
    あるとき in_flight を True に保ったままそのフレームを返すため、
    1 回呼ぶだけでは輻輳中 (= 保留が埋まっているのが常態) にちょうど
    in_flight が固定される。None が返るまで呼び切る必要がある。

    send_fn: 1フレーム送信する非同期関数 (async def send_fn(frame))。
             例外を投げてもよい (状態の後始末はここが引き受ける)。
             ただしその呼び出しで残っていた保留フレームは送られずに
             捨てられる — 送信そのものが失敗している状況で同じ呼び出しの
             中で続行するのは安全ではないため。捨てても次の 20Hz
             キープアライブが最新値で送り直すので実害は 50ms 以内に収まる。
    complete_fn: 引数無しの同期関数。呼ぶたびに1回分の送信完了を通知し、
                 次に送るべきフレーム (無ければ None) を返す。実際の
                 状態更新とロックはノード側 (コアレッシング状態は
                 スレッド間で共有されるため) の責務であり、ここでは
                 渡された関数を呼ぶだけにする。
    """
    while frame is not None:
        try:
            await send_fn(frame)
        except BaseException:
            # 送信に失敗した以上、このフレーム以降は同じ呼び出しの中では送らない。
            # ただし **状態は必ず idle (in_flight=False) へ戻してから** 抜ける。
            #
            # complete() は保留フレームがあるとき in_flight=True を保ったまま
            # そのフレームを返す仕様なので、ここで 1 回呼ぶだけだと
            # 「保留を返しただけで誰も送らず、二度と complete されない」状態に
            # なり、in_flight が True で固定される。以後の送信要求 (20Hz の
            # キープアライブを含む) は永久に pending へ落ちるだけになり、
            # WHEEL_CMD が二度と送信されない — K-1 違反の恒久故障になる。
            # **輻輳中は pending が埋まっているのが常態**なので、これは例外的な
            # 経路ではなく本番の経路である。
            #
            # そこで None が返るまで呼び切って idle に戻す。保留は高々 1 件
            # なので有界 (最大 2 周)。捨てたフレームは次の 20Hz キープアライブが
            # 最新値で送り直すため、復帰は 50ms 以内で済む。
            #
            # BaseException で受けるのは asyncio.CancelledError (Python 3.8+ では
            # Exception ではなく BaseException 直下) を取りこぼさないため。
            # それがこの防御の存在理由なので Exception では意味がない。
            while complete_fn() is not None:
                pass
            raise
        frame = complete_fn()
