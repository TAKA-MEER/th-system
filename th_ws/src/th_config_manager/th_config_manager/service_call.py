"""
service_call.py
================
サービスコールバックの中から別サービスを呼び、その完了を待つための
共通ヘルパー。

**やってはいけないこと（WS-9U で確定）**: 呼び出し元コールバックの中から
`rclpy.spin_until_future_complete(node, future, executor=node._executor)` を
呼ぶこと。`main()` で既に回っている MultiThreadedExecutor を、別スレッドから
**再入 spin** することになり、`MultiThreadedExecutor._spin_once_impl` が持つ
共有ジェネレータ（`wait_for_ready_callbacks` の `self._cb_iter`）と共有リスト
`self._futures` を 2 スレッドが同時に触って壊れる
（`ValueError: generator already executing` 等）。さらに `future.result()` が
無関係なコールバックの例外まで再送出してノードごと落ちる。実機で
`replay_runner` が長距離経路の再読込中にこれで exit 1 した。

**代わりに**: `call_async` してから `future.done()` を **ポーリング**するだけ。
spin は一切しない。応答は呼び出し元ノードの `main()` の `executor.spin()`
スレッドが処理する。呼び出し元コールバックと応答コールバックが別スレッドで
並行実行できるよう、client と呼び出し元コールバックのグループは並行実行を
許すこと（ReentrantCallbackGroup、または別々の MutuallyExclusive グループ。
呼び出し元は MultiThreadedExecutor であること）。この関数は worker スレッド上で
`time.sleep` するだけなので、スレッド数 >= 2 が要る（実機は 8。念のため各ノードの
MultiThreadedExecutor は num_threads を明示している）。
"""

import time

POLL_INTERVAL_SEC = 0.02


def call_and_wait(node, client, request, timeout_sec: float):
    """対象サービスを呼び、完了 or タイムアウトまで待つ（spin せずポーリング）。

    戻り値: (result, error_message) のどちらか一方が None。
    """
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(POLL_INTERVAL_SEC)
    if not future.done():
        future.cancel()
        return None, "応答がタイムアウトしました"
    return future.result(), None
