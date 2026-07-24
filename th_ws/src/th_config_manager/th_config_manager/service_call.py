"""
service_call.py
================
サービスコールバックの中から別サービスを呼び、その完了を待つための
共通ヘルパー。

rclpy.spin_until_future_complete() は executor 未指定だと内部で一時的な
別 Executor を作って同じノードを二重に spin してしまい、購読コールバック
の二重発火など不定な挙動を招きうるため、呼び出し側の Executor を明示的に
渡す。ノード側は MultiThreadedExecutor + ReentrantCallbackGroup を使い、
サービス呼び出し元のコールバックと応答コールバックが並行実行できるように
すること（詳細は config_manager.py のモジュール docstring 参照）。
"""

import rclpy


def call_and_wait(node, client, request, timeout_sec: float):
    """対象サービスを呼び、完了 or タイムアウトまで待つ。

    戻り値: (result, error_message) のどちらか一方が None。
    """
    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        node, future, executor=node._executor, timeout_sec=timeout_sec)
    if not future.done():
        future.cancel()
        return None, "応答がタイムアウトしました"
    return future.result(), None
