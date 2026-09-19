"""
test_service_call.py
====================
WS-9U: call_and_wait は「自分の実行スレッドから MultiThreadedExecutor を再入
spin する」実装をやめ、`call_async` + `future.done()` のポーリングにした。
再入 spin は `MultiThreadedExecutor._spin_once_impl` の共有ジェネレータ
（`wait_for_ready_callbacks` の `_cb_iter`）と共有リスト `_futures` を 2 スレッドで
壊し、実機で replay_runner が長距離経路の再読込中に exit 1 で落ちた。

ROS2 なし・純粋 Python。
"""
import ast
import os
import sys
import time

_PKG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager', 'th_config_manager'))
SERVICE_CALL = os.path.join(_PKG, 'service_call.py')

sys.path.insert(0, _PKG)

from service_call import call_and_wait   # noqa: E402


# ── ast: 再入 spin をしていない ──────────────────────────────────────
def _src():
    with open(SERVICE_CALL, encoding='utf-8') as f:
        return f.read()


def test_service_call_does_not_spin_the_executor():
    """spin_until_future_complete / executor spin を一切呼ばないこと。

    変異チェック: `rclpy.spin_until_future_complete(...)` を戻すと赤くなる。
    """
    tree = ast.parse(_src(), filename=SERVICE_CALL)
    func = next((n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == 'call_and_wait'), None)
    assert func is not None
    called = set()
    for n in ast.walk(func):
        if isinstance(n, ast.Call):
            if isinstance(n.func, ast.Attribute):
                called.add(n.func.attr)
            elif isinstance(n.func, ast.Name):
                called.add(n.func.id)
    assert 'spin_until_future_complete' not in called, (
        'call_and_wait が spin_until_future_complete を呼んでいる（再入 spin。WS-9U で禁止）')
    assert not any('spin' in c for c in called), f'spin 系呼び出しが残っている: {called}'
    assert 'call_async' in called, 'call_async を使っていない'
    assert 'sleep' in called, 'ポーリング（time.sleep）を使っていない'


# ── 機能: fake client で戻り値を確認（rclpy 不要）────────────────────
class _DoneFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result

    def cancel(self):
        pass


class _NeverFuture:
    def __init__(self):
        self.cancelled = False

    def done(self):
        return False

    def result(self):
        raise AssertionError('done() が False なのに result() が呼ばれた')

    def cancel(self):
        self.cancelled = True


class _FakeClient:
    def __init__(self, future):
        self._future = future
        self.calls = 0

    def call_async(self, request):
        self.calls += 1
        return self._future


def test_call_and_wait_returns_result_on_completion():
    fut = _DoneFuture('OK')
    client = _FakeClient(fut)
    result, err = call_and_wait(node=None, client=client, request=object(), timeout_sec=1.0)
    assert result == 'OK'
    assert err is None
    assert client.calls == 1


def test_call_and_wait_times_out_and_cancels():
    fut = _NeverFuture()
    client = _FakeClient(fut)
    t0 = time.monotonic()
    result, err = call_and_wait(node=None, client=client, request=object(), timeout_sec=0.2)
    elapsed = time.monotonic() - t0
    assert result is None
    assert err is not None and 'タイムアウト' in err
    assert fut.cancelled is True
    assert 0.15 <= elapsed < 1.0    # timeout_sec 前後で返る（無限ループしない）
