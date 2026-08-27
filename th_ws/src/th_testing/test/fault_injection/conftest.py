"""
fault_injection/conftest.py — WP-TEST-01 共通の道具
=====================================================
`DetailedDesign-safety.md` §10（故障注入 13 項目）／
`DetailedDesign-wp2.md` `WP-TEST-01` §4.1 の骨格をここに置く。

**このパケットの範囲は「共通骨格 ＋ 実機専用3本 ＋ ctest登録」だけ。**
Gazebo を実際に起動する試験本体（1・2・3・5・6・7・9・11・12・13）は別パケットで書く。
このファイルの `sim_stack` フィクスチャと `assert_*` 系フィクスチャは、その次パケットが
呼び出す想定の**インターフェース**として今回作る。このパケット自身のテスト（実機専用3本と
対照ケースのプレースホルダ）からはまだ使われない。

【ホスト pytest からの分離】
このディレクトリの各テストファイルは意図的に `test_` で始まらないファイル名にしてある
（`case_NN_*.py` / `control.py`）。pytest の既定の収集パターンは `python_files =
test_*.py / *_test.py` で、これはディレクトリを再帰的に辿って集める場合にのみ働く。
そのため `pytest src/th_testing/test/` のような既存のホスト実行では一切収集されず、
このパケットで大量に増える「意図的に落ちる」プレースホルダがホストの
514 件（+control 等）を巻き込まない。一方 `ament_add_pytest_test` は colcon 側で
ファイルをフルパス指定して起動するため、ファイル名が `test_` で始まるかどうかに
関係なく実行できる（pytest はコマンドラインで明示されたパスを python_files の
パターンとは無関係に収集する。ローカルで
`python3 -m pytest fault_injection/case_01_....py` と直接指定しても同様に動く）。
モジュール内のテスト関数自体は pytest の既定どおり `test_` で始まる名前にする。

【なぜ `fault_injection/__init__.py` を置いているか】
このディレクトリの `conftest.py` と親の `test/conftest.py` は同じベース名
`conftest` を持つ。`test/` 側に `__init__.py` が無い（既存の 45+ 件のテストが
前提にしている構成なので変更しない）ため、pytest はデフォルトの import mode
（`prepend`。`__init__.py` が無いディレクトリは sys.modules にベース名だけで
登録する）では両方を同じ `sys.modules['conftest']` として扱おうとし、
どちらか一方が上書きされる。実際に `fault_injection/__init__.py` を置かずに
`pytest src/th_testing/test/` を再帰実行すると、`test_transition_table.py` の
`import conftest; conftest.th_state_config_dir()` が
`AttributeError: module 'conftest' has no attribute 'th_state_config_dir'`
で collection error になることを確認した（`fault_injection/conftest.py` の方が
`sys.modules['conftest']` を乗っ取ってしまうため）。`fault_injection/` にだけ
`__init__.py` を置くと、この配下は `fault_injection` パッケージとして扱われ、
中の `conftest.py` は `fault_injection.conftest` という別名で import されるため
衝突しない（`test/` 側の `_repo_root` の意味は変えていない。あくまで
`fault_injection/` サブディレクトリだけをパッケージ化する変更）。

【T-1: 合格条件の時間はパラメータから引く】
`fault_params` フィクスチャが `th_params/config/registry.yaml`（正本）を
`th_params.export.resolve_registry()` でその場で解決した値を返す。
`th_ws/data/generated/*.yaml`（生成物）を読まない・書かない理由:
  - 生成物はユーザーの Docker 検証で頻繁に上書きされる（このパケットの作業指示に
    明記されている）。生成物ファイルに依存するとテストの再現性がその都合に左右される。
  - `registry.yaml → resolve_registry()` は決定論的（`test_digest_stable` が担保）。
    `build_node_outputs()` は同じ `resolved` 値からノード別 YAML を組み立てるだけなので、
    生成物ファイルを実際に書き出した場合と数値的に同じ結果になる。
  - ファイルを一切書かないので「`th_ws/data/generated/` を編集しない」という
    このパケットの制約も自動的に満たす。
  - 「起動中のノードから `ros2 param get` 相当で読む」案は、`sim_stack` が
    未実装のこのパケットでは検証できず、実ノードを一つも起動しないプレースホルダの
    ためにこの方式を選ぶ理由がない。次パケットで実ノードを起動するようになったら、
    「起動済みノードの実パラメータ」と `fault_params` の値が一致することを確認する
    テストを別途足すとよい（このパケットでは書かない）。
"""
from __future__ import annotations

import os
import sys
import time
from typing import Any, Callable

import pytest
import yaml

# ---------------------------------------------------------------------------
# パス解決（親の test/conftest.py と同じ流儀。th_params を sys.path に足す）
# ---------------------------------------------------------------------------
_TEST_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))  # th_testing/test
_SRC_ROOT = os.path.abspath(os.path.join(_TEST_ROOT, '..', '..'))  # th_ws/src
_TH_PARAMS = os.path.join(_SRC_ROOT, 'th_params')
if _TH_PARAMS not in sys.path:
    sys.path.insert(0, _TH_PARAMS)


def _registry_rows() -> list[dict]:
    registry_path = os.path.join(_TH_PARAMS, 'config', 'registry.yaml')
    with open(registry_path, encoding='utf-8') as f:
        return yaml.safe_load(f)


# T-1 の唯一の例外（DetailedDesign-safety.md §10 の注・#6行）: 100ms は
# 「フォルト検知 → 停止」（層3。safety_monitor が /safety/fault_lock 等を
# 立ててから twist_mux が /cmd_vel をゼロにするまで）の応答時間そのものであり、
# lidar_timeout_ms 等と違って registry.yaml のパラメータではなく設計上の定数。
# 次パケットの故障注入6 (test_06_fault_to_stop) がこの値を使う想定。
FAULT_TO_STOP_LAYER3_MS = 100  # noqa: E305


@pytest.fixture(scope='session')
def fault_params() -> dict[str, Any]:
    """故障注入の合格条件に使うタイムアウト値（T-1）。上のモジュール docstring 参照。"""
    from th_params import export as params_export

    rows = _registry_rows()
    resolved = params_export.resolve_registry(rows)

    def _value(name: str):
        status, value = resolved[name]
        if status == 'placeholder':
            pytest.fail(
                f"registry.yaml の {name} が placeholder のまま解決できない "
                f"（DEBT-3 未解除の可能性。generated/safety_monitor.yaml の"
                f"導出値化が必要）。")
        return value

    return {
        'lidar_timeout_ms': _value('lidar_timeout_ms'),
        'esp32_timeout_ms': _value('esp32_timeout_ms'),
        'cmd_vel_stale_ms': _value('cmd_vel_stale_ms'),
    }


# ---------------------------------------------------------------------------
# sim_stack（次パケットで実装）
# ---------------------------------------------------------------------------
@pytest.fixture
def sim_stack(request):
    """`gazebo.launch.py` を stage 指定で起動するフィクスチャ。

    **このパケットでは中身を実装しない。**次パケットで、`request.param`
    （または `pytest.mark.parametrize` 等）で受け取った stage を使って
    `gazebo.launch.py` を `launch_testing` 経由で立ち上げ、`ReadyToTest()` 後の
    プロセスハンドルを返す実装に置き換える想定。

    【`pytest.fail` を選んだ理由（`NotImplementedError` にしなかった理由）】
    - `NotImplementedError` は「コードパスが未実装」を表す汎用の組み込み例外で、
      呼び出し側の広い `except Exception` に飲まれて原因が分かりにくくなる余地がある。
    - この `th_testing` の親 `conftest.py`（`test/conftest.py`）は既に
      `resolve_spec_dir()` の失敗を `spec_modes_file` フィクスチャの中で
      `pytest.fail(str(e))` として明示的に落とす流儀を採っている。同じ流儀に揃える
      ことで、「意図的に未実装として落ちている」ことが pytest の出力
      （`Failed: ...`）からそのまま読み取れる。
    - 実機専用3本（T-3）も `pytest.fail` で明示的に落とす方針にしたため、
      「プレースホルダは `pytest.fail`、`pytest.skip` は使わない」という
      一貫したルールになる（7・13 の意図的な `pytest.skip` は段階ゲート待ちという
      別の理由によるもので、これとは区別する。下記の該当テストファイル参照）。
    """
    pytest.fail(
        "sim_stack は未実装（このブランチ feat/wp-test-01-fault-injection の"
        "作業指示により、このパケットの範囲は「共通骨格 + 実機専用3本 + ctest登録」"
        "だけ。Gazebo を実際に起動する試験本体は次パケットで実装する）。"
        "DetailedDesign-wp2.md WP-TEST-01 §4.1 参照。"
    )


# ---------------------------------------------------------------------------
# 時間つきの合格判定（T-2: 5 と 6 は別のアサーションを使うこと）
# ---------------------------------------------------------------------------
def _get_field(msg: Any, field_path: str) -> Any:
    """`'linear.x'` のようなドット区切りのフィールドパスを辿って値を取り出す。"""
    obj = msg
    for part in field_path.split('.'):
        obj = getattr(obj, part)
    return obj


class TopicWatcher:
    """トピックを購読し `(受信時刻[time.monotonic()], msg)` を貯め続ける。

    **次パケットの試験本体は、故障注入の前に watcher を作っておくこと。**
    `assert_zero_within` の `since`（故障検知時刻）はテスト本体側の
    `time.monotonic()` と揃える必要があるため、watcher を故障の後から作ると、
    故障直後〜watcher 作成までの間のメッセージを取りこぼして
    偽陽性（実際は遅いのに合格と判定してしまう）になる。
    """

    def __init__(self, node, topic: str, msg_type, qos: int = 10):
        self.records: list[tuple[float, Any]] = []
        self._node = node
        self._sub = node.create_subscription(msg_type, topic, self._on_msg, qos)

    def _on_msg(self, msg: Any) -> None:
        self.records.append((time.monotonic(), msg))

    def destroy(self) -> None:
        self._node.destroy_subscription(self._sub)


def _spin_until(node, predicate: Callable[[], bool], deadline: float) -> bool:
    import rclpy

    while True:
        if predicate():
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return predicate()
        rclpy.spin_once(node, timeout_sec=min(0.05, remaining))


@pytest.fixture
def ros_node():
    """テスト用の rclpy クライアントノードを 1 つ用意する。

    `sim_stack` とは独立。`launch_testing` で別プロセスとして立ち上げた
    ノード群に対して、テストプロセス側から購読するためだけの通常の rclpy ノード。
    """
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('fault_injection_test_client')
    yield node
    node.destroy_node()


@pytest.fixture
def assert_stops_within(ros_node):
    """`assert_stops_within(watcher, field, ms)` を返すフィクスチャファクトリ。

    呼び出し時点から `ms` ミリ秒以内に `watcher` が監視するトピックの `field` が
    0 になることを確認する。故障注入 1・2・3・11（「停止する」）向け。
    """

    def _assert(watcher: TopicWatcher, field: str, ms: float, atol: float = 1e-6) -> None:
        deadline = time.monotonic() + ms / 1000.0
        ok = _spin_until(
            ros_node,
            lambda: any(abs(_get_field(m, field)) <= atol for _t, m in watcher.records),
            deadline)
        last = _get_field(watcher.records[-1][1], field) if watcher.records else '受信なし'
        assert ok, f"{field} が {ms}ms 以内に 0 にならなかった（直近の値: {last}）"

    return _assert


@pytest.fixture
def assert_fault_within(ros_node):
    """`assert_fault_within(watcher, fault_type, ms)` を返すフィクスチャファクトリ。

    `watcher`（`/safety/fault` を購読している想定）に、呼び出し時点から `ms`
    ミリ秒以内に `fault_type` が active な `FaultStatus` が届くことを確認する。
    故障注入 5（「通信断 → フォルト検知」）向け。**T-2**: 6（フォルト検知 → 停止）は
    これと別に `assert_zero_within` で確認すること。1本にまとめない。
    """

    def _assert(watcher: TopicWatcher, fault_type: str, ms: float) -> None:
        deadline = time.monotonic() + ms / 1000.0
        ok = _spin_until(
            ros_node,
            lambda: any(m.active and m.fault_type == fault_type for _t, m in watcher.records),
            deadline)
        assert ok, f"{fault_type} が {ms}ms 以内に /safety/fault で active にならなかった"

    return _assert


@pytest.fixture
def assert_zero_within(ros_node):
    """`assert_zero_within(watcher, field, since, ms)` を返すフィクスチャファクトリ。

    `watcher` が `since`（`time.monotonic()` の値。例: 故障検知時刻）より前から
    監視を続けている前提で、`since` から `ms` ミリ秒以内に `field` が 0 に
    なった記録があることを確認する。`assert_stops_within` との違いは基準時刻が
    「呼び出し時点」ではなく「過去のイベント時刻」であること。
    故障注入 6（フォルトから `FAULT_TO_STOP_LAYER3_MS`＝100ms 以内）・
    12（stale から `cmd_vel_stale_ms` 以内）向け。
    """

    def _assert(watcher: TopicWatcher, field: str, since: float, ms: float,
                atol: float = 1e-6) -> None:
        deadline = since + ms / 1000.0
        ok = _spin_until(
            ros_node,
            lambda: any(t <= deadline and abs(_get_field(m, field)) <= atol
                        for t, m in watcher.records),
            deadline)
        assert ok, (
            f"{field} が since={since:.3f} から {ms}ms 以内（締切 {deadline:.3f}）に "
            f"0 にならなかった")

    return _assert
