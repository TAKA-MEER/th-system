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

import glob
import math
import os
import signal
import subprocess
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


def _resolved(resolved: dict[str, tuple[str, Any]], name: str) -> Any:
    """`resolve_registry()` の結果から1つ引く。placeholder のまま解決できない
    ものは、この場で呼び出し側テストを明示的に落とす(T-1と同じ規約)。
    `fault_params` / `limiter_param` の両方から使う共通ヘルパー。"""
    status, value = resolved[name]
    if status == 'placeholder':
        pytest.fail(
            f"registry.yaml の {name} が placeholder のまま解決できない "
            f"(値={value!r})。registry.yaml の該当行のコメント・derived_from を"
            f"確認すること。")
    return value


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

    return {
        'lidar_timeout_ms': _resolved(resolved, 'lidar_timeout_ms'),
        'esp32_timeout_ms': _resolved(resolved, 'esp32_timeout_ms'),
        'cmd_vel_stale_ms': _resolved(resolved, 'cmd_vel_stale_ms'),
    }


# ---------------------------------------------------------------------------
# limiter_param（このパケットで追加。故障注入 1・2・11 が使う）
# ---------------------------------------------------------------------------
@pytest.fixture(scope='session')
def limiter_resolved() -> dict[str, tuple[str, Any]]:
    """`obstacle_limiter` 関連の registry 値の**生の**解決結果 `{name: (status, value)}`。

    `fault_params` と違ってここでは placeholder を即座に fail させない。
    `v_reverse`（後退時の上限）は現時点で registry.yaml 上 **placeholder** のまま
    （`blind_clearance_m` が `derived_from` だが、その `blind_clearance_m` 自体が
    `status: placeholder`。コメント「O-a2 が決まるまで placeholder」）。もしここで
    即座に fail させると、`v_reverse` を使わない他のテスト（1・11・control）まで
    巻き添えで落ちてしまう。実際に必要なテスト（2）だけが `limiter_param('v_reverse')`
    を呼んだ時点で落ちるようにするため、値の取り出しは `limiter_param` に遅延させる。
    """
    from th_params import export as params_export

    rows = _registry_rows()
    return params_export.resolve_registry(rows)


@pytest.fixture
def limiter_param(limiter_resolved):
    """`limiter_param('v_max')` の形で1つずつ registry 値を引く callable。

    `obstacle_limiter_core.hpp` の `ObstacleLimiterParams` に対応する値
    （`obstacle_floor_distance_m` / `hysteresis_band_m` / `brake_accel_mps2` /
    `v_max` / `v_reverse` 等）や `safety_monitor` のタイムアウト
    （`limiter_dead_ms` 等）を、T-1 の規約（テストに数値を直書きしない）に
    従って registry.yaml から引くための窓口。
    """

    def _get(name: str) -> Any:
        return _resolved(limiter_resolved, name)

    return _get


# ---------------------------------------------------------------------------
# sim_stack — gazebo.launch.py を subprocess で起動し、観測可能な条件を
# ポーリングして立ち上がりを待つ。後始末は SIGTERM（プロセスグループ全体）。
# ---------------------------------------------------------------------------

# gazebo.launch.py sim:=true の実測起動時間は 40〜45 秒（作業指示に記載）。
# 固定 sleep は使わず、後述の _ready() 条件をポーリングして待つ。ここでの
# 90 秒は「それでも上がってこなければ環境異常として明示的に fail する」ための
# 上限であり、合否判定のしきい値ではない（T-1 の対象外。安全パラメータではない
# テスト基盤側のタイムアウト）。
_DEFAULT_READY_TIMEOUT_SEC = 90.0


class SimStackHandle:
    """`sim_stack` フィクスチャが返すハンドル。テスト本体は基本的に
    `ros_node` 経由で ROS トピック/サービスだけを見ればよく、`proc` を
    直接操作する必要はない（例外: 故障注入11のように、スタック内の
    特定ノードだけを個別に SIGKILL したい場合。その場合も対象PIDは
    `find_pids_by_exe_basename()` で ROS 側から探す。プロセスグループ
    全体の後始末は本フィクスチャの finalizer が担う）。
    """

    def __init__(self, proc: subprocess.Popen, log_path: str, launch_args: dict[str, str]):
        self.proc = proc
        self.log_path = log_path
        self.launch_args = launch_args


def _terminate_process_group(proc: subprocess.Popen, grace_sec: float = 20.0) -> None:
    """プロセスグループ全体に SIGTERM を送り、終了を待つ。

    CLAUDE.md「ノードを kill -9 で落とすことを繰り返すと DDS discovery が壊れる」
    を踏まえ、既定は SIGTERM のみ（`kill -TERM`）。`gazebo.launch.py` は
    `ros2 launch` が子プロセス群を1つのプロセスグループにまとめるので、
    `os.killpg` でグループごと落とす（`ros2 launch` 自身は SIGINT/SIGTERM を
    受けて子へ伝播させる設計。個々の子を1つずつ探して TERM するより確実）。

    唯一の例外: 2回 SIGTERM を送っても `grace_sec` 以内に終了しないという
    異常系だけ、最終手段として SIGKILL へ 1 回だけ escalate する。これは
    CLAUDE.md の警告（「繰り返し kill -9 する」デバッグ運用）とは別の状況だと
    判断した——SIGTERM に応答しないプロセス群をここで見逃すと、同じコンテナで
    続けて動く次の ctest エントリ（fault_injection_02 等）に Gazebo/ROS
    プロセスが居座ったまま混入し、そちらの結果まで壊れる。1回きりの
    最終手段としての SIGKILL のほうが、その実害より小さいと判断した。
    """
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    for attempt_grace in (grace_sec, grace_sec / 2.0):
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=attempt_grace)
            return
        except subprocess.TimeoutExpired:
            continue

    # 最終手段（上記docstring参照）。
    try:
        os.killpg(pgid, signal.SIGKILL)
        proc.wait(timeout=10.0)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        pass


def _cleanup_idle_dds_shm() -> list[str]:
    """`/dev/shm/fastrtps_*`（FastRTPS の共有メモリ transport の残骸）のうち、
    **どのプロセスも開いていないもの**だけを削除する。

    CLAUDE.md「kill -9 を繰り返すと DDS discovery が壊れる」への対策。
    `/proc/[0-9]*/fd/*` を readlink してどのファイルが実際に開かれているかを
    先に集め、その集合に無いものだけを消す（＝現在生きているどの ROS
    プロセスの共有メモリも壊さない）。ホスト上の別プロセスが同名の
    セグメントを保持していても、開いている限りは対象から除外されるので、
    「実は使用中だったものを誤って消す」方向の事故は起きない設計。
    掃除しても直らないケース（CLAUDE.md に記載）もあるため、これは
    ベストエフォートの緩和策であり、確実な対策ではないことに注意。
    """
    removed: list[str] = []
    try:
        shm_paths = sorted(set(
            glob.glob('/dev/shm/fastrtps_*') + glob.glob('/dev/shm/sem.fastrtps_*')))
    except OSError:
        return removed
    if not shm_paths:
        return removed

    held: set[str] = set()
    try:
        for fd_link in glob.glob('/proc/[0-9]*/fd/*'):
            try:
                target = os.readlink(fd_link)
            except OSError:
                continue
            if target.startswith('/dev/shm/'):
                held.add(target)
    except OSError:
        pass

    for path in shm_paths:
        if path in held:
            continue
        try:
            os.remove(path)
            removed.append(path)
        except OSError:
            pass
    return removed


def find_pids_by_exe_basename(basename: str) -> list[int]:
    """`/proc/<pid>/cmdline` の argv[0] のベース名が完全一致する PID を探す。

    `pkill -f <pattern>` は部分一致でコマンドライン全体を見るため、
    CLAUDE.md に記録された「`bash -lc` 経由で自分自身の引数文字列に
    マッチして自滅した」ような誤爆をしうる。ここでは argv[0]（実行ファイル
    パス）のベース名の完全一致だけを見ることで、たとえば `obstacle_limiter`
    という文字列が `obstacle_limiter.yaml` という `--params-file` 引数に
    含まれていても誤って拾わないようにしている。
    """
    pids: list[int] = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                raw = f.read()
        except OSError:
            continue
        if not raw:
            continue
        argv0 = raw.split(b'\x00', 1)[0].decode('utf-8', 'replace')
        if os.path.basename(argv0) == basename:
            pids.append(int(entry))
    return pids


def place_entity_state(node, model_name: str, x: float, y: float, z: float = 0.85,
                        yaw: float = 0.0, timeout_sec: float = 10.0) -> None:
    """既存の `/gazebo/set_entity_state`（`libgazebo_ros_state` プラグイン）を
    呼び、既存の Gazebo モデルを瞬間移動させる。

    `th_perception/scripts/obstacle_mover.py` が "wanderer" を動かすのと
    **同じサービス・同じ流儀**（新しいサービスは作らない。
    `DetailedDesign-wp2.md` `WP-TEST-01` §3 の制約）。obstacle_mover.py 自体は
    ランダム歩行で非決定的なため、故障注入 1・2・11 のような「決まった位置に
    障害物を置いて再現性のある試験をしたい」場面ではこちらを直接呼ぶ
    （`obstacle:=false` で obstacle_mover ノード自体は起動せず、モデルだけが
    世界に残っている状態を前提にする）。
    """
    import rclpy
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState
    from geometry_msgs.msg import Point, Pose, Quaternion

    cli = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    try:
        if not cli.wait_for_service(timeout_sec=timeout_sec):
            pytest.fail(
                '/gazebo/set_entity_state が見つからない '
                '(Gazebo の libgazebo_ros_state プラグインが起動していない?)')
        req = SetEntityState.Request()
        state = EntityState()
        state.name = model_name
        state.pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)))
        state.reference_frame = 'world'
        req.state = state
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        result = future.result()
        if result is None or not result.success:
            pytest.fail(f"/gazebo/set_entity_state({model_name}) が失敗した: {result}")
    finally:
        node.destroy_client(cli)


@pytest.fixture
def sim_stack(request, ros_node, tmp_path):
    """`gazebo.launch.py` を `ros2 launch` の subprocess として起動し、
    観測可能な条件（固定 sleep ではない）をポーリングして立ち上がりを待つ。

    `request.param`（`pytest.mark.parametrize('sim_stack', [{...}], indirect=True)`
    で渡す）:
      - `launch_args`: dict[str, str]。`sim:=true rviz:=false` に追加で渡す
        launch 引数（例: `{'obstacle': 'false', 'robot_yaw': '3.14159'}`）。
      - `ready_timeout_sec`: 準備完了ポーリングの上限秒（既定 90 秒）。
      - `cleanup_dds_shm`: True なら後始末で `/dev/shm/fastrtps_*` の
        アイドル分を掃除する（故障注入11のような SIGKILL を伴う試験向け。
        `_cleanup_idle_dds_shm()` 参照）。

    【`rviz:=false` を既定にする理由】
    ヘッドレス環境では `rviz2` が必ず異常終了する（作業指示に記載の実測）。
    試験に不要なプロセスを1つ減らせるので起動も速くなる。

    【準備完了の判定に何を使ったか】
    固定 sleep ではなく、以下をすべて満たすことをポーリングする
    （CLAUDE.md「`ros2 node list` の結果だけで生死を判断しない」に従い、
    ノード一覧ではなくトピックの実際の状態を見る）:
      1. `/cmd_vel` に publisher が 1 つ以上いる（obstacle_limiter 起動済み。
         起動時に base_link<-laser_link TF を取得できないと起動失敗するため、
         これが立つ時点で URDF/TF/spawn も済んでいる）
      2. `/safety/fault_lock` に publisher が 1 つ以上いる（safety_monitor 起動済み）
      3. `/scan` を実際に1件以上受信した（Gazebo 本体・LiDAR センサプラグイン・
         ロボットのスポーンが完了している証拠）
    `gzclient` の異常終了（ヘッドレスでは想定内。作業指示に記載）はこの
    判定に影響しない――上記はいずれも `gzserver` 側・ROS ノード側の状態
    だけを見ており、GUI である `gzclient` の生死を見ていないため。

    【後始末】
    `_terminate_process_group()` で `ros2 launch` のプロセスグループ全体に
    SIGTERM を送る（`kill -TERM`。`kill -9` は使わない。例外は
    `_terminate_process_group` のdocstring参照）。

    【`th_ws/data/generated/` について】
    このフィクスチャ自身は生成物を読み書きしない。`gazebo.launch.py` が
    内部で `/root/th_data/generated/*.yaml` へ書き出すが、これは作業指示に
    「書かれること自体は許容し、テスト側では編集しない」とある既知の挙動
    なので、ここでは何もしない（意図的に無視している）。
    """
    params = getattr(request, 'param', None) or {}
    launch_args: dict[str, str] = {'sim': 'true', 'rviz': 'false'}
    launch_args.update({k: str(v) for k, v in (params.get('launch_args') or {}).items()})
    ready_timeout_sec = float(params.get('ready_timeout_sec', _DEFAULT_READY_TIMEOUT_SEC))
    cleanup_dds_shm = bool(params.get('cleanup_dds_shm', False))

    cmd = ['ros2', 'launch', 'th_bringup', 'gazebo.launch.py']
    cmd += [f'{k}:={v}' for k, v in launch_args.items()]

    log_path = tmp_path / 'gazebo_launch.log'
    log_f = open(log_path, 'w', encoding='utf-8')
    proc = subprocess.Popen(
        cmd, stdout=log_f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
        start_new_session=True,  # 新しいプロセスグループを作る(setsidと同等)。
    )

    def _finalize():
        try:
            _terminate_process_group(proc)
        finally:
            log_f.close()
            if cleanup_dds_shm:
                _cleanup_idle_dds_shm()

    request.addfinalizer(_finalize)

    scan_received: list[bool] = []
    scan_sub = None
    try:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        scan_sub = ros_node.create_subscription(
            LaserScan, '/scan', lambda _m: scan_received.append(True),
            qos_profile_sensor_data)

        deadline = time.monotonic() + ready_timeout_sec

        def _ready() -> bool:
            if proc.poll() is not None:
                pytest.fail(
                    f"gazebo.launch.py が準備完了前に終了した "
                    f"(exit={proc.returncode})。ログ: {log_path}")
            return (ros_node.count_publishers('/cmd_vel') >= 1
                    and ros_node.count_publishers('/safety/fault_lock') >= 1
                    and bool(scan_received))

        ok = _spin_until(ros_node, _ready, deadline)
    finally:
        if scan_sub is not None:
            ros_node.destroy_subscription(scan_sub)

    if not ok:
        pytest.fail(
            f"sim_stack: {ready_timeout_sec}秒以内に準備完了条件"
            f"(/cmd_vel publisher・/safety/fault_lock publisher・/scan 受信)"
            f"を満たさなかった。固定 sleep ではなくポーリングで待った上での"
            f"打ち切り。ログ: {log_path}")

    return SimStackHandle(proc=proc, log_path=str(log_path), launch_args=launch_args)


class DriveController:
    """`/cmd_vel_nav` と `/system/state` を一定周期で発行し続けるヘルパー。

    【なぜ `/system/state` も発行するか（設計書に無い自己判断）】
    `obstacle_limiter.cpp` の `update()` は `/system/state` が
    `state_stale_ms`（既定1500ms）以内に更新され続けていないと
    `applied_limit_mps` を強制的に 0.0 にする（`state_fresh` 分岐。
    `obstacle_limiter_core.cpp` §3.4.2）。ところが **`gazebo.launch.py` は
    `/system/state` の publisher である `state_manager` を一切起動しない**
    （`state_manager` は `bringup.launch.py` 専用。sim では
    `condition=UnlessCondition(sim)` にすら入っておらず、そもそも
    ノード定義自体が無い）。つまり **何もしなければ Gazebo 上の
    obstacle_limiter は常に速度上限 0.0 を出し、ロボットは絶対に動かない**
    （障害物の有無に関係なく）。これは実装管理者の作業指示にも設計書にも
    書かれていなかった、実装を読んで判明した事実。この空白を埋めるため、
    テスト自身が `/system/state` を代わりに発行する
    （新しいトピックは作らず、既存の観測点に**書く**側に回るだけ）。

    QoS はいずれも obstacle_limiter 側の購読 QoS に合わせている
    （`/system/state` は `reliable().transient_local()`。合わせないと
    そもそも接続が成立しない＝メッセージが永遠に届かない）。
    """

    def __init__(self, node, linear_x: float = 0.0, angular_z: float = 0.0,
                 speed_limit_name: str = 'v_max', mode: str = 'IDLE',
                 period_sec: float = 0.1):
        from geometry_msgs.msg import Twist
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from th_system_msgs.msg import SystemState

        self._node = node
        self._Twist = Twist
        self._SystemState = SystemState
        self.linear_x = linear_x
        self.angular_z = angular_z
        self._speed_limit_name = speed_limit_name
        self._mode = mode

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_cmd = node.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._pub_state = node.create_publisher(SystemState, '/system/state', state_qos)
        self._timer = node.create_timer(period_sec, self._tick)

    def _tick(self) -> None:
        twist = self._Twist()
        twist.linear.x = float(self.linear_x)
        twist.angular.z = float(self.angular_z)
        self._pub_cmd.publish(twist)

        state = self._SystemState()
        state.header.stamp = self._node.get_clock().now().to_msg()
        # "IDLE" 自体が th_state の18モード表と厳密に一致している必要はない
        # ——compute_source_class() が見るのは "MANUAL"/"TEACH_MANUAL" との
        # 不一致だけ（それ以外は全て AUTO 扱い）。
        state.mode = self._mode
        state.zone = 'OUT'
        state.jog_active = False
        state.auto_brake = True
        state.speed_limit = self._speed_limit_name
        self._pub_state.publish(state)

    def stop(self) -> None:
        self._node.destroy_timer(self._timer)


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


# ---------------------------------------------------------------------------
# 追加の合格判定（このパケットで追加。故障注入11「リミッタの死」向け）
# ---------------------------------------------------------------------------
@pytest.fixture
def assert_true_within(ros_node):
    """`assert_true_within(watcher, field, ms)`: `assert_stops_within` の真偽版。
    呼び出し時点から `ms` ミリ秒以内に `field` が truthy になることを確認する。
    `/safety/fault_lock`（`Bool.data`）が立つことの確認向け。"""

    def _assert(watcher: TopicWatcher, field: str, ms: float) -> None:
        deadline = time.monotonic() + ms / 1000.0
        ok = _spin_until(
            ros_node, lambda: any(_get_field(m, field) for _t, m in watcher.records), deadline)
        last = _get_field(watcher.records[-1][1], field) if watcher.records else '受信なし'
        assert ok, f"{field} が {ms}ms 以内に真にならなかった（直近の値: {last}）"

    return _assert


@pytest.fixture
def assert_equals_within(ros_node):
    """`assert_equals_within(watcher, field, expected, ms)`: 呼び出し時点から
    `ms` ミリ秒以内に、直近の受信値の `field` が `expected` と等しくなることを
    確認する。`/robot/mode` が特定の値（例: `RobotMode.ESTOP`）になることの
    確認向け（多値フィールドなので `assert_true_within` は使えない）。"""

    def _assert(watcher: TopicWatcher, field: str, expected: Any, ms: float) -> None:
        deadline = time.monotonic() + ms / 1000.0
        ok = _spin_until(
            ros_node,
            lambda: bool(watcher.records) and _get_field(watcher.records[-1][1], field) == expected,
            deadline)
        last = _get_field(watcher.records[-1][1], field) if watcher.records else '受信なし'
        assert ok, f"{field} が {ms}ms 以内に {expected!r} にならなかった（直近の値: {last!r}）"

    return _assert


@pytest.fixture
def assert_silent_within(ros_node):
    """`assert_silent_within(watcher, since, ms, grace_ms=150.0)`: `since` から
    `ms` ミリ秒経過するまで待ち、`since + grace_ms` より後に届いたメッセージが
    1件も無いことを確認する（プロセスの死亡＝ハートビート途絶の直接証拠。
    故障注入11の `/safety/limiter_status` 向け）。

    `grace_ms` を設ける理由: SIGKILL の瞬間、直前に送出済みだが配送が
    わずかに遅れているメッセージが `since` の直後に届くことがある
    （プロセス自体は既に死んでいるのに false negative になる）。20Hz
    ハートビートの数周期分（既定150ms）は許容し、それより後だけを厳密に見る。
    """

    def _assert(watcher: TopicWatcher, since: float, ms: float, grace_ms: float = 150.0) -> None:
        deadline = since + ms / 1000.0
        _spin_until(ros_node, lambda: time.monotonic() >= deadline, deadline)
        cutoff = since + grace_ms / 1000.0
        late = [t for t, _m in watcher.records if t > cutoff]
        assert not late, (
            f"since={since:.3f}（猶予{grace_ms}ms後={cutoff:.3f}）より後にも "
            f"{len(late)} 件のメッセージが届いた（プロセスが死んでいない可能性）")

    return _assert


@pytest.fixture
def assert_no_nonzero_after(ros_node):
    """`assert_no_nonzero_after(watcher, field, since, ms, atol=1e-6)`: `since`
    から `ms` ミリ秒の間に受信した全メッセージについて `field` が 0 であること
    （非ゼロが1件も無いこと）を確認する。`assert_zero_within`（「いずれか1件が
    0になる」）の裏返しで、「一度停止したあと再び動き出さないか」を見る用途
    （故障注入11の `/cmd_vel` 向け）。受信0件（トピックが完全に沈黙した）は
    合格として扱う——publisher が死んでメッセージが来なくなること自体は
    「非ゼロが来ていない」の一種であり、ここで判定したいのは「死んだ後に
    再び動き出さないか」であるため。
    """

    def _assert(watcher: TopicWatcher, field: str, since: float, ms: float,
                atol: float = 1e-6) -> None:
        deadline = since + ms / 1000.0
        _spin_until(ros_node, lambda: time.monotonic() >= deadline, deadline)
        offenders = [(t, _get_field(m, field)) for t, m in watcher.records
                     if since <= t <= deadline and abs(_get_field(m, field)) > atol]
        assert not offenders, (
            f"{field} が since={since:.3f} から {ms}ms の間に非ゼロを発行した: "
            f"{offenders[:3]}")

    return _assert
