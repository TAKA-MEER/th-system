"""test_params_launch.py — th_bringup/launch/params_generation.py と
bringup.launch.py / gazebo.launch.py の生成配線を検証する（WP-PARAM-02 §7）。

【実行に関する注意】このファイルは ROS2 を起動しない。
`launch` / `launch_ros` は import しない（`params_generation.py` がそれらを
`make_opaque_function()` の中でだけ遅延 import する設計になっているため）。
検査は次の2種類だけで完結させる:
  - `params_generation.run_generation()` / `reshape_twist_mux()` を直接呼ぶ純粋単体試験
    （`python3 -m th_params.export` をサブプロセスで呼ぶが、これは registry.yaml を
    読んで YAML を書くだけの CLI であり ROS2 ノードではない。DDS 通信は発生しない）
  - launch ファイルのソーステキストに対する grep / AST 検査（構造の確認のみ。
    `generate_launch_description()` を実際に呼び出しはしない）
"""
from __future__ import annotations

import ast
import os
import sys
import tempfile

import pytest
import yaml

_REPO_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_LAUNCH_DIR = os.path.join(_REPO_SRC, "th_bringup", "launch")
_PARAMS_SRC = os.path.join(_REPO_SRC, "th_params")

for _p in (_LAUNCH_DIR, _PARAMS_SRC):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402

BRINGUP_PY = os.path.join(_LAUNCH_DIR, "bringup.launch.py")
GAZEBO_PY = os.path.join(_LAUNCH_DIR, "gazebo.launch.py")
REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def _subprocess_env() -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = _PARAMS_SRC + os.pathsep + env.get("PYTHONPATH", "")
    return env


# ============================================================================
# G-1: 生成はノード起動より前に同期実行する（OpaqueFunction の評価順）
# ============================================================================


def test_generated_before_nodes():
    """G-1: params_generation_action が、両 launch ファイルの中で
    Node(...) を1つも起動する前に構築・登録されていることをソーステキストで検査する。"""

    # --- bringup.launch.py: nodes = [] の後、最初の nodes.append(...) が
    #     params_generation_action でなければならない（Python リストは append 順を
    #     保つので、これが LaunchDescription(args + nodes) の実際の並び順になる）。
    tree = ast.parse(_read(BRINGUP_PY), filename=BRINGUP_PY)
    func = _find_function(tree, "generate_launch_description")
    appends = _find_nodes_append_calls(func, list_name="nodes")
    assert appends, "bringup.launch.py: nodes.append(...) が1つも見つからない"
    first_arg = appends[0]
    assert isinstance(first_arg, ast.Name) and first_arg.id == "params_generation_action", (
        "bringup.launch.py: nodes への最初の append が params_generation_action でない "
        f"(実際: {ast.dump(first_arg)})")

    # --- gazebo.launch.py: 最終的な LaunchDescription(...) の引数リストの中で、
    #     params_generation_action が scenario_action や common_nodes より前にあること。
    src = _read(GAZEBO_PY)
    return_start = src.index("return LaunchDescription(")
    return_src = src[return_start:]
    gen_idx = return_src.index("params_generation_action")
    scenario_idx = return_src.index("scenario_action,")
    common_nodes_idx = return_src.index("*common_nodes")
    assert gen_idx < scenario_idx, (
        "gazebo.launch.py: params_generation_action が scenario_action より後にある")
    assert gen_idx < common_nodes_idx, (
        "gazebo.launch.py: params_generation_action が common_nodes より後にある")


def _find_function(tree: ast.AST, name: str) -> ast.FunctionDef:
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"関数 {name} が見つからない")


def _find_nodes_append_calls(func: ast.FunctionDef, list_name: str) -> list[ast.AST]:
    """`<list_name>.append(<arg>)` 呼び出しを、関数本体を上から辿った順に集め、
    その引数 (AST ノード) のリストを返す（ネストしたブロックの中も含めて順序どおり）。"""
    result = []
    for node in ast.walk(func):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "append"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == list_name
                and node.args):
            result.append((node.lineno, node.args[0]))
    result.sort(key=lambda t: t[0])
    return [arg for _lineno, arg in result]


# ============================================================================
# G-2: アサーション違反なら launch を止める
# ============================================================================


def test_assertion_stops_launch():
    """G-2: registry を意図的に壊す（A2a を破る）と run_generation() が例外を投げる。
    完了条件③と同じ壊し方（floor_margin_m を 99.0 にする）。"""
    with tempfile.TemporaryDirectory() as tmp:
        bad_registry = os.path.join(tmp, "registry.yaml")
        rows = yaml.safe_load(_read(REGISTRY_YAML))
        for row in rows:
            if row["name"] == "floor_margin_m":
                row["value"] = 99.0
                row["status"] = "given"
        with open(bad_registry, "w", encoding="utf-8") as f:
            yaml.safe_dump(rows, f, allow_unicode=True)

        out_dir = os.path.join(tmp, "generated")
        with pytest.raises(pg.GenerationError):
            pg.run_generation(
                stage=2, sim=False, nodes=["obstacle_limiter"],
                out_dir=out_dir, registry_path=bad_registry, env=_subprocess_env())


def test_assertion_ok_does_not_raise_and_writes_files():
    """G-2 の裏側: 正常な registry なら例外を投げず、generated/ に書き出す
    （FMEA①: 直前に古いファイルを置いておき、削除されて作り直されることも確認する）。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        os.makedirs(out_dir)
        stale_path = os.path.join(out_dir, "stale_leftover.yaml")
        with open(stale_path, "w", encoding="utf-8") as f:
            f.write("stale: true\n")

        pg.run_generation(stage=1, sim=True, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML, env=_subprocess_env())

        assert not os.path.exists(stale_path), "FMEA①: 古い generated/ の中身が残っている"
        assert os.path.isfile(os.path.join(out_dir, "twist_mux.yaml"))
        assert os.path.isfile(os.path.join(out_dir, "params_digest.json"))


def test_generation_success_does_not_swallow_export_stderr(capsys):
    """G-2 の裏側（続き）: export.py が終了コード 0（警告のみ）で終わっても、
    その stderr を run_generation() が握り潰さず launch 側の stderr へ出すこと。

    実物の registry.yaml + 実機条件（stage=1, sim=False）は、この変更の時点で
    A6（esp32_timeout_ms が WATCHDOG_MS 以下）の警告を出す状態になっている
    （eed7ee1 で拒否→警告化された）。ただし「A6 の具体的な文言」自体には
    依存させず、「export.py が stderr に何か書いた → run_generation() がそれを
    素通しした」ことだけを見る（将来 registry の値が変わって A6 が警告しなく
    なっても、このテストの意図が壊れないように）。
    """
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")

        # 前提確認: 実機条件で export.py 自体が stderr に何か書くこと
        # （書かない状態になったら、このテストは「素通し」を検証できていない
        # ので、意図を明示して先に失敗させる）。
        import subprocess
        cmd = [sys.executable, "-m", "th_params.export",
               "--registry", REGISTRY_YAML, "--out", out_dir, "--stage", "1",
               "--nodes", ",".join(pg.REGISTRY_NODES)]
        precheck = subprocess.run(cmd, capture_output=True, text=True, env=_subprocess_env())
        assert precheck.returncode == 0, "前提が崩れている: 実機条件の registry がそもそも失敗する"
        assert precheck.stderr, (
            "前提が崩れている: 実機条件で export.py が stderr に何も書かなくなった"
            "（= このテストが検証したい『成功時 stderr の素通し』を再現できていない。"
            "registry.yaml の値が変わって A6 等の警告が出なくなった可能性がある）")

        capsys.readouterr()  # ここまでの出力を捨てる
        pg.run_generation(stage=1, sim=False, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML, env=_subprocess_env())
        captured = capsys.readouterr()

        assert captured.err, "run_generation() が成功時の stderr を握り潰している"
        assert "[params_generation]" in captured.err, (
            "stderr に出所を示す前置きが無い")


# ============================================================================
# G-3: twist_mux.yaml も生成対象（階層構造への組み直し）
# ============================================================================


def test_twist_mux_generated():
    """G-3: reshape_twist_mux() が registry 由来のタイムアウト3種を twist_mux の
    実際の階層パラメータ名 (topics.<name>.timeout) に差し込み、トピック名・優先度・
    ロックのタイムアウトは変えないこと。"""
    flat = {
        "manual_joy_timeout": 1.0,
        "behavior_cmd_timeout_s": 0.5,
        "nav_cmd_timeout_s": 0.5,
    }
    nested = pg.reshape_twist_mux(flat)
    params = nested["twist_mux"]["ros__parameters"]

    assert params["topics"]["manual_joy"]["timeout"] == 1.0
    assert params["topics"]["behavior"]["timeout"] == 0.5
    assert params["topics"]["nav"]["timeout"] == 0.5
    # 配線情報（トピック名・優先度・ロックのタイムアウト）は registry に無いので不変
    assert params["topics"]["manual_joy"]["topic"] == "/cmd_vel_manual"
    assert params["topics"]["manual_joy"]["priority"] == 30
    assert params["locks"]["estop"]["priority"] == 255
    assert params["locks"]["estop"]["timeout"] == 0.5

    # placeholder（None）は上書きしない（静的フォールバックのまま）
    nested2 = pg.reshape_twist_mux({"manual_joy_timeout": None})
    assert nested2["twist_mux"]["ros__parameters"]["topics"]["manual_joy"]["timeout"] == 1.0


def test_twist_mux_end_to_end_via_export():
    """G-3 の結合確認: 実際の registry.yaml → export.py → reshape の結果、
    twist_mux.yaml が階層構造になっていること（flat な生の consumers 出力のままでない）。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=1, sim=True, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML, env=_subprocess_env())

        with open(os.path.join(out_dir, "twist_mux.yaml"), encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        params = doc["twist_mux"]["ros__parameters"]
        assert "locks" in params and "topics" in params
        assert "manual_joy_timeout" not in params, (
            "flat な registry 名がそのまま残っている（reshape されていない）")
        assert params["topics"]["manual_joy"]["timeout"] == pytest.approx(1.0)


# ============================================================================
# G-4: bringup / gazebo の両方の parameters= を generated/ へ差し替える
# ============================================================================


def test_blind_angle_ranges_end_to_end_via_export():
    """O-4 の結合確認（2026-08-27 に表明を反転): 実際の registry.yaml →
    export.py → sanitize_node_params() の結果、lidar_filter.yaml /
    obstacle_limiter.yaml の**どちらにも blind_angle_ranges というキー自体が
    存在しない**こと（空配列で残っていない）。

    以前は「空配列のまま残ること」を検査していたが、これは Docker 実機での
    起動失敗（`gazebo.launch.py sim:=true` で obstacle_limiter・lidar_filter の
    両方が `parameter_value_from failed for parameter 'blind_angle_ranges':
    No parameter value set` で落ちた。実測で確認済み）を固定するテストだった。
    原因は ROS2 Humble の `rcl_yaml_param_parser` が空配列の parameter
    override をそもそも扱えないこと（`declare_parameter` の型推論の問題ではない
    ——素の `tf2_ros::static_transform_publisher` に空配列 1 個だけの
    params ファイルを渡しても同じ例外で落ちることを実測で確認済み）。
    `ParameterDescriptor` で型を明示しても override が空である限り解決できない。

    「死角なし」と「値が抜けた」の区別は `blind_calibrated`（`class: b` /
    `given`。N-8）という独立フラグが担うため、`blind_angle_ranges` というキー
    自体が生成物に存在しなくても意味は失われない——ノード側の
    `declare_parameter` の既定値（空配列 = マスクなし。安全側）がそのまま
    使われるだけ。

    stage=2・sim=False（A8 が blind_angle_ranges の blocking_from_stage:2 を見る条件）
    でも GenerationError にならず完走することも合わせて確認する
    （registry.yaml が status: measured になったことで A8 の対象から外れているはず）。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=2, sim=False, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML, env=_subprocess_env())

        with open(os.path.join(out_dir, "lidar_filter.yaml"), encoding="utf-8") as f:
            lidar_params = yaml.safe_load(f)["lidar_filter"]["ros__parameters"]
        assert "blind_angle_ranges" not in lidar_params, (
            "blind_angle_ranges が空配列のまま生成物に残っている"
            "（ROS2 Humble はこの override を扱えずノードが起動失敗する）")
        assert lidar_params["blind_calibrated"] is True

        obstacle_path = os.path.join(out_dir, "obstacle_limiter.yaml")
        assert os.path.exists(obstacle_path), "obstacle_limiter.yaml が生成されていない"
        with open(obstacle_path, encoding="utf-8") as f:
            obstacle_params = yaml.safe_load(f)["obstacle_limiter"]["ros__parameters"]
        assert "blind_angle_ranges" not in obstacle_params, (
            "blind_angle_ranges が空配列のまま生成物に残っている"
            "（ROS2 Humble はこの override を扱えずノードが起動失敗する）")
        assert obstacle_params["blind_calibrated"] is True


def test_both_launch_files_use_generated():
    """G-4: 両 launch ファイルが th_data/generated を最低1回は参照すること
    （完了条件①と同じ grep 検査。コメントの中の文字列だけでも通ってしまうので、
    実質的な配線の検査は _assert_generated_dir_wired() が別途行う）。"""
    for path in (BRINGUP_PY, GAZEBO_PY):
        src = _read(path)
        assert "th_data/generated" in src, f"{path}: th_data/generated を参照していない"

    # twist_mux は両方とも generated/twist_mux.yaml だけを読む（静的ファイルへの
    # 直接参照が残っていないこと。二重管理の防止）
    for path in (BRINGUP_PY, GAZEBO_PY):
        src = _read(path)
        assert "'twist_mux', 'config', 'twist_mux.yaml'" not in src.replace('"', "'"), (
            f"{path}: 静的な th_safety/config/twist_mux.yaml への参照が残っている")


# D5: 上の grep 検査は「コメントに文字列が出てくるだけ」でも合格してしまい、
# `parameters=` の実配線を全部消しても通る（完了条件①と同じ穴）。
# ここでは AST で実際の配線を検査する（test_generated_before_nodes と同じ流儀）。
_REQUIRED_GENERATED_FILES = frozenset(
    {"twist_mux.yaml", "safety_monitor.yaml", "lidar_filter.yaml", "esp32_bridge.yaml"})


def _imports_generated_dir(tree: ast.AST) -> bool:
    """`from params_generation import GENERATED_DIR` の形の import があるか。"""
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "params_generation":
            if any(alias.name == "GENERATED_DIR" for alias in node.names):
                return True
    return False


def _find_generated_dir_join_filenames(tree: ast.AST) -> list[str]:
    """`os.path.join(GENERATED_DIR, '<name>.yaml')` の呼び出しを全て探し、
    `<name>.yaml` の文字列部分を集めて返す。"""
    names: list[str] = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "join"
                and isinstance(node.func.value, ast.Attribute)
                and node.func.value.attr == "path"):
            continue
        if len(node.args) < 2:
            continue
        first_arg = node.args[0]
        if not (isinstance(first_arg, ast.Name) and first_arg.id == "GENERATED_DIR"):
            continue
        second_arg = node.args[1]
        if isinstance(second_arg, ast.Constant) and isinstance(second_arg.value, str):
            names.append(second_arg.value)
    return names


def _assert_generated_dir_wired(path: str) -> None:
    tree = ast.parse(_read(path), filename=path)
    assert _imports_generated_dir(tree), (
        f"{path}: `from params_generation import GENERATED_DIR` が無い")

    filenames = _find_generated_dir_join_filenames(tree)
    assert len(filenames) >= 4, (
        f"{path}: os.path.join(GENERATED_DIR, '<name>.yaml') が4件未満"
        f"（実際: {filenames}）。G-4 の parameters= 差し替えが失われている可能性がある")

    missing = _REQUIRED_GENERATED_FILES - set(filenames)
    assert not missing, (
        f"{path}: generated/ を参照する parameters= に {sorted(missing)} が無い"
        f"（実際に参照されているファイル: {filenames}）")


# ============================================================================
# D3: sanitize_node_params() — None / 空 list・dict を落とす（それ以外は残す）
# ============================================================================


def test_sanitize_node_params_drops_none():
    out = pg.sanitize_node_params({"a": None, "b": 1.0})
    assert "a" not in out
    assert out["b"] == 1.0


def test_sanitize_node_params_drops_empty_list_and_dict():
    out = pg.sanitize_node_params({"empty_list": [], "empty_dict": {}, "kept": [1, 2]})
    assert "empty_list" not in out
    assert "empty_dict" not in out
    assert out["kept"] == [1, 2]


def test_sanitize_node_params_keeps_falsy_non_empty_values():
    """0 / False / 空文字列は「値が無い」ことと違う。落としてはいけない
    （falsy を「無い」と混同するのはよくある間違い）。"""
    out = pg.sanitize_node_params({"zero": 0, "false": False, "empty_str": "", "zero_f": 0.0})
    assert out == {"zero": 0, "false": False, "empty_str": "", "zero_f": 0.0}


def test_sanitize_node_params_pure_no_mutation():
    src = {"a": None, "b": [1]}
    out = pg.sanitize_node_params(src)
    assert src == {"a": None, "b": [1]}, "入力を書き換えてはいけない（純粋関数）"
    assert out == {"b": [1]}


def test_sanitize_node_params_ceils_ms_floats_to_int():
    """導出パラメータが返す `*_ms` の float を整数へ切り上げること。

    このリポジトリの `*_ms` は 14 箇所すべて整数で宣言されている
    （`safety_monitor.cpp` / `obstacle_limiter.cpp` / `esp32_bridge.py` /
    `connectivity_checker.py` / `state_manager.py`）。rclcpp は int 宣言に
    double を渡すと `InvalidParameterTypeException` を投げてノードを abort
    させる（2026-08-31 に実機で `safety_monitor` が exit code -6 で落ちた）。
    """
    out = pg.sanitize_node_params({
        "lidar_timeout_ms": 308.0,
        "esp32_timeout_ms": 322.4,
    })
    assert out["lidar_timeout_ms"] == 308
    assert isinstance(out["lidar_timeout_ms"], int)
    # 切り上げ: 導出の下限（p99 + 余裕）を生成物の上でも下回らせない
    assert out["esp32_timeout_ms"] == 323, "切り捨て・四捨五入ではなく切り上げること"


def test_sanitize_node_params_ms_coercion_does_not_touch_others():
    """`_ms` で終わっても dict は対象外（`link_gap_p99_ms`）。int / bool /
    `_ms` で終わらない float は素通しすること。"""
    out = pg.sanitize_node_params({
        "link_gap_p99_ms": {"esp32": 161, "lidar": 154},   # dict は触らない
        "limiter_dead_ms": 250,                              # 既に int
        "runaway_ratio": 1.5,                                # _ms ではない float
        "v_max": 1.12,
    })
    assert out["link_gap_p99_ms"] == {"esp32": 161, "lidar": 154}
    assert out["limiter_dead_ms"] == 250 and isinstance(out["limiter_dead_ms"], int)
    assert out["runaway_ratio"] == 1.5 and isinstance(out["runaway_ratio"], float)
    assert out["v_max"] == 1.12 and isinstance(out["v_max"], float)


# ============================================================================
# D3 回帰試験: 実物の registry.yaml から生成した全 YAML に None / 空コンテナが
# 1つも残っていないこと（twist_mux.yaml を含む）
# ============================================================================


def test_generated_yaml_has_no_null_or_empty_containers():
    """D3: registry.yaml には `enabled_targets: []` のような
    `status: given, value: []` の行や、blocking でない placeholder（→ null）が
    実在する。生成された全 *.yaml（twist_mux.yaml 含む。params_digest.json は
    ROS2 パラメータファイルではないので対象外）の `ros__parameters` に
    None / 空 list / 空 dict が1つも無いことを検査する
    （残っていると ROS2 Humble の rcl_yaml_param_parser が override を解決
    できず、ノードが `parameter_value_from failed ... No parameter value set`
    で起動失敗する。Docker 実機での実測で確認済み）。

    **例外は無い**（2026-08-27 に撤去）。以前は `blind_angle_ranges` を
    `reshape_blind_angles()` が空配列で明示的に書き戻し、この検査から
    例外扱いしていたが、それ自体が実機起動を壊す原因だった——
    `ParameterDescriptor` で型を明示しても override が空配列である限り
    解決できないため、「例外的に許容してよい空配列」は存在しない。
    このテストは今後もこの種の例外が増えないことを保証する
    （test_blind_angle_ranges_end_to_end_via_export が blind_angle_ranges
    個別の回帰を守る）。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=1, sim=True, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML, env=_subprocess_env())

        yaml_paths = [p for p in os.listdir(out_dir) if p.endswith(".yaml")]
        assert yaml_paths, "生成された *.yaml が1つも無い"

        offenders = []
        for fname in yaml_paths:
            with open(os.path.join(out_dir, fname), encoding="utf-8") as f:
                doc = yaml.safe_load(f) or {}
            for node_name, node_body in doc.items():
                ros_params = (node_body or {}).get("ros__parameters") or {}
                for key, value in ros_params.items():
                    if value is None:
                        offenders.append(f"{fname}:{node_name}.{key} は null")
                    elif isinstance(value, (list, dict)) and len(value) == 0:
                        offenders.append(f"{fname}:{node_name}.{key} は空の{type(value).__name__}")

        assert not offenders, "null / 空コンテナが残っている: " + "; ".join(offenders)


def test_both_launch_files_wire_generated_params():
    """D5: G-4 の実質検査。両 launch ファイルが `GENERATED_DIR` を import し、
    `os.path.join(GENERATED_DIR, '<name>.yaml')` を twist_mux / safety_monitor /
    lidar_filter / esp32_bridge の最低4ファイル分、`parameters=` の配線として
    持っていること。ここが落ちる場合はテストではなく launch 側の配線が壊れている
    ——閾値を下げて合わせるのではなく launch を直すこと。"""
    for path in (BRINGUP_PY, GAZEBO_PY):
        _assert_generated_dir_wired(path)


def test_generated_yaml_has_no_float_ms():
    """実物の registry.yaml から生成した全 YAML に `*_ms` の float が
    1 つも無いこと（実機 bringup が読む経路そのものを検査する）。

    **この経路は Gazebo では通らない。**`gazebo.launch.py` の `safety_monitor` は
    静的な `safety_monitor_sim.yaml` を読むため、生成 YAML の型崩れは sim では
    絶対に露見しない。実機で `safety_monitor` が abort して初めて分かった
    （2026-08-31）。ユニットテストで塞ぐ意味はここにある。"""
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        pg.run_generation(stage=1, sim=False, nodes=list(pg.REGISTRY_NODES),
                           out_dir=out_dir, registry_path=REGISTRY_YAML, env=_subprocess_env())

        offenders = []
        for fname in [p for p in os.listdir(out_dir) if p.endswith(".yaml")]:
            with open(os.path.join(out_dir, fname), encoding="utf-8") as f:
                doc = yaml.safe_load(f) or {}
            for node_name, node_body in doc.items():
                for key, value in ((node_body or {}).get("ros__parameters") or {}).items():
                    if key.endswith("_ms") and isinstance(value, float):
                        offenders.append(f"{fname}:{node_name}.{key} = {value!r}")

        assert not offenders, (
            "*_ms が float のまま生成されている（int 宣言のノードが abort する）: "
            + "; ".join(offenders))
