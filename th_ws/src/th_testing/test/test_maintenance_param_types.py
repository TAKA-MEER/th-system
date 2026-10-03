"""始業点検・校正ノードの宣言型が、生成 yaml の値の型と一致していること。

rclpy（Humble）は `declare_parameter(name, 既定値)` の既定値から型を決め、
params ファイルの INTEGER と DOUBLE を相互に変換しない。宣言が `300.0` で
生成 yaml が `300` だと、起動時に `InvalidParameterTypeException` で落ちる。

2026-10-02 の実機で `opcheck_runner` がこれ（`scan_stale_ms`）で起動に失敗し、
始業点検の MOTOR 項目が `/cmd_vel_behavior` を一切出さなかった。launch の
ログに `process has died` が 1 行出るだけで、画面からは気づけない。
ノード試験（`test_opcheck_runner_node.py`）は生成 yaml を読まず dict で
上書きしていたため、本番の経路を通らずに緑のままだった。

ここでは実際の registry.yaml から生成した yaml と、各ノードの `PARS`
（`declare_parameter` に渡す既定値）の型を突き合わせる。ROS 不要。
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
_MAINTENANCE_PKG = os.path.join(_REPO_SRC, "th_maintenance")
_SCRIPTS = os.path.join(_MAINTENANCE_PKG, "scripts")

for _p in (_LAUNCH_DIR, _PARAMS_SRC, _MAINTENANCE_PKG):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402
from th_maintenance import blind_core  # noqa: E402

REGISTRY_YAML = os.path.join(_PARAMS_SRC, "config", "registry.yaml")

# (スクリプト, launch が渡す生成 yaml のノード名)。bringup.launch.py の
# parameters= と同じ対応にする（opcheck_auto は opcheck_runner.yaml を読む）。
_CASES = [
    ("opcheck_runner.py", "opcheck_runner"),
    ("opcheck_auto.py", "opcheck_runner"),
    ("calib_runner.py", "calib_runner"),
]


def _load_pars(script: str) -> dict:
    """スクリプトの `PARS = {...}` だけを取り出して評価する。

    スクリプト全体は rclpy の型注釈を持つためホストでは import できない。
    値に `blind_core.MAX_*` を使うもの（calib_runner）があるので、その名前だけ渡す。
    """
    path = os.path.join(_SCRIPTS, script)
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=path)
    for node in tree.body:
        targets = (node.targets if isinstance(node, ast.Assign)
                   else [node.target] if isinstance(node, ast.AnnAssign) else [])
        if any(isinstance(t, ast.Name) and t.id == "PARS" for t in targets):
            expr = ast.Expression(node.value)
            return dict(eval(compile(expr, path, "eval"),  # noqa: S307（自リポジトリのリテラル）
                             {"blind_core": blind_core}))
    raise AssertionError(f"{script} に PARS が無い")


def _ros_type(value) -> str:
    # bool は int の派生なので先に見る
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        return "list"
    return type(value).__name__


@pytest.fixture(scope="module")
def generated():
    with tempfile.TemporaryDirectory() as tmp:
        out_dir = os.path.join(tmp, "generated")
        env = dict(os.environ)
        env["PYTHONPATH"] = _PARAMS_SRC + os.pathsep + env.get("PYTHONPATH", "")
        pg.run_generation(stage=4, sim=False, nodes=list(pg.REGISTRY_NODES),
                          out_dir=out_dir, registry_path=REGISTRY_YAML, env=env,
                          calib_dir=os.path.join(tmp, "no_calib"))
        docs = {}
        for _, node in _CASES:
            with open(os.path.join(out_dir, f"{node}.yaml"), encoding="utf-8") as f:
                docs[node] = yaml.safe_load(f)[node]["ros__parameters"]
        yield docs


@pytest.mark.parametrize("script,node", _CASES)
def test_declared_types_match_generated_yaml(generated, script, node):
    pars = _load_pars(script)
    params = generated[node]
    shared = sorted(set(pars) & set(params))
    assert shared, f"{script} と {node}.yaml に共通のパラメータが無い（突き合わせが空振り）"
    mismatched = [
        f"{name}: 宣言 {pars[name]!r}（{_ros_type(pars[name])}）"
        f" / 生成 yaml {params[name]!r}（{_ros_type(params[name])}）"
        for name in shared
        if _ros_type(pars[name]) != _ros_type(params[name])
    ]
    assert not mismatched, (
        f"{script} の宣言型が生成 yaml と違う（rclpy は INTEGER↔DOUBLE を変換せず"
        f"起動時に落ちる）: " + "; ".join(mismatched))


def test_opcheck_runner_scan_stale_ms_is_compared(generated):
    """突き合わせが今回の原因の行を実際に含んでいること（空振り防止）。"""
    assert "scan_stale_ms" in generated["opcheck_runner"]
    assert "scan_stale_ms" in _load_pars("opcheck_runner.py")
