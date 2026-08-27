#!/usr/bin/env python3
"""params_generation.py — launch の OpaqueFunction から呼ぶパラメータ生成ヘルパー（WP-PARAM-02）。

DetailedDesign-params.md §5「①生成」の実体。`registry.yaml` から `th_params.export`
（WP-PARAM-01）を CLI 経由で呼び、ノード別 ROS2 パラメータ YAML と `twist_mux.yaml` を
`/root/th_data/generated/` へ書く。**生成そのもの（値の計算・アサーション判定）は
export.py / assertions.py の中身であり、ここでは作らない（呼ぶだけ）。**

不変条件:
  G-1 生成はノード起動より前に同期実行する
      → `make_opaque_function()` が返す callback は `OpaqueFunction(function=...)` に渡され、
        launch がこの関数を「その後の全アクションを展開する前」に同期的に呼ぶ。
        bringup.launch.py / gazebo.launch.py の両方で、Node(...) を1つも追加する前に
        この OpaqueFunction を最初のアクションとして登録すること。
  G-2 アサーション違反（export.py の終了コード != 0）なら例外を投げて launch を止める。
      「起動はしたが値が危険」という状態を作らない。
  G-3 twist_mux.yaml も生成対象。ただしノード実体（ros-teleop/twist_mux）が読む
      パラメータ名は `locks.<name>.timeout` / `topics.<name>.timeout` のような階層名であり、
      export.py の汎用生成（consumers ごとに `{node: {ros__parameters: {flat_name: value}}}`
      を書く）はそのままでは twist_mux に読ませられない（flat name と階層名が食い違う）。
      ロックのトピック名・優先度・ロック自体のタイムアウトは registry.yaml に持たない
      配線情報（DetailedDesign-reuse.md §2.3「値は維持」）なのでこのモジュールの定数として
      持ち、registry 由来の3つのタイムアウト値だけを export.py の生成結果から差し込んで
      組み直す（`reshape_twist_mux()`）。数値の実体は registry.yaml のまま
      （二重管理にならない）。

不変条件 P-1 に準じ、`reshape_twist_mux()` はファイル I/O をしない純粋関数にする
（`run_generation()` がファイル I/O を引き受ける）。

このモジュールは `launch` / `launch_ros` を**モジュール読み込み時にはインポートしない**
（`make_opaque_function()` の中で遅延インポートする）。これにより
`reshape_twist_mux()` / `run_generation()` は ROS2 launch 環境が無くても
（= `python3 -m pytest` から直接）テストできる。
"""
from __future__ import annotations

import copy
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import yaml

GENERATED_DIR = "/root/th_data/generated"

# このパケット (WP-PARAM-02) の時点で実際に起動しており、かつ registry.yaml の
# consumers 語彙と名前が一致するノードだけを挙げる。
#
# th_planning (follow_planner 等) / th_config_manager 等の旧アーキテクチャのノードは
# registry.yaml の consumers が新アーキテクチャの名前（follow_runner 等）を使っており
# 1:1 で対応しない（旧ノードのコード自体を書き換える別 WP の範囲）。ここに含めない
# ノードの consumers-only な blocking placeholder 行は A8 の対象外になる
# （assertions.a8_blocking_placeholders の `nodes` 引数による絞り込み。「使わない
# ノードのパラメータで起動が止まることを防ぐ」という設計そのもの）。判断は
# WP-PARAM-02 完了報告に明記。
REGISTRY_NODES: tuple[str, ...] = (
    "params_audit",
    "esp32_bridge",
    "lidar_filter",
    "safety_monitor",
    "twist_mux",
    "state_manager",
    "connectivity_checker",
)

# ---------------------------------------------------------------------------
# twist_mux.yaml の組み直し（G-3）
# ---------------------------------------------------------------------------
#
# ロック・トピックの配線（トピック名・優先度・ロック自体のタイムアウト）は
# registry.yaml に無い（th_safety/config/twist_mux.yaml の現行値をそのまま維持。
# DetailedDesign-reuse.md §2.3「値は維持」）。registry から生成するのは
# manual_joy / retreat(behavior) / nav の3つのタイムアウトだけ（G-3 の対象）。
TWIST_MUX_STRUCTURE: dict[str, Any] = {
    "locks": {
        "estop": {"topic": "/safety/estop", "timeout": 0.5, "priority": 255},
        "fault": {"topic": "/safety/fault_lock", "timeout": 0.5, "priority": 254},
    },
    # timeout はここでは「registry が placeholder のときのフォールバック既定値」
    # （現行 th_safety/config/twist_mux.yaml の値と同じ）。通常は resolved（給値済み）
    # なので reshape_twist_mux() が registry 由来の値で必ず上書きする。
    "topics": {
        "retreat": {"topic": "/cmd_vel_retreat", "priority": 20, "timeout": 0.5},
        "nav": {"topic": "/cmd_vel_nav", "priority": 10, "timeout": 0.5},
        "manual_joy": {"topic": "/cmd_vel_manual", "priority": 30, "timeout": 1.0},
    },
}

# registry のフラットなパラメータ名 → twist_mux の階層パスの対応
# (トップレベルの2キー分だけ = topics.<key>.timeout)
_TWIST_MUX_TIMEOUT_MAP: dict[str, str] = {
    "manual_joy_timeout": "manual_joy",
    "behavior_cmd_timeout_s": "retreat",
    "nav_cmd_timeout_s": "nav",
}


def sanitize_node_params(params: Mapping[str, Any]) -> dict[str, Any]:
    """export.py が書いたノード別 `ros__parameters` から、ROS2 が受け取れない値の
    キーを落とす（純粋関数。ファイル I/O をしない）。D3 対応。

    落とす理由（rcl の YAML パーサの挙動）:
      - `None`（export.build_node_outputs() が placeholder を表すのに書く `null`）。
        rcl の YAML パーサはスカラを int → double → bool → string の順に解釈するため、
        `null` は**文字列 `"null"`** になる。宣言側の型（例: DOUBLE_ARRAY）と食い違うと
        `declare_parameter` が `InvalidParameterValueException` を投げてノードが落ちる。
      - 空の list / dict（registry.yaml の `status: given, value: []` 等。
        `enabled_targets` / `required_nodes` が該当）。要素が無いと YAML パーサが型を
        決められず `No parameter value set` になる。**これはそのパラメータを宣言して
        いないノードでも起きる**（ノード生成時にオーバーライド全体を変換するため、
        当該パラメータを使わないノードの起動まで巻き込んで落ちる）。

    キーを落とすと、そのキーは生成物から消え、**静的パラメータファイル（土台）の値が
    そのまま残る**——これは G-4 の重ね書き設計（generated/ を土台の後段に重ねる）の
    意図どおりで、「registry が値を持たないなら土台の値を使う」という以上でも以下でも
    ない。未測定のまま起動してよいかどうかの判定は A8（run_assertions の
    blocking placeholder 検査）が別途行う——ここでは「壊れた YAML でノードを
    落とさない」だけを保証する。

    それ以外の値（0 / False / 空文字列を含む）はそのまま残す
    （falsy であることと「値が無い」ことは別。空文字列はまさに空リストと違って
    STRING 型として妥当に配れる）。
    """
    sanitized: dict[str, Any] = {}
    for key, value in params.items():
        if value is None:
            continue
        if isinstance(value, (list, dict)) and len(value) == 0:
            continue
        sanitized[key] = value
    return sanitized


def _sanitize_node_params_file(path: Path) -> None:
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    changed = False
    for node_name, node_body in doc.items():
        ros_params = (node_body or {}).get("ros__parameters")
        if ros_params is None:
            continue
        clean = sanitize_node_params(ros_params)
        if clean != ros_params:
            doc[node_name]["ros__parameters"] = clean
            changed = True
    if changed:
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)


def reshape_twist_mux(flat_params: Mapping[str, Any]) -> dict[str, Any]:
    """export.py が書いた twist_mux.yaml の flat ros__parameters を、
    twist_mux ノードが実際に読む階層構造へ組み直す（純粋関数。ファイル I/O をしない）。

    `flat_params` は `{registry_param_name: value_or_None}`（export.build_node_outputs の
    出力そのもの）。値が None（= placeholder）の場合は組み直し先を上書きしない
    （静的な既定値のまま残る。placeholder のタイムアウトで twist_mux を起動させない
    判断は A8 = launch 側の仕事であり、ここでは「上書きしない」だけに留める）。
    """
    structure = copy.deepcopy(TWIST_MUX_STRUCTURE)
    for flat_name, topic_key in _TWIST_MUX_TIMEOUT_MAP.items():
        value = flat_params.get(flat_name)
        if value is not None:
            structure["topics"][topic_key]["timeout"] = value
    return {"twist_mux": {"ros__parameters": structure}}


def _reshape_twist_mux_file(path: Path) -> None:
    if not path.exists():
        return
    with open(path, encoding="utf-8") as f:
        flat_doc = yaml.safe_load(f) or {}
    flat_params = (flat_doc.get("twist_mux") or {}).get("ros__parameters") or {}
    nested = reshape_twist_mux(flat_params)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(nested, f, allow_unicode=True, sort_keys=True)


# ---------------------------------------------------------------------------
# blind_angle_ranges: 空配列でも生成物にキーを残す
# ---------------------------------------------------------------------------
#
# registry.yaml の blind_angle_ranges は「死角なし」を status: measured / value: []
# で表す（O-4。走行体のみの構成では死角が無いことを確認済み）。しかし
# sanitize_node_params() は D3 の設計どおり空リストをキーごと落とすため、その後段の
# 生成物だけを見ると「死角なしで確定」と「値が抜けて壊れた」を区別できない。
# ---------------------------------------------------------------------------
# 生成本体
# ---------------------------------------------------------------------------


def default_registry_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("th_params"), "config", "registry.yaml")


class GenerationError(RuntimeError):
    """export.py がアサーション違反・スキーマ違反で失敗したときに送出する（G-2）。"""


def run_generation(*, stage: int, sim: bool, nodes: Sequence[str] | None = REGISTRY_NODES,
                    out_dir: str = GENERATED_DIR, registry_path: str | None = None,
                    env: Mapping[str, str] | None = None) -> None:
    """registry.yaml から生成物を作る。

    FMEA①: 古い generated/ が残って使われることを防ぐため、書く前に必ず削除する。
    G-2: export.py の終了コードが 0 でなければ GenerationError を投げる（launch を止める）。
    `env` はサブプロセスへ渡す環境変数（省略時は現プロセスを継承。テストが
    PYTHONPATH を差し替えるために使う。launch から呼ぶときは省略でよい —
    th_params は colcon install 済みで通常の PYTHONPATH に乗っている）。

    成功時（終了コード 0）でも export.py の stderr は握り潰さず、そのまま
    launch 側の stderr へ出す。A6（esp32_timeout_ms が WATCHDOG_MS 以下）のように
    「起動は拒否しないが黙って見過ごしてもいけない」判定があるため
    （eed7ee1 で A6 を拒否→警告に変更した際、警告の出口が launch 経路に
    無いままだった）。
    """
    registry_path = registry_path or default_registry_path()
    out = Path(out_dir)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    cmd = [sys.executable, "-m", "th_params.export",
           "--registry", registry_path, "--out", str(out), "--stage", str(stage)]
    if sim:
        cmd.append("--sim")
    if nodes:
        cmd += ["--nodes", ",".join(nodes)]

    result = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if result.returncode != 0:
        raise GenerationError(
            "params 生成に失敗した（G-2: launch を止める。古い generated/ は使わない）。\n"
            f"cmd={' '.join(cmd)}\n"
            f"exit={result.returncode}\n"
            f"--- stdout ---\n{result.stdout}\n"
            f"--- stderr ---\n{result.stderr}")

    if result.stderr:
        print(f"[params_generation] export.py の警告:\n{result.stderr}", file=sys.stderr)

    # D3: export.py は `--nodes` で絞っても consumers に現れる全ノード分の YAML を書く
    # （REGISTRY_NODES の5つだけではない）。params_digest.json は ROS2 パラメータ
    # ファイルではないので対象外、それ以外の *.yaml 全てをサニタイズする。
    # reshape_twist_mux() は静的な既定値を土台に持つ設計（キーが無ければ上書きしない）
    # なので、サニタイズの後に行う。
    for yaml_path in sorted(out.glob("*.yaml")):
        _sanitize_node_params_file(yaml_path)

    _reshape_twist_mux_file(out / "twist_mux.yaml")
    # blind_angle_ranges は「空なら sanitize_node_params() がキーごと落とす」に委ねる。
    # 以前はここで空配列を書き戻していたが、**ROS 2 Humble は params YAML の空配列を
    # 扱えない**（rclcpp は `parameter_value_from failed ... No parameter value set` で
    # 落ち、rclpy は未初期化のまま `get_parameter().value` で例外になる。標準ノード
    # tf2_ros/static_transform_publisher でも同じ現象を実測で確認した）。
    # 「死角なし」と「未校正」の区別は blind_calibrated（明示フラグ）が運ぶので、
    # 配列の形から推論する必要はない（DetailedDesign-open.md の N-8）。


# ---------------------------------------------------------------------------
# launch 用 OpaqueFunction ラッパ（`launch` はここで初めて import する）
# ---------------------------------------------------------------------------


def make_opaque_function(*, nodes: Sequence[str] = REGISTRY_NODES,
                          stage_arg_name: str = "stage",
                          sim_arg_name: str | None = None,
                          sim_default: bool = False) -> Callable:
    """`OpaqueFunction(function=...)` に渡すコールバックを作る。

    `sim_arg_name` を指定すると、その名前の launch 引数（'true'/'false' 文字列）を
    `--sim` の可否に使う（gazebo.launch.py 向け）。指定しなければ `sim_default`
    を使う（bringup.launch.py は実機専用launchなので常に False）。
    """
    def _opaque(context, *args, **kwargs):
        from launch.substitutions import LaunchConfiguration

        stage = int(LaunchConfiguration(stage_arg_name).perform(context))
        if sim_arg_name is not None:
            sim_str = LaunchConfiguration(sim_arg_name).perform(context).strip().lower()
            sim = sim_str in ("true", "1", "yes")
        else:
            sim = sim_default

        run_generation(stage=stage, sim=sim, nodes=nodes)
        return []

    return _opaque
