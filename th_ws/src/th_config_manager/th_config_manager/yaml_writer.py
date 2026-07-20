"""
yaml_writer.py
===============
ROS2 パラメータ YAML (`<node>: {ros__parameters: {...}}}`) の特定ブロックの
特定キーだけを書き換える（ROS2 非依存の純粋 Python、pytest で直接テスト可能）。

ruamel.yaml の round-trip モードを使い、コメント・キー順序・インデント
スタイルを保持したまま値だけを更新する。planning_params.yaml /
perception_params.yaml には調整の経緯を説明する重要なコメントが多数
埋め込まれているため、単純な yaml.safe_dump で上書きすると失われてしまう。

indent(mapping=2, sequence=4, offset=2) は本リポジトリの config/*.yaml が
実際に使っている書式（キー0/2/4段、リスト要素はキーの2文字下げ）に一致する
よう検証済みの設定。既存ファイルと異なる書式の YAML には使わないこと。

既知の制約: 行内コメントの列位置を揃えるための余分な空白（例:
`key:  value` の2連スペース）は ruamel の round-trip でも保持されない
（単一スペースに正規化される）。コメント文自体・キー構造・値は保持される。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Union

from ruamel.yaml import YAML


def update_ros_params_yaml(
    path: Union[str, Path],
    block_key: str,
    updates: Mapping[str, Any],
) -> None:
    """path 内の `<block_key>: {ros__parameters: {...}}` を updates で上書きして保存する。

    リスト値（例: blind_angle_ranges）は要素ごとに代入することで、既存のリスト
    オブジェクトを保持し、要素ごとのインラインコメントを残す。
    """
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=4, offset=2)

    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        doc = yaml.load(f)

    params = doc[block_key]["ros__parameters"]
    for name, value in updates.items():
        current = params.get(name)
        if isinstance(current, list) and isinstance(value, (list, tuple)):
            for i, v in enumerate(value):
                current[i] = v
        else:
            params[name] = value

    with path.open("w", encoding="utf-8") as f:
        yaml.dump(doc, f)
