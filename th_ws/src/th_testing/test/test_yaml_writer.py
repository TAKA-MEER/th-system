"""
test_yaml_writer.py
====================
th_config_manager.yaml_writer.update_ros_params_yaml の単体テスト。

ROS2 なし・純粋 Python で実行可能。
pytest で実行: pytest test/test_yaml_writer.py -v

WebUI の「YAML に保存」機能が、config/*.yaml に埋め込まれた調整の経緯を
説明するコメントを壊さずに値だけを更新できることを確認する。
"""

import os
import sys

import pytest

# ── パスを通す（colcon build 前にも直接 pytest できるように）
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager', 'th_config_manager'))

ruamel = pytest.importorskip("ruamel.yaml")

from yaml_writer import update_ros_params_yaml  # noqa: E402


PLANNING_PARAMS_YAML = """\
# ============================================================
# planning_params.yaml — テスト用抜粋
# ============================================================
follow_planner_mapless:
  ros__parameters:
    # ── 試験員接近時の停止/再開（ヒステリシス） ────────────
    stop_distance: 1.0               # m これ未満で停止
    resume_distance: 1.3             # m これ以上で追従再開
    v_max: 0.3                       # m/s 人の歩行速度に追従できる上限
    odom_frame: "odom"

panel_navigator:
  ros__parameters:
    arrival_threshold_m: 0.3
    map_frame: "map"
"""

PERCEPTION_PARAMS_YAML = """\
# ============================================================
# perception_params.yaml — テスト用抜粋
# ============================================================
lidar_filter:
  ros__parameters:
    input_topic:  "/scan"
    # 死角範囲 [start_deg, end_deg, ...] (フラットリスト、ペアで指定)
    blind_angle_ranges:
      - 40.0    # 右前 開始
      - 50.0    # 右前 終了
      - 130.0   # 右後 開始
      - 140.0   # 右後 終了
"""


def test_scalar_update_preserves_comments_and_other_keys(tmp_path):
    path = tmp_path / "planning_params.yaml"
    path.write_text(PLANNING_PARAMS_YAML, encoding="utf-8")

    update_ros_params_yaml(path, "follow_planner_mapless", {"v_max": 0.8, "stop_distance": 1.2})

    out = path.read_text(encoding="utf-8")
    assert "v_max: 0.8" in out
    assert "stop_distance: 1.2" in out
    # コメントが保持されていること
    assert "# m これ以上で追従再開" in out
    assert "ヒステリシス" in out
    # 変更していないキー・他ブロックはそのまま
    assert "resume_distance: 1.3" in out
    assert 'odom_frame: "odom"' in out
    assert "panel_navigator:" in out
    assert "arrival_threshold_m: 0.3" in out


def test_list_update_preserves_per_item_comments(tmp_path):
    path = tmp_path / "perception_params.yaml"
    path.write_text(PERCEPTION_PARAMS_YAML, encoding="utf-8")

    update_ros_params_yaml(
        path, "lidar_filter",
        {"blind_angle_ranges": [41.0, 51.0, 131.0, 141.0]})

    out = path.read_text(encoding="utf-8")
    assert "- 41.0" in out
    assert "- 51.0" in out
    assert "- 131.0" in out
    assert "- 141.0" in out
    # 各要素のインラインコメントが保持されていること
    assert "# 右前 開始" in out
    assert "# 右前 終了" in out
    assert "# 右後 開始" in out
    assert "# 右後 終了" in out
    # 更新していないキーはそのまま（列揃え目的の余分な空白は round-trip で
    # 単一スペースに正規化されるため、値の有無のみ確認する）
    assert 'input_topic:' in out and '"/scan"' in out


def test_update_is_idempotent_when_value_unchanged(tmp_path):
    path = tmp_path / "planning_params.yaml"
    path.write_text(PLANNING_PARAMS_YAML, encoding="utf-8")

    before = path.read_text(encoding="utf-8")
    update_ros_params_yaml(path, "follow_planner_mapless", {"v_max": 0.3})
    after = path.read_text(encoding="utf-8")

    assert before == after
