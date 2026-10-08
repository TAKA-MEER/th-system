"""test_arrival_tol_registry.py — 到着許容差の一本化（SG-B15・1b-11①）。

registry の `arrival_xy_tol_m`（0.12）が venue_navigator の到着判定と Nav2
（実機用）の `xy_goal_tolerance`／`yaw_goal_tolerance` の両方の根拠であること。
Nav2 側へは params_generation が生成 nav2_params.yaml に写す
（Nav2 の params_file は registry 駆動にできないため）。

ROS2 不要（host で走る）。
"""
import copy
import os
import sys

import pytest
import yaml

from th_params import export

_LAUNCH_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_bringup', 'launch'))
sys.path.insert(0, _LAUNCH_DIR)
from params_generation import patch_nav2_goal_tolerance, run_generation

_SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_STATIC_NAV2 = os.path.join(_SRC_ROOT, 'th_bringup', 'config', 'nav2_params.yaml')


def _row(rows, name):
    for row in rows:
        if row.get('name') == name:
            return row
    pytest.fail(f'registry に {name} が無い')


def test_arrival_tol_is_single_012(registry_rows):
    """registry の到着許容差は 0.12（spec §6.2 の 1 つの値）。"""
    row = _row(registry_rows, 'arrival_xy_tol_m')
    assert row['value'] == pytest.approx(0.12)
    assert 'venue_navigator' in (row.get('consumers') or [])


def test_export_emits_arrival_to_venue(registry_rows):
    """export が venue_navigator.yaml に同じ値を載せること（本番の経路の前半）。"""
    resolved = export.resolve_registry(registry_rows)
    outputs = export.build_node_outputs(registry_rows, resolved)
    assert outputs['venue_navigator']['arrival_xy_tol_m'] == pytest.approx(0.12)


def test_patch_nav2_sets_both_tolerances():
    """Nav2 への写しは xy と yaw の両方に同じ値を入れる。節が無ければ無変更。"""
    with open(_STATIC_NAV2, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    patched = patch_nav2_goal_tolerance(doc, 0.12)
    checker = patched['controller_server']['ros__parameters']['general_goal_checker']
    assert checker['xy_goal_tolerance'] == pytest.approx(0.12)
    assert checker['yaw_goal_tolerance'] == pytest.approx(0.12)
    # 入力は変えない（複写元の静的ファイルを汚さない）
    orig = doc['controller_server']['ros__parameters']['general_goal_checker']
    assert orig['xy_goal_tolerance'] == pytest.approx(0.12)
    assert patch_nav2_goal_tolerance({}, 0.12) == {}


def test_generation_propagates_registry_change(tmp_path, registry_rows):
    """registry の値を変えると venue 生成物と nav2 生成物の両方が変わる。

    run_generation の実物（export サブプロセス＋写し）を tmp に回す。
    """
    rows = copy.deepcopy(registry_rows)
    _row(rows, 'arrival_xy_tol_m')['value'] = 0.09
    reg_path = tmp_path / 'registry.yaml'
    with open(reg_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(rows, f, allow_unicode=True)
    out_dir = tmp_path / 'generated'
    env = dict(os.environ)
    th_params_dir = os.path.join(_SRC_ROOT, 'th_params')
    env['PYTHONPATH'] = th_params_dir + (os.pathsep + env['PYTHONPATH']
                                         if env.get('PYTHONPATH') else '')
    run_generation(stage=4, sim=False, nodes=['venue_navigator'],
                   out_dir=str(out_dir), registry_path=str(reg_path),
                   env=env, calib_dir=str(tmp_path / 'calib'),
                   overrides_path=None, nav2_static_path=_STATIC_NAV2)
    with open(out_dir / 'venue_navigator.yaml', encoding='utf-8') as f:
        venue = yaml.safe_load(f)
    assert venue['venue_navigator']['ros__parameters']['arrival_xy_tol_m'] == \
        pytest.approx(0.09)
    with open(out_dir / 'nav2_params.yaml', encoding='utf-8') as f:
        nav2 = yaml.safe_load(f)
    checker = nav2['controller_server']['ros__parameters']['general_goal_checker']
    assert checker['xy_goal_tolerance'] == pytest.approx(0.09)
    assert checker['yaw_goal_tolerance'] == pytest.approx(0.09)


def test_launch_files_read_generated_nav2_params():
    """実機用 Nav2 の params は生成物を読む（静的 config を直接読むと registry の
    値が Nav2 に届かない）。launch の配線の 3 か所を縛る。"""
    import re
    launch_dir = os.path.join(_SRC_ROOT, 'th_bringup', 'launch')
    expect = {
        'bringup.launch.py': r"nav2_yaml\s*=\s*os\.path\.join\(GENERATED_DIR,\s*'nav2_params\.yaml'\)",
        'gazebo.launch.py': r"nav2_params_real\s*=\s*os\.path\.join\(GENERATED_DIR,\s*'nav2_params\.yaml'\)",
    }
    for fname, pat in expect.items():
        with open(os.path.join(launch_dir, fname), encoding='utf-8') as f:
            src = f.read()
        assert re.search(pat, src), f'{fname} が静的な nav2_params.yaml を読んでいる'
