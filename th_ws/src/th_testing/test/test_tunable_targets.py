"""
test_tunable_targets.py
=======================
WS-9W: WebUI 設定パネルの調整対象（th_config_manager/tunable_targets.py）が、
参照先の YAML ブロックに実在するパラメータだけを挙げていることを固定する。
名前を打ち間違えると config_manager の get/set_parameters が空振りし、
「保存」で YAML に存在しないキーが書かれる。

SettingsPanel.jsx の各 FIELDS 配列とも名前が一致していること（UI と
tunable_targets のドリフト防止）を ast で確認する。

ROS2 なし・純粋 Python。
"""
import ast
import os
import re
import sys

import pytest
import yaml

_CONF_PKG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'th_config_manager', 'th_config_manager'))
sys.path.insert(0, _CONF_PKG)

from tunable_targets import TUNABLE_TARGETS   # noqa: E402

_SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# WS-9X: 設定パネルは S-01 のサブ画面 S-50 になった（SettingsPanel.jsx は削除）。
# フィールド定義は screens/S50Settings.jsx へそのまま移設（配列名は不変）。
_SETTINGS_PANEL = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'web_ui', 'src', 'screens', 'S50Settings.jsx'))


def _yaml_block(pkg: str, relpath: str, block_key: str) -> dict:
    path = os.path.join(_SRC_ROOT, pkg, relpath)
    assert os.path.exists(path), f'{path} が無い'
    doc = yaml.safe_load(open(path, encoding='utf-8'))
    assert block_key in doc, f'{path} に {block_key}: ブロックが無い'
    return doc[block_key]['ros__parameters']


# lidar_filter.blind_angle_ranges は例外: 出所は registry.yaml で、
# perception_params.yaml には最初「保存」するまでキーが無い（config_manager の
# update_ros_params_yaml が書き足す）。YAML 実在チェックの対象外にする。
_YAML_KEY_EXEMPT = {'lidar_filter'}


@pytest.mark.parametrize('name', sorted(set(TUNABLE_TARGETS) - _YAML_KEY_EXEMPT))
def test_every_tunable_param_exists_in_its_yaml(name):
    """TUNABLE_TARGETS[name].params が参照先 YAML に実在すること。"""
    t = TUNABLE_TARGETS[name]
    params = _yaml_block(t['yaml_package'], t['yaml_relpath'], t['block_key'])
    missing = [p for p in t['params'] if p not in params]
    assert not missing, (
        f'{name}: {missing} が {t["yaml_relpath"]} の {t["block_key"]} ブロックに無い')
    assert t['params'], f'{name}: params が空'


def test_all_tunable_targets_have_required_keys():
    """全エントリが yaml_package / yaml_relpath / block_key / params を持つこと。"""
    for name, t in TUNABLE_TARGETS.items():
        for k in ('yaml_package', 'yaml_relpath', 'block_key', 'params'):
            assert k in t, f'{name} に {k} が無い'
        assert isinstance(t['params'], list) and t['params'], f'{name}: params が空/非リスト'


def test_slam_toolbox_is_a_tunable_target():
    """WS-9W: slam_toolbox が調整対象に入っていること（回帰防止）。"""
    assert 'slam_toolbox' in TUNABLE_TARGETS
    t = TUNABLE_TARGETS['slam_toolbox']
    assert t['block_key'] == 'slam_toolbox'
    assert 'config/slam_params.yaml' in t['yaml_relpath']
    for p in ('minimum_travel_distance', 'correlation_search_space_dimension'):
        assert p in t['params'], f'{p} が slam_toolbox の調整対象に無い'


def _settings_panel_field_names(array_name: str) -> set:
    """SettingsPanel.jsx の `const <array_name> = [ {name:'...'}... ]` から name を拾う。"""
    # web_ui/ は Docker コンテナにマウントされない（CLAUDE.md）。その環境では
    # UI との一致検査はスキップし、ホスト実行（web_ui あり）に任せる。
    if not os.path.exists(_SETTINGS_PANEL):
        pytest.skip(f'{_SETTINGS_PANEL} が無い（web_ui 未マウント環境）')
    src = open(_SETTINGS_PANEL, encoding='utf-8').read()
    m = re.search(rf'const {array_name}\s*=\s*\[(.*?)\n\]', src, re.S)
    assert m, f'SettingsPanel.jsx に {array_name} が無い'
    return set(re.findall(r"name:\s*'([^']+)'", m.group(1)))


def test_settings_panel_slam_fields_match_tunable_targets():
    """SettingsPanel.jsx の SLAM_FIELDS と tunable_targets の slam_toolbox.params が一致。"""
    ui = _settings_panel_field_names('SLAM_FIELDS')
    backend = set(TUNABLE_TARGETS['slam_toolbox']['params'])
    assert ui == backend, (
        f'SLAM_FIELDS と tunable_targets がずれている: UIのみ={ui - backend} / backendのみ={backend - ui}')


def test_settings_panel_mapless_fields_match_tunable_targets():
    """既存の MAPLESS_FIELDS も同様に一致していること（ドリフト検知の一般化）。"""
    ui = _settings_panel_field_names('MAPLESS_FIELDS')
    backend = set(TUNABLE_TARGETS['follow_planner_mapless']['params'])
    assert ui == backend, (
        f'MAPLESS_FIELDS と tunable_targets がずれている: UIのみ={ui - backend} / backendのみ={backend - ui}')
