#!/usr/bin/env python3
"""
test_scenario_configs.py
=========================
シナリオプリセット (th_bringup/config/scenarios/*.yaml) の整合性検証。
ROS2 不要の純粋 pytest。ワールド・地図の参照切れや配列長の崩れなど、
ワールド編集とプリセットの乖離を早期に検出する。

実行:
  python3 -m pytest src/th_testing/test/test_scenario_configs.py -v
"""
import math
import warnings
from pathlib import Path

import pytest
import yaml

SRC_DIR      = Path(__file__).resolve().parents[2]
BRINGUP_DIR  = SRC_DIR / 'th_bringup'
SCENARIO_DIR = BRINGUP_DIR / 'config' / 'scenarios'
WORLDS_DIR   = BRINGUP_DIR / 'worlds'
MAPS_DIR     = BRINGUP_DIR / 'maps'

VALID_PATTERNS = {'waypoints', 'patrol', 'approach', 'static'}
VALID_MODES    = {1, 2, 3, 4, 5, 6, 7}

scenario_files = sorted(SCENARIO_DIR.glob('*.yaml'))


def _load(path: Path) -> dict:
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict) and 'scenario' in data, \
        f'{path.name}: トップレベルに scenario キーが必要'
    scen = data['scenario']
    assert isinstance(scen, dict), f'{path.name}: scenario は dict であること'
    return scen


def test_scenario_dir_has_presets():
    assert scenario_files, f'{SCENARIO_DIR} にプリセットがありません'


@pytest.mark.parametrize('path', scenario_files, ids=lambda p: p.stem)
class TestScenarioConfig:

    def test_world_exists(self, path):
        scen = _load(path)
        world = scen.get('world')
        assert world, f'{path.name}: world は必須'
        assert (WORLDS_DIR / world).exists(), \
            f'{path.name}: ワールド {world} が {WORLDS_DIR} に存在しない'

    def test_map_reference(self, path):
        scen = _load(path)
        slam = scen.get('slam', True)
        map_yaml = scen.get('map_yaml', '')
        if slam is False:
            assert map_yaml, \
                f'{path.name}: slam: false のとき map_yaml は必須'
        if map_yaml:
            map_path = MAPS_DIR / map_yaml
            if not map_path.exists():
                # 地図は SLAM 走行で生成してコミットする運用のため、
                # 未生成はエラーにせず警告に留める (docs/simulation.md 参照)
                warnings.warn(
                    f'{path.name}: 地図 {map_yaml} が未生成です。'
                    f'slam:=true で走行して map_saver_cli で生成してください')

    def test_recommended_mode(self, path):
        scen = _load(path)
        mode = scen.get('recommended_mode')
        if mode is not None:
            assert mode in VALID_MODES, \
                f'{path.name}: recommended_mode {mode} は RobotMode 定数でない'

    def test_robot_spawn(self, path):
        scen = _load(path)
        spawn = scen.get('robot_spawn')
        if spawn is not None:
            assert set(spawn.keys()) <= {'x', 'y', 'yaw'}, \
                f'{path.name}: robot_spawn のキーは x/y/yaw のみ'
            for k, v in spawn.items():
                assert isinstance(v, (int, float)) and math.isfinite(v), \
                    f'{path.name}: robot_spawn.{k} が数値でない'

    def test_person(self, path):
        scen = _load(path)
        person = scen.get('person') or {}
        pattern = person.get('pattern', 'patrol')
        assert pattern in VALID_PATTERNS, \
            f'{path.name}: pattern {pattern} は {VALID_PATTERNS} のいずれかであること'
        wps = person.get('waypoints') or []
        if pattern == 'waypoints':
            assert wps, f'{path.name}: pattern: waypoints のとき waypoints は必須'
        if wps:
            assert len(wps) % 3 == 0, \
                f'{path.name}: waypoints 長 {len(wps)} は 3 の倍数 [x,y,pause,...] であること'
            assert all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in wps), \
                f'{path.name}: waypoints に数値でない要素がある'

    def test_occlusion(self, path):
        scen = _load(path)
        relay = scen.get('relay') or {}
        segs = relay.get('occlusion_segments') or []
        if relay.get('occlusion_check'):
            assert segs, \
                f'{path.name}: occlusion_check: true のとき occlusion_segments は必須'
        if segs:
            assert len(segs) % 4 == 0, \
                f'{path.name}: occlusion_segments 長 {len(segs)} は 4 の倍数 ' \
                f'[x1,y1,x2,y2,...] であること'
            assert all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in segs), \
                f'{path.name}: occlusion_segments に数値でない要素がある'

    def test_planning_overrides(self, path):
        scen = _load(path)
        overrides = scen.get('planning_overrides') or {}
        assert set(overrides.keys()) <= {'follow_planner', 'follow_planner_mapless'}, \
            f'{path.name}: planning_overrides のキーはプランナノード名のみ'
        for node, params in overrides.items():
            assert isinstance(params, dict) and params, \
                f'{path.name}: planning_overrides.{node} は空でない dict であること'
