"""
test_map_freezer_core.py — 地図更新 OFF の間 /map を凍結する判定と状態機械
（th_config_manager/map_freezer_core.py。ROS 非依存・ホストで走る。Spec-onsite §4.0.2）。

  a. decide: 全組合せ（loaded × 写し有無 × モード × map_update）
  b. 写しを取る契機（opening/opened/saving/saved/…）と /map に出すものの選択
  c. 配線の静的確認: bringup が slam_toolbox を /map_live へ remap し map_freezer を同じ
     条件で起動する／slam_control が契機を正しい順で出す

変異: decide の `map_update` を見る行を消す→ a が赤 / opened で staged を写しにする行を
消す→ b が赤 / opening で last_live を捨てる行を消す→ b の「開く前の地図を掴まない」が赤。
"""
import ast
import itertools
import os
import sys

import pytest

_PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..',
                                    'th_config_manager'))
sys.path.insert(0, _PKG)

from th_config_manager.map_freezer_core import (  # noqa: E402
    FREEZE, PASS, PASS_NO_COPY, MapFreezer, decide, effective_mode,
    make_anchor, parse_anchor)

MODES = ['INIT', 'IDLE', 'ESTOP', 'CARRY', 'FOLLOW', 'MANUAL', 'TEACH_FOLLOW',
         'TEACH_MANUAL', 'REPLAY', 'LINE', 'LEASH', 'PREP', 'PANEL_NAV',
         'AT_PANEL', 'SUMMON', 'HOME_NAV', 'AT_HOME', 'OPCHECK', 'CALIB']
PASS_MODES = {'PREP', 'TEACH_FOLLOW', 'TEACH_MANUAL'}


# ── a. decide ──
@pytest.mark.parametrize('loaded,has_copy,mode,update',
                         itertools.product([False, True], [False, True], MODES,
                                           [False, True]))
def test_a_decide_all_combinations(loaded, has_copy, mode, update):
    got = decide(loaded, has_copy, mode, '', update)
    if not loaded or update or mode in PASS_MODES:
        assert got == PASS
    elif has_copy:
        assert got == FREEZE
    else:
        assert got == PASS_NO_COPY


@pytest.mark.parametrize('mode', ['ESTOP', 'CARRY'])
def test_a_latched_modes_use_prev_mode(mode):
    assert effective_mode(mode, 'PREP') == 'PREP'
    assert decide(True, True, mode, 'PREP', False) == PASS
    assert decide(True, True, mode, 'PANEL_NAV', False) == FREEZE
    # prev_mode が空なら ESTOP 自身で判定する（凍結側）
    assert decide(True, True, mode, '', False) == FREEZE


# ── b. 写しを取る契機 ──
class Seq:
    def __init__(self):
        self.f = MapFreezer()
        self.n = 0
        self.out = []

    def anchor(self, event, boot='b1'):
        self.n += 1
        assert self.f.on_anchor(boot, self.n, event)
        self._poll()

    def live(self, name):
        self.f.on_live(name)
        self._poll()

    def state(self, mode='REPLAY', update=False, prev=''):
        self.f.on_state(mode, prev, update)
        self._poll()

    def _poll(self):
        m = self.f.poll()
        if m is not None:
            self.out.append(m)


def _opened_seq():
    s = Seq()
    s.state('REPLAY', False)
    s.anchor('opening')
    s.live('opened-map')
    s.anchor('opened')
    return s


def test_b_pass_before_any_map_is_opened():
    s = Seq()
    s.state('IDLE', False)
    s.live('m1')
    s.live('m2')
    assert s.out == ['m1', 'm2']


def test_b_frozen_after_open_ignores_live_changes():
    s = _opened_seq()
    assert s.out[-1] == 'opened-map'
    n = len(s.out)
    s.live('with-temp-scan-1')
    s.live('with-temp-scan-2')
    assert len(s.out) == n and s.out[-1] == 'opened-map'


def test_b_map_before_opening_is_not_captured():
    s = Seq()
    s.state('REPLAY', False)
    s.live('prep-map')          # 開く前の地図
    s.anchor('opening')
    s.live('opened-map')
    s.anchor('opened')
    s.live('temp-scan')
    assert s.f.copy == 'opened-map'
    assert s.out[-1] == 'opened-map'


def test_b_capture_next_live_when_none_arrived_between_opening_and_opened():
    s = Seq()
    s.state('REPLAY', False)
    s.live('prep-map')
    s.anchor('opening')
    s.anchor('opened')
    assert s.f.copy is None
    assert s.f.decision() == PASS_NO_COPY
    assert s.out == ['prep-map']            # 開く前の地図を新たに出さない
    s.live('first-after-open')
    assert s.f.copy == 'first-after-open'
    assert s.out[-1] == 'first-after-open'
    s.live('later')
    assert s.out[-1] == 'first-after-open'


def test_b_save_right_after_open_does_not_take_the_pre_open_map():
    s = Seq()
    s.state('REPLAY', True)
    s.live('prep-map')
    s.anchor('opening')
    s.anchor('opened')
    s.anchor('saving')
    s.anchor('saved')
    assert s.f.copy is None          # 開く前の地図を写しにしない（次の地図を待つ）
    s.live('first-after-open')
    assert s.f.copy == 'first-after-open'


def test_b_update_on_passes_then_off_returns_to_copy_without_save():
    s = _opened_seq()
    s.state('REPLAY', True)
    s.live('written-1')
    assert s.out[-1] == 'written-1'
    s.live('written-2')
    assert s.out[-1] == 'written-2'
    s.state('REPLAY', False)            # 保存せず OFF
    assert s.out[-1] == 'opened-map'
    s.live('temp')
    assert s.out[-1] == 'opened-map'


def test_b_update_on_save_then_off_freezes_saved_map():
    s = _opened_seq()
    s.state('REPLAY', True)
    s.live('written-1')
    s.anchor('saving')
    s.live('written-2-after-saving-started')
    s.anchor('saved')
    assert s.out[-1] == 'written-2-after-saving-started'   # ON の間は通す
    s.live('written-3-unsaved')
    s.state('REPLAY', False)
    assert s.out[-1] == 'written-1'      # saving の時点の最新が保存した地図
    s.live('temp')
    assert s.out[-1] == 'written-1'


def test_b_save_failed_keeps_old_copy():
    s = _opened_seq()
    s.state('REPLAY', True)
    s.live('written-1')
    s.anchor('saving')
    s.anchor('save_failed')
    s.state('REPLAY', False)
    assert s.out[-1] == 'opened-map'


def test_b_reopen_takes_new_copy():
    s = _opened_seq()
    s.anchor('opening')
    s.live('map-2')
    s.anchor('opened')
    s.live('temp')
    assert s.f.copy == 'map-2' and s.out[-1] == 'map-2'


def test_b_unloaded_passes_live():
    s = _opened_seq()
    s.anchor('unloaded')
    s.live('new-empty-mapping-map')
    assert s.f.copy is None
    assert s.out[-1] == 'new-empty-mapping-map'


def test_b_prep_and_teach_pass_even_when_loaded():
    for mode in ('PREP', 'TEACH_FOLLOW', 'TEACH_MANUAL'):
        s = _opened_seq()
        s.state(mode, False)
        s.live('raw')
        assert s.out[-1] == 'raw', mode


def test_b_estop_during_replay_stays_frozen():
    s = _opened_seq()
    s.state('ESTOP', False, prev='REPLAY')
    s.live('temp')
    assert s.out[-1] == 'opened-map'


def test_b_respawn_does_not_overwrite_copy_without_anchor():
    """契機が無いまま新しい /map_live が来ても、写しは変えない。"""
    s = _opened_seq()
    s.live('first-map-from-respawned-slam')
    assert s.f.copy == 'opened-map' and s.out[-1] == 'opened-map'


def test_b_redelivered_anchor_is_ignored_but_new_boot_is_accepted():
    f = MapFreezer()
    assert f.on_anchor('b1', 5, 'opened') is True
    assert f.on_anchor('b1', 5, 'unloaded') is False
    assert f.on_anchor('b1', 4, 'unloaded') is False
    assert f.loaded is True
    assert f.on_anchor('b2', 1, 'unloaded') is True     # slam_control の再起動
    assert f.loaded is False


def test_b_poll_publishes_only_on_change():
    s = _opened_seq()
    assert s.f.poll() is None
    assert s.f.current_frozen() == 'opened-map'
    s.state('REPLAY', True)
    assert s.f.current_frozen() is None


def test_anchor_roundtrip_and_garbage():
    assert parse_anchor(make_anchor('x', 3, 'saved')) == ('x', 3, 'saved')
    for bad in ('', 'nope', '{}', '{"boot":"a","seq":"z","event":"opened"}',
                '{"boot":"a","seq":1,"event":"bogus"}', None):
        assert parse_anchor(bad) is None


# ── c. 配線の静的確認 ──
_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _read(*p):
    with open(os.path.join(_SRC, *p), encoding='utf-8') as f:
        return f.read()


def test_c_bringup_remaps_slam_and_starts_freezer_with_same_condition():
    src = _read('th_bringup', 'launch', 'bringup.launch.py')
    slam = src.split("executable='map_and_localization_slam_toolbox_node'", 1)[1]
    slam = slam.split('nodes.append', 1)[0]
    assert "remappings=[('/map', '/map_live')]" in slam
    frz = src.split("executable='map_freezer.py'", 1)[1].split('nodes.append', 1)[0]

    def cond(t):
        c = t[t.index('condition='):]
        return c[:c.index('\n    ))')]
    assert cond(frz) == cond(slam)


def _func(tree, name):
    return next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == name)


def _events(func):
    """関数内の _publish_anchor('x') と deserialize/serialize 呼び出しの出現順（行番号順）。"""
    out = []
    for c in (n for n in ast.walk(func) if isinstance(n, ast.Call)):
        f = c.func
        if isinstance(f, ast.Attribute) and f.attr == '_publish_anchor':
            out.append((c.lineno, c.args[0].value))
        elif isinstance(f, ast.Name) and f.id == 'call_and_wait':
            out.append((c.lineno, 'deserialize'))
        elif isinstance(f, ast.Attribute) and f.attr == '_serialize':
            out.append((c.lineno, 'serialize'))
    return [e for _, e in sorted(out)]


def test_c_slam_control_publishes_anchors_in_order():
    tree = ast.parse(_read('th_config_manager', 'scripts', 'slam_control.py'))
    reload_ev = _events(_func(tree, '_handle_map_reload'))
    assert reload_ev.index('opening') < reload_ev.index('deserialize') \
        < reload_ev.index('opened')
    assert 'unloaded' in reload_ev
    for name in ('_handle_map_save', '_commit_route_patch'):
        ev = _events(_func(tree, name))
        assert ev.index('saving') < ev.index('serialize') < ev.index('saved'), name
        assert 'save_failed' in ev, name
    for name in ('_handle_map_reset', '_cb_discard_map', '_check_slam_restart',
                 '_cb_toggle', '_cb_set_mapping'):
        assert 'unloaded' in _events(_func(tree, name)), name
