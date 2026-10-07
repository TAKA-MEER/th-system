"""test_auto_brake.py — 自動ブレーキの有効／無効の純関数（1b-15 SG-B10）。

Spec-safety.md §2.1 の表をそのまま写す。attributes.yaml の実物を読んで、
「どのモードが手動系か」を試験側で決め打ちしない。
"""
import itertools
import os

import pytest
import yaml

from th_state.auto_brake import (AUTO_BRAKE_LOCKED_REASON, effective_auto_brake,
                                  override_allowed, zone_default)

_ATTR = os.path.join(os.path.dirname(__file__), '..', '..', 'th_state', 'config',
                     'attributes.yaml')
with open(_ATTR, encoding='utf-8') as f:
    ATTRS = yaml.safe_load(f)

MANUAL_LIKE = [m for m, a in ATTRS.items() if a['auto_brake_default'] != 'on_locked']
LOCKED = [m for m, a in ATTRS.items() if a['auto_brake_default'] == 'on_locked']


def test_attributes_manual_like_are_manual_and_teach_manual():
    # 手動系の定義が attributes.yaml から導かれていること（手動走行・教示（手動））。
    assert set(MANUAL_LIKE) == {'MANUAL', 'TEACH_MANUAL'}


@pytest.mark.parametrize('mode', MANUAL_LIKE)
def test_manual_out_default_off(mode):
    assert effective_auto_brake(ATTRS[mode], 'OUT', False, None) is False


@pytest.mark.parametrize('mode', MANUAL_LIKE)
def test_manual_in_default_on(mode):
    assert effective_auto_brake(ATTRS[mode], 'IN', False, None) is True


@pytest.mark.parametrize('mode', MANUAL_LIKE)
def test_manual_override_both_directions(mode):
    # 場外でも ON にでき、場内でも OFF にできる（既定であって固定ではない）。
    assert effective_auto_brake(ATTRS[mode], 'OUT', False, True) is True
    assert effective_auto_brake(ATTRS[mode], 'IN', False, False) is False
    assert effective_auto_brake(ATTRS[mode], 'OUT', False, False) is False
    assert effective_auto_brake(ATTRS[mode], 'IN', False, True) is True


@pytest.mark.parametrize('mode', MANUAL_LIKE)
def test_zone_na_is_always_on(mode):
    # ゾーン NA（画面途絶・使用中 0 台を含む）は要求があっても ON。
    for ov in (None, True, False):
        assert effective_auto_brake(ATTRS[mode], 'NA', False, ov) is True
        assert effective_auto_brake(ATTRS[mode], 'NA', True, ov) is True


@pytest.mark.parametrize('zone', ['', 'XX', 'out'])
def test_unknown_zone_is_on(zone):
    assert effective_auto_brake(ATTRS['MANUAL'], zone, False, False) is True


@pytest.mark.parametrize('mode', LOCKED)
@pytest.mark.parametrize('zone', ['IN', 'OUT', 'NA'])
@pytest.mark.parametrize('override', [None, True, False])
def test_locked_modes_cannot_be_disabled_without_jog(mode, zone, override):
    assert effective_auto_brake(ATTRS[mode], zone, False, override) is True
    assert override_allowed(ATTRS[mode], False) is False


@pytest.mark.parametrize('mode', LOCKED)
def test_jog_in_locked_mode_follows_zone_default(mode):
    # ジョグ中は元のモードが自律系でも手動系として扱う（場外 OFF・場内 ON）。
    assert effective_auto_brake(ATTRS[mode], 'OUT', True, None) is False
    assert effective_auto_brake(ATTRS[mode], 'IN', True, None) is True
    assert override_allowed(ATTRS[mode], True) is True
    # 試験員の切り替えもジョグ中は効く。
    assert effective_auto_brake(ATTRS[mode], 'OUT', True, True) is True
    assert effective_auto_brake(ATTRS[mode], 'IN', True, False) is False


def test_override_ignored_when_not_allowed_even_if_false():
    # OFF の要求が古いまま残っていても、自律系に入れば ON（無効化不可）。
    assert effective_auto_brake(ATTRS['FOLLOW'], 'OUT', False, False) is True


def test_exhaustive_never_off_when_locked_or_na():
    # 全モード×ゾーン×ジョグ×要求の全探索: OFF になってよいのは
    # 「手動系またはジョグ中」かつ「ゾーンが IN/OUT」だけ。
    for mode, zone, jog, ov in itertools.product(
            ATTRS, ['IN', 'OUT', 'NA'], [False, True], [None, True, False]):
        off = not effective_auto_brake(ATTRS[mode], zone, jog, ov)
        if off:
            assert zone in ('IN', 'OUT')
            assert override_allowed(ATTRS[mode], jog)


def test_zone_default_matches_effective_without_override():
    for mode, zone, jog in itertools.product(ATTRS, ['IN', 'OUT', 'NA'], [False, True]):
        assert zone_default(ATTRS[mode], zone, jog) == effective_auto_brake(
            ATTRS[mode], zone, jog, None)


def test_reject_reason_key_is_registered_in_webui_reasons():
    here = os.path.dirname(__file__)
    path = os.path.join(here, '..', '..', '..', 'web_ui', 'src', 'i18n', 'reasons.js')
    if not os.path.exists(path):
        pytest.skip('web_ui が見えない環境（Docker）。ホストで確かめる')
    with open(path, encoding='utf-8') as f:
        assert AUTO_BRAKE_LOCKED_REASON in f.read()
