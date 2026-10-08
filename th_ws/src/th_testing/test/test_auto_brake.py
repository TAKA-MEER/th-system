"""test_auto_brake.py — 自動ブレーキの有効／無効の純関数（1b-15 SG-B10）。

Spec-safety.md §2.1 の表をそのまま写す。attributes.yaml の実物を読んで、
「どのモードが手動系か」を試験側で決め打ちしない。
"""
import itertools
import os

import pytest
import yaml

from th_state.auto_brake import (AUTO_BRAKE_LOCKED_REASON, DEV_ITEM_AUTO_BRAKE,
                                  DEV_MODE_STALE_MS, dev_item_effective,
                                  effective_auto_brake, override_allowed, zone_default)

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


# ── 開発モードの項目 auto_brake（Spec-safety.md §10。2026-10-08 改定） ─────────

def _dev(dev_mode=True, effective=None):
    import json
    return json.dumps({'dev_mode': dev_mode,
                       'effective': effective if effective is not None
                       else {DEV_ITEM_AUTO_BRAKE: True}})


@pytest.mark.parametrize('mode', LOCKED)
@pytest.mark.parametrize('zone', ['IN', 'OUT'])
def test_dev_item_lets_autonomous_toggle_but_default_is_unchanged(mode, zone):
    # 項目ありで自律系・場内外でも OFF にできる。要求なしの既定は通常と同じ（ON）。
    assert override_allowed(ATTRS[mode], False, True) is True
    assert effective_auto_brake(ATTRS[mode], zone, False, False, True) is False
    assert effective_auto_brake(ATTRS[mode], zone, False, None, True) is True
    assert zone_default(ATTRS[mode], zone, False) is True


@pytest.mark.parametrize('mode', LOCKED)
def test_dev_item_absent_never_turns_autonomous_off(mode):
    assert override_allowed(ATTRS[mode], False, False) is False
    assert effective_auto_brake(ATTRS[mode], 'OUT', False, False, False) is True


@pytest.mark.parametrize('mode', list(ATTRS))
def test_dev_item_never_off_in_zone_na(mode):
    assert effective_auto_brake(ATTRS[mode], 'NA', False, False, True) is True


def test_dev_item_default_does_not_depend_on_item():
    for mode, zone, jog in itertools.product(ATTRS, ['IN', 'OUT', 'NA'], [False, True]):
        assert effective_auto_brake(ATTRS[mode], zone, jog, None, True) == \
            effective_auto_brake(ATTRS[mode], zone, jog, None, False)


def test_dev_item_effective_requires_explicit_true_and_fresh():
    ok = _dev()
    assert dev_item_effective(ok, 1000.0, 1000.0, DEV_ITEM_AUTO_BRAKE) is True
    # 鮮度: ちょうど 3 秒は有効、超えたら失効。
    assert dev_item_effective(ok, 1000.0 + DEV_MODE_STALE_MS, 1000.0, DEV_ITEM_AUTO_BRAKE) is True
    assert dev_item_effective(ok, 1000.0 + DEV_MODE_STALE_MS + 1, 1000.0,
                              DEV_ITEM_AUTO_BRAKE) is False
    # 未受信
    assert dev_item_effective(None, 1000.0, None, DEV_ITEM_AUTO_BRAKE) is False
    # マスタ OFF・effective 無し・型違い・別項目・壊れた JSON
    assert dev_item_effective(_dev(dev_mode=False), 0, 0, DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective('{"dev_mode": true}', 0, 0, DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective('{"dev_mode": true, "effective": []}', 0, 0,
                              DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective(_dev(effective={'scan_stop': True}), 0, 0,
                              DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective(_dev(effective={DEV_ITEM_AUTO_BRAKE: 'true'}), 0, 0,
                              DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective(_dev(effective={DEV_ITEM_AUTO_BRAKE: 1}), 0, 0,
                              DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective('{"dev_mode": "true", "effective": {"auto_brake": true}}', 0, 0,
                              DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective('{broken', 0, 0, DEV_ITEM_AUTO_BRAKE) is False
    assert dev_item_effective('[]', 0, 0, DEV_ITEM_AUTO_BRAKE) is False
