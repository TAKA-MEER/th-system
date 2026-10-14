"""rpi_speaker_logic の試験（ROS 不要の素の pytest）。

受信 JSON → 何を鳴らすか・途絶判定・status JSON を縛る。ROS に触る部分
（rpi_speaker_play.py のスレッド・paplay 呼び出し）は対象外。
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

from rpi_speaker_logic import (CUE_WAVS, STALE_S, make_status, parse_cue,  # noqa: E402
                               parse_status, resolve_playback)


def cue_msg(cue='drive', enabled=True, volume=60, test=False):
    return {'cue': cue, 'enabled': enabled, 'volume': volume, 'test': test}


def test_parse_valid():
    m = parse_cue(json.dumps(cue_msg()))
    assert m == {'cue': 'drive', 'enabled': True, 'volume': 60, 'test': False}


def test_parse_clamps_volume():
    assert parse_cue(json.dumps(cue_msg(volume=120)))['volume'] == 100
    assert parse_cue(json.dumps(cue_msg(volume=-5)))['volume'] == 0


def test_parse_broken_is_none():
    assert parse_cue('not json') is None
    assert parse_cue(json.dumps(['drive'])) is None
    assert parse_cue(json.dumps(cue_msg(cue='loud'))) is None
    assert parse_cue(json.dumps(cue_msg(volume='60'))) is None
    assert parse_cue(json.dumps(cue_msg(volume=True))) is None


def test_resolve_switches_immediately():
    assert resolve_playback(cue_msg('drive'), 0.1) == 'drive'
    assert resolve_playback(cue_msg('blocked'), 0.1) == 'blocked'
    assert resolve_playback(cue_msg('warning'), 0.1) == 'warning'


def test_resolve_stops():
    assert resolve_playback(cue_msg('none'), 0.1) is None
    assert resolve_playback(cue_msg('drive', enabled=False), 0.1) is None
    assert resolve_playback(None, 0.1) is None
    assert resolve_playback(cue_msg('drive'), None) is None
    assert resolve_playback(cue_msg('drive'), STALE_S) is None
    assert resolve_playback(cue_msg('drive'), STALE_S + 1.0) is None


def test_every_cue_has_a_wav():
    assert set(CUE_WAVS) == {'drive', 'blocked', 'warning'}


def test_status_roundtrip():
    s = make_status('drive', 60, '')
    assert parse_status(s)['cue'] == 'drive'
    assert make_status(None, 60) == json.dumps({'cue': 'none', 'volume': 60, 'error': ''})
    assert parse_status('broken') is None
