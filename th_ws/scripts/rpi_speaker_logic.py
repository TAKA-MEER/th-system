#!/usr/bin/env python3
"""rpi_speaker_logic.py — ラズパイ側の再生判定の純粋ロジック（Spec-safety.md §11）。

rclpy を import しない（ホストの素の pytest で試験する。PC 側の
speaker_announce_core と同じ二層）。ROS に触る部分は rpi_speaker_play.py が持つ。

受信する cue JSON（/speaker/cue。PC の speaker_announcer が 2 Hz で出す）:
  {"cue": "none"|"drive"|"blocked"|"warning", "enabled": bool,
   "volume": 0〜100, "test": bool, "rpi_ok": str/bool（読まない）}

出す status JSON（/speaker/status。1 Hz）:
  {"cue": 再生中の cue（止まっていたら "none"）, "volume": int, "error": str}
"""
import json

CUE_NONE = "none"
CUE_DRIVE = "drive"
CUE_BLOCKED = "blocked"
CUE_WARNING = "warning"
CUES = frozenset({CUE_NONE, CUE_DRIVE, CUE_BLOCKED, CUE_WARNING})

# cue → wav ファイル名（wav 自体は make_speaker_chimes.py が作る）。
CUE_WAVS = {CUE_DRIVE: "drive.wav", CUE_BLOCKED: "blocked.wav",
            CUE_WARNING: "warning.wav"}

# 配信がこの秒数途絶えたら止める（PC 側は 2 Hz で出す）。
STALE_S = 1.0


def parse_cue(data: str):
    """cue JSON を辞書に直す。壊れていたら None（止める側に倒す）。"""
    try:
        obj = json.loads(data)
    except ValueError:
        return None
    if not isinstance(obj, dict):
        return None
    cue = obj.get('cue')
    if cue not in CUES:
        return None
    volume = obj.get('volume')
    if not isinstance(volume, int) or isinstance(volume, bool):
        return None
    return {'cue': cue, 'enabled': obj.get('enabled') is True,
            'volume': min(100, max(0, volume)),
            'test': obj.get('test') is True}


def resolve_playback(cue_msg, last_recv_age_s):
    """いま鳴らすべき wav のキー（"drive" 等）を返す。止めるときは None。

    cue_msg: parse_cue の結果（None＝壊れた受信・未受信と同じ扱い）。
    last_recv_age_s: 最後の受信からの経過秒（None＝一度も受信なし）。
    切り替えは即時。none・無効・配信途絶で停止。
    """
    if cue_msg is None:
        return None
    if last_recv_age_s is None or last_recv_age_s >= STALE_S:
        return None
    if not cue_msg['enabled']:
        return None
    if cue_msg['cue'] == CUE_NONE:
        return None
    return cue_msg['cue']


def make_status(playing_cue_or_none, volume: int, error: str = '') -> str:
    """status JSON を作る。"""
    return json.dumps({'cue': playing_cue_or_none or CUE_NONE,
                       'volume': int(volume), 'error': error})


def parse_status(data: str):
    """status JSON を辞書に直す（PC 側・WebUI 側の表示用。壊れていたら None）。"""
    try:
        obj = json.loads(data)
    except ValueError:
        return None
    if not isinstance(obj, dict) or obj.get('cue') not in CUES:
        return None
    return obj
