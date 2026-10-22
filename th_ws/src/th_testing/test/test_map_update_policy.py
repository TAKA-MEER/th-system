"""test_map_update_policy.py — 地図更新 ON の拒否判定（th_state.map_update_policy）

地図更新 ON は当面使えない（2026-10-22）。ON への変更だけ拒否し、OFF は受ける。
ROS2 なし・純粋 Python で実行可能。
"""
from th_state.map_update_policy import (MAP_UPDATE_ON_DENIED_REASON,
                                        map_update_on_denied)


def test_on_is_denied():
    assert map_update_on_denied("map_update", True)


def test_off_is_allowed():
    assert not map_update_on_denied("map_update", False)


def test_other_flags_are_not_affected():
    for flag in ("tracker_enabled", "auto_brake"):
        assert not map_update_on_denied(flag, True), flag
        assert not map_update_on_denied(flag, False), flag


def test_reason_key_matches_webui():
    assert MAP_UPDATE_ON_DENIED_REASON == "map_update_unavailable"
