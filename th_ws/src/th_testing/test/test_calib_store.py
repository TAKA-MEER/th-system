"""test_calib_store.py — 校正の永続化（現在値＋履歴 3 世代＋項目ごとのロールバック）。

DetailedDesign-maintenance.md §7 の #3（確定しかけを残さない）・#6（ロールバックで 1 つ前に戻る）。
"""
import os
import sys

import yaml

_MAINT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "th_maintenance"))
sys.path.insert(0, _MAINT)

from th_maintenance.calib_store import CalibStore, MAX_GENERATIONS  # noqa: E402


def _commit(store, item, scale, when="2026-10-01T00:00:00"):
    return store.commit(item, {"wheel_radius_scale": scale}, when,
                        {"result": "OK"}, operator="tester", params_digest="abc123")


def test_empty_store_has_no_entries(tmp_path):
    s = CalibStore(str(tmp_path / "calib"))
    assert s.load_current() == {}
    assert s.history("LINEAR") == []
    assert s.current_entry("LINEAR") is None


def test_commit_writes_current_with_all_fields(tmp_path):
    s = CalibStore(str(tmp_path), robot_id="r1")
    _commit(s, "LINEAR", 0.98)
    e = s.current_entry("LINEAR")
    assert e["values"] == {"wheel_radius_scale": 0.98}
    assert e["calibrated_at"] == "2026-10-01T00:00:00"
    assert e["verification"] == {"result": "OK"}
    assert e["operator"] == "tester"
    assert e["params_digest"] == "abc123"
    assert s.robot_id() == "r1"


def test_commit_pushes_previous_to_history_newest_first(tmp_path):
    s = CalibStore(str(tmp_path))
    for v in (0.98, 0.99, 1.01):
        _commit(s, "LINEAR", v)
    assert s.current_entry("LINEAR")["values"]["wheel_radius_scale"] == 1.01
    assert [h["values"]["wheel_radius_scale"] for h in s.history("LINEAR")] == [0.99, 0.98]


def test_history_keeps_only_three_generations(tmp_path):
    s = CalibStore(str(tmp_path))
    for v in (0.91, 0.92, 0.93, 0.94, 0.95, 0.96):
        _commit(s, "LINEAR", v)
    hist = [h["values"]["wheel_radius_scale"] for h in s.history("LINEAR")]
    assert len(hist) == MAX_GENERATIONS
    assert hist == [0.95, 0.94, 0.93]


def test_rollback_restores_one_before_and_only_that_item(tmp_path):
    s = CalibStore(str(tmp_path))
    _commit(s, "LINEAR", 0.98)
    _commit(s, "LINEAR", 0.99)
    s.commit("ROTATION", {"wheel_base": 0.40}, "t", {"result": "OK"})
    restored = s.rollback("LINEAR", 1)
    assert restored["values"]["wheel_radius_scale"] == 0.98
    assert s.current_entry("LINEAR")["values"]["wheel_radius_scale"] == 0.98
    # 別項目は動かない
    assert s.current_entry("ROTATION")["values"]["wheel_base"] == 0.40
    # 置き換えられた 0.99 が履歴の先頭に残る（取り消し可能）
    assert s.history("LINEAR")[0]["values"]["wheel_radius_scale"] == 0.99


def test_rollback_with_missing_generation_changes_nothing(tmp_path):
    s = CalibStore(str(tmp_path))
    _commit(s, "LINEAR", 0.98)
    assert s.rollback("LINEAR", 1) is None
    assert s.rollback("LINEAR", 0) is None
    assert s.rollback("LINEAR", 4) is None
    assert s.current_entry("LINEAR")["values"]["wheel_radius_scale"] == 0.98


def test_rollback_generation_two(tmp_path):
    s = CalibStore(str(tmp_path))
    for v in (0.97, 0.98, 0.99):
        _commit(s, "LINEAR", v)
    s.rollback("LINEAR", 2)
    assert s.current_entry("LINEAR")["values"]["wheel_radius_scale"] == 0.97


def test_corrupt_current_yaml_reads_as_empty_not_crash(tmp_path):
    (tmp_path / "current.yaml").write_text(":\n- [broken", encoding="utf-8")
    s = CalibStore(str(tmp_path))
    assert s.load_current() == {}


def test_write_is_atomic_no_tmp_left(tmp_path):
    s = CalibStore(str(tmp_path))
    _commit(s, "LINEAR", 0.98)
    leftovers = [p for _, _, fs in os.walk(tmp_path) for p in fs if p.endswith(".tmp")]
    assert leftovers == []
    with open(tmp_path / "current.yaml", encoding="utf-8") as f:
        assert yaml.safe_load(f)["items"]["LINEAR"]


def test_status_view_carries_verification_for_s40(tmp_path):
    """S-40 の一覧・履歴に出す 3 つ（last / last_verification / history）。
    検証記録（verification）が前回値と世代の行に載ること。
    要再校正の自動判定はしない（判定の材料だけ）。"""
    from th_maintenance.calib_store import status_view
    s = CalibStore(str(tmp_path / "calib"))
    s.commit("LINEAR", {"wheel_radius_scale": 0.98}, "2026-10-01T00:00:00",
             {"result": "OK", "commanded": 2.0, "measured_verify": 1.99},
             operator="tester")
    s.commit("LINEAR", {"wheel_radius_scale": 0.99}, "2026-10-02T00:00:00",
             {"result": "OK", "commanded": 2.0, "measured_verify": 2.01},
             operator="tester")
    last, last_verification, history = status_view(s, ["LINEAR", "ROTATION"])
    assert last == {"LINEAR": "2026-10-02T00:00:00", "ROTATION": ""}
    assert last_verification["LINEAR"]["measured_verify"] == 2.01
    assert last_verification["ROTATION"] == {}
    assert [r["generation"] for r in history["LINEAR"]] == [1]
    # 履歴の先頭（1 世代前）は 1 回目の確定＝古い方の値と検証記録。
    assert history["LINEAR"][0]["values"]["wheel_radius_scale"] == 0.98
    assert history["LINEAR"][0]["verification"]["measured_verify"] == 1.99
    assert history["ROTATION"] == []
