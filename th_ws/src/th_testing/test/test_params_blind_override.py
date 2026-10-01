"""test_params_blind_override.py — 校正で確定した死角マスクが、起動時に生成 yaml の
`blind_angle_ranges` を**4 ノードすべて**で上書きすること（Spec-checks.md §3.5・2026-10-01）。

registry は出荷値、正本は `calib/current.yaml`。上限違反の確定値は使わず出荷値へ戻る。
"""
import os
import sys
import tempfile

import yaml

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _p in (os.path.join(_SRC, "th_bringup", "launch"), os.path.join(_SRC, "th_params"),
           os.path.join(_SRC, "th_maintenance"), os.path.join(_SRC, "th_perception")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import params_generation as pg  # noqa: E402

REGISTRY_YAML = os.path.join(_SRC, "th_params", "config", "registry.yaml")
NODES = ("lidar_filter", "obstacle_limiter", "opcheck_runner", "calib_runner")


def _doc(flat):
    return {"robot_id": "", "items": {"BLIND": {"values": {"blind_angle_ranges": flat}}}}


def test_override_none_when_no_blind_entry():
    assert pg.blind_override_flat({"items": {"LINEAR": {"values": {}}}}) == (None, "")
    assert pg.blind_override_flat(None) == (None, "")


def test_override_returns_confirmed_values():
    flat, warn = pg.blind_override_flat(_doc([0.0, 20.0, 100.0, 120.0]))
    assert flat == [0.0, 20.0, 100.0, 120.0] and warn == ""


def test_override_rejects_over_limit_and_falls_back():
    flat, warn = pg.blind_override_flat(_doc([0.0, 40.0]))
    assert flat is None and "上限違反" in warn
    flat, warn = pg.blind_override_flat(_doc([0.0, 10.0, 5.0]))
    assert flat is None


def test_override_empty_means_mask_removed():
    flat, warn = pg.blind_override_flat(_doc([]))
    assert flat == [] and warn == ""


def _generate(tmp, flat=None, write_current=True):
    out_dir = os.path.join(tmp, "generated")
    calib_dir = os.path.join(tmp, "calib")
    if write_current:
        os.makedirs(calib_dir)
        with open(os.path.join(calib_dir, "current.yaml"), "w", encoding="utf-8") as f:
            yaml.safe_dump(_doc(flat), f)
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([os.path.join(_SRC, "th_params")] + sys.path)
    pg.run_generation(stage=2, sim=False, nodes=list(pg.REGISTRY_NODES), out_dir=out_dir,
                      registry_path=REGISTRY_YAML, env=env, calib_dir=calib_dir)
    return out_dir


def _flat_of(out_dir, node):
    with open(os.path.join(out_dir, f"{node}.yaml"), encoding="utf-8") as f:
        body = yaml.safe_load(f)[node]["ros__parameters"]
    return body.get("blind_angle_ranges")


def test_generated_yaml_of_all_four_nodes_gets_calibrated_mask():
    with tempfile.TemporaryDirectory() as tmp:
        out = _generate(tmp, [10.0, 30.0, -100.0, -80.0])
        for n in NODES:
            assert _flat_of(out, n) == [10.0, 30.0, -100.0, -80.0], n


def test_generated_yaml_without_calib_keeps_registry_value():
    with open(REGISTRY_YAML, encoding="utf-8") as f:
        reg = {p["name"]: p for p in yaml.safe_load(f)}
    with tempfile.TemporaryDirectory() as tmp:
        out = _generate(tmp, write_current=False)
        for n in NODES:
            assert _flat_of(out, n) == reg["blind_angle_ranges"]["value"], n


def test_generated_yaml_over_limit_calib_falls_back_to_registry(capsys):
    with open(REGISTRY_YAML, encoding="utf-8") as f:
        reg = {p["name"]: p for p in yaml.safe_load(f)}
    with tempfile.TemporaryDirectory() as tmp:
        out = _generate(tmp, [0.0, 80.0])
        for n in NODES:
            assert _flat_of(out, n) == reg["blind_angle_ranges"]["value"], n
    assert "上限違反" in capsys.readouterr().err


def test_empty_calibrated_mask_removes_key_everywhere():
    with tempfile.TemporaryDirectory() as tmp:
        out = _generate(tmp, [])
        for n in NODES:
            assert _flat_of(out, n) is None, n


def test_calib_runner_yaml_carries_limits_and_tolerance():
    with tempfile.TemporaryDirectory() as tmp:
        out = _generate(tmp, write_current=False)
        with open(os.path.join(out, "calib_runner.yaml"), encoding="utf-8") as f:
            body = yaml.safe_load(f)["calib_runner"]["ros__parameters"]
        assert body["blind_max_sector_deg"] == 30.0
        assert body["blind_max_total_deg"] == 90.0
        assert body["blind_max_sectors"] == 8
        assert body["calib_blind_tolerance_deg"] == 5.0
        with open(os.path.join(out, "obstacle_limiter.yaml"), encoding="utf-8") as f:
            ob = yaml.safe_load(f)["obstacle_limiter"]["ros__parameters"]
        assert ob["blind_max_sector_deg"] == 30.0 and ob["blind_max_sectors"] == 8
