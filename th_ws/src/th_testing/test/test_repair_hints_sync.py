"""test_repair_hints_sync.py — S-31（故障診断）のチェック手順の正本 yaml と WebUI 用 json の一致。

repair_hints.yaml（th_maintenance/config）→ gen_repair_hints.py → web_ui/src/generated/repair_hints.json。
変換のし忘れ（yaml だけ直して json が古い）と、check_core.py の NG 理由（ESTOP / MOTOR）に対する
手順の抜けを止める。ROS2 不要（素の pytest で走る）。
"""
import json
import os
import re

import yaml

_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_REPO = os.path.abspath(os.path.join(_SRC, "..", ".."))
YAML_PATH = os.path.join(_SRC, "th_maintenance", "config", "repair_hints.yaml")
JSON_PATH = os.path.join(_REPO, "th_ws", "web_ui", "src", "generated", "repair_hints.json")
CHECK_CORE = os.path.join(_SRC, "th_maintenance", "th_maintenance", "check_core.py")


def _load():
    with open(YAML_PATH, encoding="utf-8") as f:
        y = yaml.safe_load(f)
    with open(JSON_PATH, encoding="utf-8") as f:
        j = json.load(f)
    return y, j


def test_json_is_generated_from_yaml():
    y, j = _load()
    assert j == y, "repair_hints.json が repair_hints.yaml と違う。gen_repair_hints.py を実行すること"


def test_every_non_calibrable_ng_reason_has_hints():
    """ESTOP / MOTOR の NG 理由（check_core.py の NG("...")）全部に手順がある。"""
    y, _ = _load()
    with open(CHECK_CORE, encoding="utf-8") as f:
        src = f.read()

    def block(name):
        m = re.search(rf"def {name}\(.*?(?=\ndef |\Z)", src, re.S)
        assert m, name
        return m.group(0)

    estop = set(re.findall(r'NG\("([a-z_]+)"\)', block("judge_estop")))
    motor_block = block("judge_motor") + block("judge_motor_samples")
    motor = set(re.findall(r'NG\("([a-z_]+)"\)', motor_block))
    motor |= {f"{p}_{side}" for p in ("sign_mismatch", "no_follow") for side in ("L", "R")
              if f"NG(f\"{p}_{{side}}\")" in motor_block}
    assert estop and motor
    assert estop <= set(y["ESTOP"]), f"ESTOP の手順が無い理由: {estop - set(y['ESTOP'])}"
    assert motor <= set(y["MOTOR"]), f"MOTOR の手順が無い理由: {motor - set(y['MOTOR'])}"


def test_only_non_calibrable_items_have_hints():
    """IMU / LIDAR は校正へ誘導する（故障診断の対象ではない）。"""
    y, _ = _load()
    assert set(y) == {"ESTOP", "MOTOR"}
