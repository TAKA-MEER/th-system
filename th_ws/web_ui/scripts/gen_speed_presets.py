#!/usr/bin/env python3
"""th_params/config/registry.yaml -> web_ui/src/generated/speed_presets.json

WebUI は YAML を読めないので、ビルド前にこの変換をかける。
速度プリセット (speed_preset_low / _mid / _high) を JSX に数値で書くのを
防ぐ (規約 R2 / DetailedDesign-wp3.md WP-UI-03 §3.3)。プリセットは「上限に
対する割合」であって上限そのものではないので、value だけを加工せず取り出す。

使い方（リポジトリルートから）:
    python3 th_ws/web_ui/scripts/gen_speed_presets.py .
"""
import json
import pathlib
import sys

import yaml

ROOT = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
SRC = ROOT / "th_ws/src/th_params/config/registry.yaml"
OUT = ROOT / "th_ws/web_ui/src/generated/speed_presets.json"

data = yaml.safe_load(SRC.read_text(encoding="utf-8"))
if not isinstance(data, list):
    sys.exit(f"registry.yaml からリストを読めなかった: {SRC}")

presets = {}
for row in data:
    name = row.get("name")
    if name in ("speed_preset_low", "speed_preset_mid", "speed_preset_high"):
        presets[name] = row.get("value")

if set(presets) != {"speed_preset_low", "speed_preset_mid", "speed_preset_high"}:
    sys.exit(
        f"speed_preset_* を 3 件読めなかった ({len(presets)} 件)。"
        "registry.yaml の書式を確認すること"
    )

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(
    json.dumps(presets, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(f"{len(presets)} 件のプリセット -> {OUT.relative_to(ROOT)}")
