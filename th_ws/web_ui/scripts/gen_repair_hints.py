#!/usr/bin/env python3
"""th_maintenance/config/repair_hints.yaml -> web_ui/src/generated/repair_hints.json

WebUI は YAML を読めないので、ビルド前にこの変換をかける（gen_mode_entry.py と同じ流儀）。
S-31（故障診断）が「NG の内容に応じたチェック手順」を出すための入力
（DetailedDesign-maintenance.md §2.6。チェック手順は yaml に持ち、コードに埋めない）。

使い方（リポジトリルートから）:
    python3 th_ws/web_ui/scripts/gen_repair_hints.py .
"""
import json
import pathlib
import sys

import yaml

ROOT = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
SRC = ROOT / "th_ws/src/th_maintenance/config/repair_hints.yaml"
OUT = ROOT / "th_ws/web_ui/src/generated/repair_hints.json"

data = yaml.safe_load(SRC.read_text(encoding="utf-8"))
if not isinstance(data, dict) or not data:
    sys.exit(f"repair_hints.yaml から 0 件しか読めなかった: {SRC}")
for item, reasons in data.items():
    if not isinstance(reasons, dict) or not reasons:
        sys.exit(f"{item}: 理由の表が空・不正")
    for reason, steps in reasons.items():
        if not isinstance(steps, list) or not steps or not all(isinstance(s, str) and s for s in steps):
            sys.exit(f"{item}.{reason}: 手順が空・不正")

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(
    json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(f"{sum(len(v) for v in data.values())} 件 -> {OUT.relative_to(ROOT)}")
