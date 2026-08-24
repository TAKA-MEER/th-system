#!/usr/bin/env python3
"""th_state/config/attributes.yaml -> web_ui/src/generated/attributes.json

WebUI は YAML を読めないので、ビルド前にこの変換をかける。
W-1 の選択肢（resume）をハードコードしないための入力になる
（DetailedDesign-wp1.md WP-UI-01 §4.1 / U-6）。

使い方（リポジトリルートから）:
    python3 th_ws/web_ui/scripts/gen_attributes.py .
"""
import json
import pathlib
import sys

import yaml

ROOT = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
SRC = ROOT / "th_ws/src/th_state/config/attributes.yaml"
OUT = ROOT / "th_ws/web_ui/src/generated/attributes.json"

data = yaml.safe_load(SRC.read_text(encoding="utf-8"))
if not isinstance(data, dict) or not data:
    sys.exit(f"attributes.yaml から 0 件しか読めなかった: {SRC}")

# 18 モードあることを検査する。減っていたら書式が変わったとみなして落とす。
if len(data) != 18:
    sys.exit(f"モード数が 18 でない ({len(data)} 件)。attributes.yaml の書式を確認すること")

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(
    json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(f"{len(data)} モード -> {OUT.relative_to(ROOT)}")
