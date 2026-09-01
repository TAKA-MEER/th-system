#!/usr/bin/env python3
"""th_state/config/mode_entry.yaml -> web_ui/src/generated/mode_entry.json

WebUI は YAML を読めないので、ビルド前にこの変換をかける。
S-01 の「どのモードへ入れるか」をハードコードしないための入力になる
(DetailedDesign-wp1.md WP-UI-02 §4.1 / DetailedDesign-state.md §8.3)。

gen_attributes.py と同じ流儀（sys.argv[1] にリポジトリルート、0 件なら落ちる、
モード数を検査してから書き出す）。

使い方（リポジトリルートから）:
    python3 th_ws/web_ui/scripts/gen_mode_entry.py .
"""
import json
import pathlib
import sys

import yaml

ROOT = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
SRC = ROOT / "th_ws/src/th_state/config/mode_entry.yaml"
OUT = ROOT / "th_ws/web_ui/src/generated/mode_entry.json"

data = yaml.safe_load(SRC.read_text(encoding="utf-8"))
if not isinstance(data, dict) or not data:
    sys.exit(f"mode_entry.yaml から 0 件しか読めなかった: {SRC}")

# CARRY: [] のように空リストの行もあるので None を [] に正規化する
# (YAML の `CARRY: []` はそのまま [] になるが、書式が変わって `CARRY:` だけに
# なった場合は None になるので、そこも落とさず [] 扱いにする)
data = {mode: (entries or []) for mode, entries in data.items()}

# 18 モードあることを検査する。減っていたら書式が変わったとみなして落とす
# (attributes.yaml と同じモード数。gen_attributes.py の検査に合わせる)。
if len(data) != 18:
    sys.exit(f"モード数が 18 でない ({len(data)} 件)。mode_entry.yaml の書式を確認すること")

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(
    json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(f"{len(data)} モード -> {OUT.relative_to(ROOT)}")
