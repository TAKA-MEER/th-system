#!/usr/bin/env python3
"""registry.yaml -> web_ui/src/generated/param_limits.json

S-50 の一般タブにある廃止予定ノードの項目のうち、registry の値と食い違う
上限（`follow_planner_mapless` の `v_max` は UI 上 1.5 まで入れられる）を、
registry の値を超えて保存できないようにするための上限値を吐く
(SG-B9・1b-10)。`v_max` は解決値（導出後。`th_params.export` の本番の解決
関数で求める）を使う。手で書き換えない。作り直し:

    python3 th_ws/web_ui/scripts/gen_param_limits.py .

`speed_presets.json`（`gen_speed_presets.py`）と同じ置き場所・同じ運用。
"""
import json
import pathlib
import sys

ROOT = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
sys.path.insert(0, str(ROOT / "th_ws" / "src" / "th_params"))

SRC = ROOT / "th_ws/src/th_params/config/registry.yaml"
OUT = ROOT / "th_ws/web_ui/src/generated/param_limits.json"

import yaml  # noqa: E402

from th_params import export  # noqa: E402

rows = yaml.safe_load(SRC.read_text(encoding="utf-8"))
if not isinstance(rows, list):
    sys.exit(f"registry.yaml からリストを読めなかった: {SRC}")

resolved = export.resolve_registry(rows)
status, v_max = resolved.get("v_max", ("placeholder", None))
if status == "placeholder" or not isinstance(v_max, (int, float)):
    sys.exit(f"v_max が未解決 (status={status})。registry.yaml を確認すること")

limits = {"v_max": float(v_max)}

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(
    json.dumps(limits, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(f"v_max={limits['v_max']} -> {OUT.relative_to(ROOT)}")
