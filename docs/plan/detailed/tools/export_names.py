#!/usr/bin/env python3
"""名前辞書 (DetailedDesign-names.md) を機械可読な JSON に落とす。

**Markdown が正本で、JSON は派生物である。**JSON を手で書き換えてはいけない。
辞書に名前を足したら Markdown を直し、このスクリプトを回して JSON を作り直す。

抽出するもの（Markdown の表の第 1 セルから、コードスパンに包まれた名前を拾う）:

| キー | 出どころ | 形 |
| --- | --- | --- |
| `topics`     | `## 6. トピック` 配下の全表 | `/` で始まる名前 |
| `services`   | 同上（`srv` 列を持つ表）＋ `## 5.` の §5.2 | `/` で始まる名前 |
| `params`     | `## 7. パラメータ` 配下の全表 | `/` を含まない小文字スネーク |
| `triggers`   | `## 8. トリガと事象の名前` 配下の全表 | `ui.` / `evt.` / `fault.` / `hw.` / `sys.` で始まる |
| `modes`      | `## 2. モード（18）` | 大文字スネーク |
| `screens`    | `## 4. 画面（15）とゾーン` | `S-nn` |

トピックとサービスは**同じ `/` 始まりなので分離できない。**
両方を `endpoints` にも入れてあり、`topics.js` の突き合わせはそちらを使う
（WebUI から見ると rosbridge の宛先という点で同じものである）。

使い方:
    python3 docs/plan/detailed/tools/export_names.py \
        [--out th_ws/web_ui/src/ros/names.json] [リポジトリルート]

終了コード: 0=生成した / 1=名前が 1 件も取れなかった（表の書式が変わった疑い）
"""
import argparse
import io
import json
import os
import re
import sys

# セクション見出し（`## <番号>.`）→ (収集キー, 見るセルの数)
#
# **モードは 1 セル目ではなく 2 セル目にある**（§2.2 は「群 | モード | …」）。
# 節ごとにどこまで見るかを決め打つ。表の列が増えても壊れないよう、
# 拾った文字列は必ずその節のパターンで濾す。
SECTIONS = {
    "2": ("modes", 2),
    "4": ("screens", 1),
    "5": ("msgs", 1),      # §5.1 は .msg、§5.2 は `/` 始まりのサービス名
    "6": ("topics", 1),
    "7": ("params", 1),
    "8": ("triggers", 2),
}

CODE = re.compile(r'`([^`]+)`')
HEADING = re.compile(r'^## (\d+)\.')

PATTERNS = {
    "topics":   re.compile(r'^/[a-z0-9_/]+$'),
    "params":   re.compile(r'^[a-z][a-z0-9_]*$'),
    "triggers": re.compile(r'^(ui|evt|fault|hw|sys)\.[a-z0-9_.]+$'),
    "modes":    re.compile(r'^[A-Z][A-Z0-9_]*$'),
    "screens":  re.compile(r'^S-\d{2}$'),
    "msgs":     re.compile(r'^[A-Za-z][A-Za-z0-9]*\.(msg|srv)$'),
    "services": re.compile(r'^/[a-z0-9_/]+$'),
}


def parse(path: str) -> dict:
    out = {k: set() for k in PATTERNS}
    cur = None
    for line in io.open(path, encoding="utf-8"):
        h = HEADING.match(line)
        if h:
            cur = SECTIONS.get(h.group(1))
            continue
        if cur is None or not line.startswith("|"):
            continue
        key, ncell = cur
        cells = line.split("|")[1:1 + ncell]
        if not cells or cells[0].strip().startswith("-"):    # 区切り行 (|---|)
            continue
        # §5 だけは 1 セル目に .msg とサービス名の 2 種が混ざる
        keys = (key, "services") if key == "msgs" else (key,)
        for cell in cells:
            # コードスパンだけを見る。`**` / `~~` は CODE が無視する
            for name in CODE.findall(cell):
                name = name.strip()
                for k in keys:
                    if PATTERNS[k].match(name):
                        out[k].add(name)
    return {k: sorted(v) for k, v in out.items()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", nargs="?", default=".")
    ap.add_argument("--out", default=os.path.join(
        "th_ws", "web_ui", "src", "ros", "names.json"))
    a = ap.parse_args()

    src = os.path.join(a.root, "docs", "plan", "detailed",
                       "DetailedDesign-names.md")
    if not os.path.exists(src):
        print(f"ERROR: 名前辞書が無い: {src}", file=sys.stderr)
        return 1

    d = parse(src)
    # WebUI から見ると rosbridge の宛先という点でトピックとサービスは同じもの
    d["endpoints"] = sorted(set(d["topics"]) | set(d["services"]))
    d["_source"] = "docs/plan/detailed/DetailedDesign-names.md"
    d["_generated_by"] = "docs/plan/detailed/tools/export_names.py"

    total = sum(len(v) for k, v in d.items() if not k.startswith("_"))
    if total == 0:
        print("ERROR: 名前が 1 件も取れなかった。表の書式が変わった疑い",
              file=sys.stderr)
        return 1

    out = os.path.join(a.root, a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with io.open(out, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write("\n")

    for k in ("modes", "screens", "topics", "services", "params",
              "triggers", "msgs"):
        print(f"  {k:9} {len(d[k])}")
    print(f"-> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
