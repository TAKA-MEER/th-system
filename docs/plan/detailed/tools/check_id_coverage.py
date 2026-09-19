#!/usr/bin/env python3
"""正本 (docs/plan/spec/) の ID が詳細設計 (docs/plan/detailed/) で覆われているか検査する。

DetailedDesign-open.md §5.4 の引用方針を機械化したもの。

- 直接引用が必要な族  : SD / G / U / C / F / O-a / O-c / O-d
- 引用しない族（出自）: CL / T-r / O-r / C-r / K-r / N / Z / DF-D / E
  → これらは Spec-open.md §6 が F-nn / O-xx に処理済みで、詳細設計はその処理先を引用する

終了コード: 0=OK / 1=直接引用が必要な ID に漏れがある
使い方: python3 docs/plan/detailed/tools/check_id_coverage.py [リポジトリルート]
"""
import collections
import glob
import io
import os
import re
import sys

ID_PAT = re.compile(
    r'\b(SD-?\d+|G-?\d+|U-?\d+|C-?\d+|F-?\d+'
    r'|O-[acd]\d+|CL-[A-Z]-\d+|DF-D-\d+'
    r'|T-r\d+|O-r\d+|C-r\d+|K-r\d+|Z-?\d+|N-?\d+|E-?\d+)\b'
)

# 直接引用が必要な族（§5.4.1）
MUST_CITE = ("SD", "G", "U", "C", "F", "O-a", "O-c", "O-d")
# 出自の記号。引用しない（Spec-open.md §6 が処理先へ写している）
PROVENANCE = ("CL", "T-r", "O-r", "C-r", "K-r", "N", "Z", "DF-D", "E")


def normalize(raw: str) -> str:
    """`F-06` と `F-6`、`G5` と `G-5` を同じ ID として扱う。"""
    m = re.match(r'([A-Za-z-]*?)-?(\d+)$', raw)
    return m.group(1).rstrip('-') + "-" + str(int(m.group(2)))


def family(ident: str) -> str:
    """`CL-B-1` -> `CL-B`、`T-r-3` -> `T-r`、`F-6` -> `F`。"""
    return ident.rsplit('-', 1)[0]


def in_group(ident: str, group: tuple) -> bool:
    """族の接頭辞で判定する。`CL-B-1` は `CL` に属する。
    `C-14` が `CL` に吸われないよう、区切りまで一致することを要求する。"""
    fam = family(ident)
    return any(fam == g or fam.startswith(g + "-") for g in group)


def collect(pattern: str) -> dict:
    found = collections.defaultdict(set)
    for path in sorted(glob.glob(pattern)):
        text = io.open(path, encoding="utf-8").read()
        for m in ID_PAT.finditer(text):
            found[normalize(m.group(1))].add(os.path.basename(path))
    return found


def main() -> int:
    root = sys.argv[1] if len(sys.argv) > 1 else "."
    spec_glob = os.path.join(root, "docs", "plan", "spec", "*.md")
    det_glob = os.path.join(root, "docs", "plan", "detailed", "*.md")

    spec = collect(spec_glob)
    det = collect(det_glob)
    if not spec:
        print(f"ERROR: 正本が見つからない: {spec_glob}", file=sys.stderr)
        return 2

    uncited = sorted(set(spec) - set(det))
    # 出自の族を先に判定する。`C-r2` は `C-nn` ではなく `C-r`（出自）である
    prov = [i for i in uncited if in_group(i, PROVENANCE)]
    must = [i for i in uncited if i not in prov and in_group(i, MUST_CITE)]
    other = [i for i in uncited if i not in must and i not in prov]

    print(f"正本 ID {len(spec)} 件 / 詳細で引用 {len(set(spec) & set(det))} 件 "
          f"/ 未引用 {len(uncited)} 件")
    print(f"  うち出自の族（引用しない・§5.4.1）: {len(prov)} 件")

    if other:
        print(f"\n未分類の族が {len(other)} 件ある。§5.4.1 の表に足すこと:")
        for i in other:
            print(f"    {i}  ({', '.join(sorted(spec[i]))})")

    if must:
        print(f"\n直接引用が必要なのに落ちている ID: {len(must)} 件")
        for i in must:
            print(f"    {i}  ({', '.join(sorted(spec[i]))})")
        return 1

    print("\nOK: 直接引用が必要な ID の漏れは 0 件")
    return 0 if not other else 1


if __name__ == "__main__":
    sys.exit(main())
