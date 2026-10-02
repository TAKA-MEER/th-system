#!/usr/bin/env python3
"""W-01 P0: 結果 CSV の集計 + 失敗地点 ASCII マップ."""
import csv
import json
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from w01_p0_measure import load_map, raycast, gen_scan, score_batch  # noqa
import numpy as np


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def table(rows, label):
    for v in ("clean", "occluded"):
        g = [r for r in rows if r["variant"] == v]
        ok = [r for r in g if r["ok"] == "True"]
        ng = [r for r in g if r["ok"] != "True"]
        tt = sorted(float(r["t_total"]) for r in g)
        pe = sorted(float(r["pos_err"]) for r in g)
        ye = sorted(float(r["yaw_err_deg"]) for r in g)
        ss = sorted(float(r["s"]) for r in g)
        mm = sorted(float(r["m"]) for r in g)
        print(f"| {label} | {v} | {len(g)} | {len(ok)} "
              f"({100*len(ok)/len(g):.0f}%) | {st.median(tt):.1f} | {max(tt):.1f} "
              f"| {st.median(pe):.2f} | {st.median(ye):.1f} "
              f"| {st.median(ss):.2f} | {st.median(mm):.2f} |")
        if ng:
            s_ng = sorted(float(r["s"]) for r in ng)
            m_ng = sorted(float(r["m"]) for r in ng)
            print(f"  ng: n={len(ng)} s_med={st.median(s_ng):.2f} "
                  f"m_med={st.median(m_ng):.2f} "
                  f"s_max={max(s_ng):.2f} m_max={max(m_ng):.2f}")


def ascii_map(maps_dir, map_id, rows):
    m = load_map(maps_dir, map_id)
    ds = 8 if max(m["H"], m["W"]) > 800 else 4
    occ = m["occ"][::ds, ::ds]
    pgm_raw = occ  # occupiedのみ見る
    H, W = occ.shape
    cv = np.full((H, W), " ", dtype="<U1")
    cv[occ] = "#"
    # 空き（未知は空白のまま）
    from w01_p0_measure import read_pgm
    pgm = read_pgm(os.path.join(maps_dir, map_id + ".pgm"))[::-1, :]
    cv[(pgm[::ds, ::ds] == 254)] = "."
    for r in rows:
        gx = int((float(r["true_x"]) - m["ox"]) / m["res"] / ds)
        gy = int((float(r["true_y"]) - m["oy"]) / m["res"] / ds)
        if 0 <= gx < W and 0 <= gy < H:
            ch = "X" if r["ok"] != "True" else ("o" if cv[gy, gx] in " ." else "O")
            cv[gy, gx] = ch
    print(f"--- {map_id} (X=失敗, o=正解, #=壁, .=空き) ---")
    for row in cv[::2]:
        print("".join(row))


if __name__ == "__main__":
    maps_dir, res_dir = sys.argv[1], sys.argv[2]
    print("| 地図 | 条件 | n | 正解 | t中央値[s] | t最大[s] "
          "| 誤差中央値[m] | 角度中央値[deg] | s中央値 | m中央値 |")
    for fn in sorted(os.listdir(res_dir)):
        if fn.endswith(".csv"):
            rows = load_csv(os.path.join(res_dir, fn))
            table(rows, fn[:-4])
    if "--map" in sys.argv:
        for mid in sys.argv[sys.argv.index("--map") + 1:]:
            rows = load_csv(os.path.join(res_dir, mid + ".csv"))
            ascii_map(maps_dir, mid, rows)
