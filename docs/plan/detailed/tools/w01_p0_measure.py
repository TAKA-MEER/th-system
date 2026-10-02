#!/usr/bin/env python3
"""W-01 P0: 全域ローカライズ粗探索のオフライン実現性検証.

設計: docs/plan/detailed/DetailedDesign-transit-localize-options.md §0.
  - 入力: .briefs/tmp/maps/<id>.pgm/.yaml（slam_toolbox localization +
    deserialize_map(match_type=3) + map_saver_cli で .posegraph/.data から変換）
    + メイン作業ツリー th_ws/data/routes/<id>.json（経路点列, map フレーム）
  - 模擬スキャン: LiDAR 実測値に合わせ N=720・0.5°刻み・angle_min=-pi
    （obstacle_limiter_core.cpp WS-9P 実測 2026-09-04）。
    max_range=40.0 は RPLIDAR S1 公称の仮定（rosbag が無いため実測なし。P6 で再確認）。
    死角は registry.yaml の blind_angle_ranges（実測 2026-09-09, 4 セクタ）を抜く。
  - 探索: 案Aの粗→細（A: 0.5m/10°全域・間引き180ビーム・tol 0.3 → NMS上位60
    → B: ±1m/0.25m・±15°/5°（tol 0.15）→ C: ±0.3m/0.1m・±5°/1°（tol 0.1））。
    スコアは尤度場による一致率 s、
    マージン m は「2m 以上離れた2番目」（同じくB/C精密化後）との差（§2）。
    1m/15°では真値 basin が埋もれることを確認済み（report.md §2）。
  - 模擬スキャンは地図と同じ世界から作るため実スキャンより楽観的
    （家具移動・人・ガラス・歪み無し）。実スキャン再確認は P6 に回す。

使い方: venv/bin/python w01_p0_measure.py [--maps ...] [--stride N] [--out DIR]
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:
    sys.exit("scipy が要る（venv に入れること）")

# ---- LiDAR 前提（実測・仕様の根拠は docstring） ----
N_BEAMS = 720
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / N_BEAMS
MAX_RANGE = 40.0
MIN_RANGE = 0.1
# registry.yaml blind_angle_ranges（度, laser_link 基準・反時計回り正・平坦配列）
BLIND_DEG = [-132.8, -117.5, -59.5, -38.9, 45.9, 61.5, 130.3, 143.5]
# スコア許容（m）。S1 測距ノイズ ±5cm 級 + 量子化を見込む
TOL_M = 0.10
# m の「別の場所」の定義（m）
M_SEP_M = 2.0
# 正解の定義
OK_POS_M = 0.5
OK_YAW_DEG = 5.0

BEAM_ANGLES = ANGLE_MIN + np.arange(N_BEAMS) * ANGLE_INC  # laser_link 基準


def blind_mask():
    deg = np.degrees(BEAM_ANGLES)
    deg = (deg + 180.0) % 360.0 - 180.0
    m = np.zeros(N_BEAMS, dtype=bool)
    for a0, a1 in zip(BLIND_DEG[0::2], BLIND_DEG[1::2]):
        if a0 <= a1:
            m |= (deg >= a0) & (deg <= a1)
        else:  # -180/180 をまたぐ場合の保険
            m |= (deg >= a0) | (deg <= a1)
    return m


BLIND = blind_mask()


def read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        assert magic == b"P5", magic
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline().strip())
        assert maxval == 255
        buf = f.read(w * h)
    return np.frombuffer(buf, dtype=np.uint8).reshape(h, w)


def load_map(maps_dir, map_id):
    pgm = read_pgm(os.path.join(maps_dir, map_id + ".pgm"))
    meta = {}
    with open(os.path.join(maps_dir, map_id + ".yaml")) as f:
        for line in f:
            if ":" in line:
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy, _ = [float(v) for v in meta["origin"].strip("[]").split(",")]
    occ = (pgm == 0)[::-1, :]  # pgm 先頭行が最大 y（map_saver 仕様）のため反転し、
    free = (pgm == 254)[::-1, :]  # occ[gy,gx] が (ox+gx*res, oy+gy*res) に対応させる
    # EDT: 占有セルまでの距離（m）。未知は通過可として扱う（楽観的。報告に明記）。
    dt = distance_transform_edt(~occ).astype(np.float32) * res
    return {"occ": occ, "free": free, "dt": dt, "res": res,
            "ox": ox, "oy": oy, "H": occ.shape[0], "W": occ.shape[1]}


def world_to_grid(x, y, m):
    gx = np.floor((np.asarray(x) - m["ox"]) / m["res"]).astype(np.int64)
    gy = np.floor((np.asarray(y) - m["oy"]) / m["res"]).astype(np.int64)
    return gx, gy


def raycast(px, py, yaw, occ, res, ox, oy, max_range=MAX_RANGE, step=None):
    """ベクトル化マーチング DDA。全 720 ビーム同時進行。戻り値: ranges(720,)."""
    if step is None:
        step = 0.5 * res
    H, W = occ.shape
    dx = np.cos(BEAM_ANGLES + yaw)
    dy = np.sin(BEAM_ANGLES + yaw)
    rng = np.full(N_BEAMS, max_range)
    alive = np.ones(N_BEAMS, dtype=bool)
    pos = np.empty((N_BEAMS, 2))
    pos[:, 0] = px
    pos[:, 1] = py
    # 開始セルが占有（経路点が壁にめり込み）の検出用
    gx0 = np.floor((px - ox) / res).astype(int)
    gy0 = np.floor((py - oy) / res).astype(int)
    start_occ = (0 <= gx0 < W and 0 <= gy0 < H and occ[gy0, gx0])
    d = 0.0
    while alive.any() and d < max_range:
        d += step
        pos[alive, 0] = px + dx[alive] * d
        pos[alive, 1] = py + dy[alive] * d
        gx = np.floor((pos[alive, 0] - ox) / res).astype(np.int64)
        gy = np.floor((pos[alive, 1] - oy) / res).astype(np.int64)
        idx = np.flatnonzero(alive)
        oob = (gx < 0) | (gx >= W) | (gy < 0) | (gy >= H)
        hit = ~oob & occ[np.clip(gy, 0, H - 1), np.clip(gx, 0, W - 1)]
        done = alive.copy()
        done[idx[oob]] = False  # 範囲外は max_range のまま無効化
        rng[idx[hit]] = d
        done[idx[hit]] = False
        alive = alive & done
        alive[idx[oob]] = False
    return rng, start_occ


def gen_scan(true_ranges, seed, occluded):
    rng = np.random.default_rng(seed)
    r = true_ranges.copy()
    r += rng.normal(0.0, 0.03 if not occluded else 0.05, size=N_BEAMS)
    if occluded:
        # 人や物の想定: 1〜3 人分相当（8〜25°幅を 1m 先の人体相当の手前距離に）
        for _ in range(int(rng.integers(1, 4))):
            center = rng.uniform(-math.pi, math.pi)
            width = math.radians(float(rng.uniform(8.0, 25.0)))
            factor = float(rng.uniform(0.3, 0.7))
            d = np.abs((BEAM_ANGLES - center + math.pi) % (2 * math.pi) - math.pi)
            sel = d < width / 2
            r[sel] = np.clip(true_ranges[sel] * factor, MIN_RANGE, MAX_RANGE)
        drop = rng.random(N_BEAMS) < 0.02
        r[drop] = MAX_RANGE + 1.0  # 欠損（無効）
    valid = (~BLIND) & (r > MIN_RANGE) & (r < MAX_RANGE - 1e-6)
    return r, valid


def score_batch(cands, ranges, valid, m, tol=TOL_M):
    """cands: (C,3) x,y,yaw。戻り値: s (C,)。tol は段階で変える（粗=広く・細=狭く）。"""
    dt, res, ox, oy, H, W = m["dt"], m["res"], m["ox"], m["oy"], m["H"], m["W"]
    rv = ranges[valid]  # (V,)
    bv = BEAM_ANGLES[valid]  # (V,)
    yaw = cands[:, 2:3]  # (C,1)
    ex = cands[:, 0:1] + rv[None, :] * np.cos(yaw + bv[None, :])
    ey = cands[:, 1:2] + rv[None, :] * np.sin(yaw + bv[None, :])
    gx = np.floor((ex - ox) / res).astype(np.int64)
    gy = np.floor((ey - oy) / res).astype(np.int64)
    oob = (gx < 0) | (gx >= W) | (gy < 0) | (gy >= H)
    look = np.empty_like(gx, dtype=np.float32)
    look[oob] = np.inf
    ok = ~oob
    look[ok] = dt[gy[ok], gx[ok]]
    return (look <= tol).mean(axis=1)


def coarse_grid(m, dxy=1.0, dth_deg=10.0):
    step = int(round(dxy / m["res"]))
    ys, xs = np.nonzero(m["free"][::step, ::step])
    wx = m["ox"] + (xs * step + 0.5) * m["res"]
    wy = m["oy"] + (ys * step + 0.5) * m["res"]
    th = np.arange(0.0, 2 * math.pi, math.radians(dth_deg))
    C = len(wx) * len(th)
    cands = np.empty((C, 3))
    k = 0
    for t in th:
        n = len(wx)
        cands[k:k + n, 0] = wx
        cands[k:k + n, 1] = wy
        cands[k:k + n, 2] = t
        k += n
    return cands


def nms_pick(A, sA, n, sep_m=1.5):
    order = np.argsort(sA)[::-1]
    picked = []
    for i in order:
        if all(math.hypot(A[i, 0] - A[j, 0], A[i, 1] - A[j, 1]) >= sep_m
               for j in picked):
            picked.append(int(i))
            if len(picked) >= n:
                break
    return picked


def refine_grid(best, dxy, xy_step, dth_deg, th_step_deg):
    xs = np.arange(-dxy, dxy + 1e-9, xy_step)
    ts = np.radians(np.arange(-dth_deg, dth_deg + 1e-9, th_step_deg))
    C = len(xs) * len(xs) * len(ts)
    c = np.empty((C, 3))
    k = 0
    for dx in xs:
        for dy in xs:
            c[k:k + len(ts), 0] = best[0] + dx
            c[k:k + len(ts), 1] = best[1] + dy
            c[k:k + len(ts), 2] = best[2] + ts
            k += len(ts)
    return c


def refine_best(center, ranges, valid, m):
    B = refine_grid(center, 1.0, 0.25, 15.0, 5.0)
    sB = score_batch(B, ranges, valid, m, tol=0.15)
    best = B[int(np.argmax(sB))].copy()
    C = refine_grid(best, 0.3, 0.1, 5.0, 1.0)
    sC = score_batch(C, ranges, valid, m, tol=TOL_M)
    j = int(np.argmax(sC))
    return C[j].copy(), float(sC[j])


def search(ranges, valid, m):
    t0 = time.perf_counter()
    # A: 全域粗探索（0.5m/10°・間引き180ビーム・tol 0.3。1m 刻みでは真値 basin が
    # 351/2700 位に埋もれて拾えないことを確認済み）
    A = coarse_grid(m, dxy=0.5, dth_deg=10.0)
    sub = np.zeros(N_BEAMS, dtype=bool)
    sub[::4] = True
    tA0 = time.perf_counter()
    sA = score_batch(A, ranges, valid & sub, m, tol=0.30)
    tA = time.perf_counter() - tA0
    top = nms_pick(A, sA, 60, sep_m=1.5)
    # B+C: 上位 40 を精密化し最良を取る
    tB0 = time.perf_counter()
    best, sb = None, -1.0
    for i in top:
        b, s = refine_best(A[i], ranges, valid, m)
        if s > sb:
            sb, best = s, b
    tB = time.perf_counter() - tB0
    # m: best から 2m 以上離れた候補のうち最良を同じく精密化し、その差
    tC0 = time.perf_counter()
    d = np.hypot(A[:, 0] - best[0], A[:, 1] - best[1])
    far = np.flatnonzero(d >= M_SEP_M)
    if far.size:
        i2 = far[int(np.argmax(sA[far]))]
        _, s2 = refine_best(A[i2], ranges, valid, m)
    else:
        s2 = 0.0
    tC = time.perf_counter() - tC0
    total = time.perf_counter() - t0
    return best, sb, sb - s2, {"tA": tA, "tB": tB, "tC": tC,
                              "total": total, "nA": len(A)}


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run_map(maps_dir, routes_dir, map_id, stride, out_csv):
    m = load_map(maps_dir, map_id)
    with open(os.path.join(routes_dir, map_id + ".json")) as f:
        route = json.load(f)
    pts = np.array(route["points"], dtype=float)
    idxs = list(range(0, len(pts), stride))
    rows = []
    for k, i in enumerate(idxs):
        tx, ty, tyaw = pts[i]
        true_ranges, start_occ = raycast(tx, ty, tyaw, m["occ"], m["res"],
                                         m["ox"], m["oy"])
        gx, gy = world_to_grid(tx, ty, m)
        in_free = (0 <= gx < m["W"] and 0 <= gy < m["H"]
                   and bool(m["free"][gy, gx]))
        for variant in ("clean", "occluded"):
            r, valid = gen_scan(true_ranges, seed=1000 + i * 2 +
                                (variant == "occluded"), occluded=(variant == "occluded"))
            best, s, marg, t = search(r, valid, m)
            pe = math.hypot(best[0] - tx, best[1] - ty)
            ye = abs(math.degrees(wrap_pi(best[2] - tyaw)))
            rows.append({
                "map": map_id, "pose_idx": i, "variant": variant,
                "true_x": tx, "true_y": ty,
                "true_yaw_deg": math.degrees(tyaw),
                "est_x": best[0], "est_y": best[1],
                "est_yaw_deg": math.degrees(wrap_pi(best[2])),
                "pos_err": pe, "yaw_err_deg": ye,
                "s": s, "m": marg, "n_valid": int(valid.sum()),
                "t_total": t["total"], "tA": t["tA"],
                "n_cand_A": t["nA"],
                "in_free": in_free, "start_occ": start_occ,
                "ok": (pe <= OK_POS_M and ye <= OK_YAW_DEG),
            })
        print(f"  {map_id}: {k + 1}/{len(idxs)} done", flush=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return rows


def summarize(rows):
    import collections
    out = {}
    for (v), g in groupby(rows, "variant"):
        tt = sorted(r["t_total"] for r in g)
        ok = [r for r in g if r["ok"]]
        ng = [r for r in g if not r["ok"]]
        out[v] = {
            "n": len(g), "n_ok": len(ok),
            "t_med": med(tt), "t_max": max(tt),
            "pos_med": med(sorted(r["pos_err"] for r in g)),
            "yaw_med": med(sorted(r["yaw_err_deg"] for r in g)),
            "s_med": med(sorted(r["s"] for r in g)),
            "m_med": med(sorted(r["m"] for r in g)),
            "s_ng": sorted((r["s"], r["m"], r["pose_idx"]) for r in ng),
            "fail_xy": [(r["pose_idx"], r["true_x"], r["true_y"]) for r in ng],
        }
    return out


def groupby(rows, key):
    d = {}
    for r in rows:
        d.setdefault(r[key], []).append(r)
    return d.items()


def med(s):
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps-dir", required=True)
    ap.add_argument("--routes-dir", required=True)
    ap.add_argument("--maps", nargs="+", required=True)
    ap.add_argument("--stride", type=int, default=30)
    ap.add_argument("--out-dir", required=True)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    import platform
    try:
        with open("/proc/cpuinfo") as f:
            cpu = [l.split(":", 1)[1].strip() for l in f
                   if l.startswith("model name")][0]
    except Exception:
        cpu = platform.processor()
    print(f"cpu: {os.cpu_count()} x {cpu}")
    print(f"numpy {np.__version__}")
    allsum = {}
    for map_id in a.maps:
        print(f"== {map_id} ==")
        rows = run_map(a.maps_dir, a.routes_dir, map_id, a.stride,
                       os.path.join(a.out_dir, map_id + ".csv"))
        s = summarize(rows)
        allsum[map_id] = s
        for v, d in s.items():
            print(f"  [{v}] n={d['n']} ok={d['n_ok']} "
                  f"t_med={d['t_med']:.2f}s t_max={d['t_max']:.2f}s "
                  f"pos_med={d['pos_med']:.3f}m yaw_med={d['yaw_med']:.2f}deg "
                  f"s_med={d['s_med']:.3f} m_med={d['m_med']:.3f}")
            for sv, mv, pi in d["s_ng"]:
                print(f"    FAIL idx={pi} s={sv:.3f} m={mv:.3f}")
    with open(os.path.join(a.out_dir, "summary.json"), "w") as f:
        json.dump({"cpu": f"{os.cpu_count()} x {cpu}",
                   "numpy": np.__version__,
                   "tol_m": TOL_M, "m_sep_m": M_SEP_M,
                   "ok_def": f"pos<={OK_POS_M}m,yaw<={OK_YAW_DEG}deg",
                   "summary": allsum}, f, indent=1)


if __name__ == "__main__":
    main()
