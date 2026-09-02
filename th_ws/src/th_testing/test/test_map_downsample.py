"""
test_map_downsample.py
=======================
map_downsample.downsample_occupancy の単体テスト（WS-9G）。

slam_toolbox の /map（resolution 0.05m/セル）は校舎 1 周級になると数十万〜百万セルと
巨大になり 2.4GHz 無線を食い潰すため、表示用に factor×factor ブロックへ畳む。
SLAM 自身の解像度は落とさず、この純関数は「表示用コピー」を間引く。

最も大事な性質は「障害物を消さない」こと（ブロック内に占有セルが 1 つでもあれば
占有になる＝規則 1 が規則 2・3 より優先）。ROS2 不要・純粋 Python。
"""

from th_planning.map_downsample import downsample_occupancy


def _fmt(d, w, h):
    return [list(d[r * w:(r + 1) * w]) for r in range(h)]


# ── factor <= 1 は間引かない ─────────────────────────────

def test_factor_1_returns_unchanged():
    data = [0, 100, -1, 50]
    nd, nw, nh = downsample_occupancy(data, 4, 1, 1)
    assert nd == data
    assert (nw, nh) == (4, 1)


def test_factor_0_returns_unchanged():
    data = [0, 100, -1]
    nd, nw, nh = downsample_occupancy(data, 3, 1, 0)
    assert nd == data
    assert (nw, nh) == (3, 1)


# ── 基本形・2×2 → 1 セル ────────────────────────────────

def test_2x2_factor_2_folds_to_one_cell():
    nd, nw, nh = downsample_occupancy([0, 0, 0, 0], 2, 2, 2)
    assert nd == [0]
    assert (nw, nh) == (1, 1)


# ── 障害物を消さない（最重要）────────────────────────────

def test_block_with_any_occupied_becomes_occupied():
    # 右下だけ 100 でも、ブロック全体が占有になる（自由空間 0 と同居していても）。
    nd, nw, nh = downsample_occupancy([0, 0, 0, 100], 2, 2, 2)
    assert nd == [100]
    assert (nw, nh) == (1, 1)


def test_block_takes_max_of_occupied_cells():
    nd, _, _ = downsample_occupancy([0, 70, 0, 100], 2, 2, 2)
    assert nd == [100]


def test_occupied_wins_over_free():
    # 自由空間 (0) と占有 (60) が混在したら占有が優先される
    nd, _, _ = downsample_occupancy([0, 0, 0, 60], 2, 2, 2)
    assert nd == [60]


# ── 占有が無ければ自由・全て未知 ─────────────────────────

def test_free_only_block_becomes_0():
    nd, _, _ = downsample_occupancy([0, 0, 10, 0], 2, 2, 2)
    assert nd == [0]


def test_all_unknown_block_stays_unknown():
    nd, _, _ = downsample_occupancy([-1, -1, -1, -1], 2, 2, 2)
    assert nd == [-1]


# ── 割り切れないサイズ（端を失わない）────────────────────

def test_non_divisible_size_keeps_edge():
    # 5×3 を factor=2 → 3×2。最右下の角 (row2, col4) だけ占有 → 端のブロックが
    # 落ちていない（out[-1] が占有）。
    width, height = 5, 3
    data = [-1] * (width * height)
    data[2 * width + 4] = 100   # (row2, col4)
    nd, nw, nh = downsample_occupancy(data, width, height, 2)
    assert (nw, nh) == (3, 2)
    assert len(nd) == 6
    assert nd[-1] == 100   # 端（最右下）の占有が残っている


# ── 空・ゼロ幅で落ちない ────────────────────────────────

def test_empty_data_and_zero_width_do_not_crash():
    assert downsample_occupancy([], 0, 0, 4) == ([], 0, 0)
    assert downsample_occupancy([], 4, 0, 4) == ([], 4, 0)


# ── bytes / bytearray ──────────────────────────────────

def test_bytes_input_works():
    nd, nw, nh = downsample_occupancy(bytes([0, 100, 0, 0]), 2, 2, 2)
    assert nd == [100]
    assert (nw, nh) == (1, 1)


def test_bytearray_input_works():
    nd, _, _ = downsample_occupancy(bytearray([50, 0, 0, 0]), 2, 2, 2)
    assert nd == [50]


# ── 校舎 1 周の実寸（1600×1200 → 400×300）──────────────

def test_1600x1200_factor_4_drops_to_400x300():
    width, height = 1600, 1200
    data = [-1] * (width * height)   # 全部未知から始める
    # 右下の角だけ占有 → 端のブロックも畳まれていることを確認
    data[(height - 1) * width + (width - 1)] = 100
    nd, nw, nh = downsample_occupancy(data, width, height, 4)
    assert (nw, nh) == (400, 300)
    assert len(nd) == 400 * 300
    assert nd[-1] == 100


# ── 元の data を破壊しない ─────────────────────────────

def test_does_not_mutate_input():
    data = [0, 0, 0, 100]
    snapshot = list(data)
    downsample_occupancy(data, 2, 2, 2)
    assert data == snapshot
