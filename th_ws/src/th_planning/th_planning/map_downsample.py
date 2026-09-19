#!/usr/bin/env python3
# ============================================================
# map_downsample.py — OccupancyGrid の表示用ダウンサンプル（WS-9G）
# ============================================================
# slam_toolbox の /map は resolution 0.05m/セルで、校舎 1 周（80m×60m 級）になると
# 約 192 万セル = 5.8MB/枚（JSON）になり、直したばかりの 2.4GHz 無線を食い潰す。
# 表示は 1280×720 の俯瞰なので 0.05m/セルは不要。SLAM 自身の解像度は落とさず、
# 表示用のコピーだけを factor 倍のセル幅に畳んで配信する。
#
# 純関数（ROS2 を import しない）なので pytest で直接叩ける。既存の
# route_replay_core.py / odom_source.py と同じ二層構造の「純コア」。


def downsample_occupancy(data, width, height, factor, occupied_threshold=50):
    """OccupancyGrid の data を factor×factor ブロックで間引く。

    data: 長さ width*height の列（行優先。値は -1=未知 / 0..100=占有確率）。
          list でも bytes/bytearray でも受け付ける。
    戻り値: (new_data, new_width, new_height) で new_data は list[int]。

    ブロック内の合成規則（表示用なので「安全側＝障害物を消さない」）:
      1. ブロック内に occupied_threshold 以上のセルが 1 つでもあれば、その最大値
      2. でなければ、0 以上のセル（既知の自由空間）が 1 つでもあれば 0
      3. どちらも無ければ -1（未知）

    要件:
      - factor <= 1 なら間引かない（元の内容・幅・高さをそのまま返す）
      - width / height が factor で割り切れないとき、端の半端なブロックも 1 ブロック
        （new_width = ceil(width / factor)）。切り捨てて地図の端を失わない
      - width または height が 0、data が空でも落ちない
      - 元の data を破壊しない
      - 障害物を消さない（規則 1 が規則 2・3 より必ず優先される）
    """
    if factor <= 1 or width <= 0 or height <= 0 or not data:
        return (list(data), width, height)

    new_width = (width + factor - 1) // factor   # ceil(width / factor)
    new_height = (height + factor - 1) // factor

    out = []
    for by in range(new_height):
        r0 = by * factor
        r1 = min(r0 + factor, height)
        for bx in range(new_width):
            c0 = bx * factor
            c1 = min(c0 + factor, width)
            max_occ = -1          # 規則1: ブロック内の占有セルの最大値
            has_free = False      # 規則2: 既知の自由空間（>=0）が 1 つでもある
            for r in range(r0, r1):
                base = r * width
                for c in range(c0, c1):
                    v = data[base + c]
                    if v >= occupied_threshold:
                        if v > max_occ:
                            max_occ = v
                    elif v >= 0:
                        has_free = True
            if max_occ >= occupied_threshold:   # 規則1 が最優先
                out.append(max_occ)
            elif has_free:
                out.append(0)
            else:
                out.append(-1)
    return (out, new_width, new_height)
