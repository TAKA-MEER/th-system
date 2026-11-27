"""pin_order_core.py — 配電盤ピンの順番を変える純ロジック（R-5。D60・D64）。

順番は pins.yaml のリストの並びそのもの（/onsite/pins の配列の並び）として
保存・配信する。「次の配電盤」はこの並びで進む（Spec-webui §3.10。機体側は
行き先を panel_id で指定されるだけなので、順序の意味付けは利用側が持つ）。

対象は kind == "PANEL" のピンだけ（待機場所 HOME の位置は動かさない）。
PANEL ピンが占めていた位置（スロット）はそのままに、PANEL の並びだけを
入れ替える。押さなかった盤は現在の順のまま後ろへ（D64）。

ROS2 非依存（pin_clearance_core と同じ作法）。
"""

from __future__ import annotations


def panel_ids(pins: list) -> list:
    """PANEL ピンの id を現在の並びで返す。"""
    return [p.get("id", "") for p in pins if p.get("kind") == "PANEL"]


def reorder_panels(pins: list, order_ids: list) -> tuple:
    """配電盤の並びをまとめて決める。返り値 (新しい pins, エラー文)。

    order_ids に載った PANEL をその順に並べ、載らなかった PANEL は現在の
    順のまま後ろへ付ける。未知の id・重複・PANEL でないピンの id は拒否
    （新しい pins は None）。
    """
    panels = {p.get("id", ""): p for p in pins if p.get("kind") == "PANEL"}
    seen = set()
    for pid in order_ids:
        if pid in seen:
            return None, f"順番に重複があります: {pid}"
        seen.add(pid)
        if pid not in panels:
            return None, f"配電盤ピンが見つかりません: {pid}"
    ordered = [panels[pid] for pid in order_ids]
    ordered += [p for p in pins
                if p.get("kind") == "PANEL" and p.get("id", "") not in seen]
    out = []
    it = iter(ordered)
    for p in pins:
        out.append(next(it) if p.get("kind") == "PANEL" else p)
    return out, ""


def move_panel(pins: list, pin_id: str, move: int) -> tuple:
    """配電盤を 1 つ前（move=-1）／後ろ（move=+1）へ動かす。

    前後とも PANEL の並びの中での移動（HOME を飛び越えない）。端にある盤を
    外へ動かす要求は拒否（新しい pins は None）。
    """
    if move not in (-1, 1):
        return None, f"move は -1（前へ）か 1（後ろへ）です: {move}"
    ids = panel_ids(pins)
    if pin_id not in ids:
        return None, f"配電盤ピンが見つかりません: {pin_id}"
    i = ids.index(pin_id)
    j = i + move
    if j < 0 or j >= len(ids):
        return None, "端の配電盤はそれ以上動かせません"
    ids[i], ids[j] = ids[j], ids[i]
    return reorder_panels(pins, ids)
