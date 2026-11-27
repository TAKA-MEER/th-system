"""test_pin_order_core.py — 配電盤の順番変更の純ロジック（R-5。D60・D64）。

順番は pins.yaml のリストの並び（/onsite/pins の配列の並び）。
対象は PANEL だけ（HOME の位置は動かない）。
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_onsite'))

from th_onsite.pin_order_core import move_panel, panel_ids, reorder_panels


def _pins():
    return [
        {"id": "home", "kind": "HOME"},
        {"id": "p1", "kind": "PANEL"},
        {"id": "p2", "kind": "PANEL"},
        {"id": "p3", "kind": "PANEL"},
    ]


def _ids(pins):
    return [p["id"] for p in pins]


def test_move_forward_and_back():
    out, err = move_panel(_pins(), "p2", -1)
    assert err == ""
    assert _ids(out) == ["home", "p2", "p1", "p3"]
    out, err = move_panel(_pins(), "p2", 1)
    assert err == ""
    assert _ids(out) == ["home", "p1", "p3", "p2"]


def test_move_at_edge_is_rejected():
    out, err = move_panel(_pins(), "p1", -1)
    assert out is None and err
    out, err = move_panel(_pins(), "p3", 1)
    assert out is None and err


def test_move_unknown_or_home_is_rejected():
    out, err = move_panel(_pins(), "home", 1)
    assert out is None and err
    out, err = move_panel(_pins(), "nope", 1)
    assert out is None and err


def test_reorder_pressed_order_then_rest_in_current_order():
    out, err = reorder_panels(_pins(), ["p3", "p1"])
    assert err == ""
    assert _ids(out) == ["home", "p3", "p1", "p2"]


def test_reorder_full_and_empty():
    out, err = reorder_panels(_pins(), ["p2", "p3", "p1"])
    assert err == "" and _ids(out) == ["home", "p2", "p3", "p1"]
    out, err = reorder_panels(_pins(), [])
    assert err == "" and _ids(out) == ["home", "p1", "p2", "p3"]


def test_reorder_rejects_unknown_duplicate_and_non_panel():
    out, err = reorder_panels(_pins(), ["p1", "nope"])
    assert out is None and err
    out, err = reorder_panels(_pins(), ["p1", "p1"])
    assert out is None and err
    out, err = reorder_panels(_pins(), ["home"])
    assert out is None and err


def test_panel_ids():
    assert panel_ids(_pins()) == ["p1", "p2", "p3"]
