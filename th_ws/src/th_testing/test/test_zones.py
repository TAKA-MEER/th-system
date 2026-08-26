"""test_zones.py — derive_limits() の単体試験（N-13）。

DetailedDesign-names.md §4.1「ゾーンの合成は『速度上限の最小値』で行う」を担保する。
`th_state/th_state/zones.py` は rclpy に依存しないので、この試験も ROS2 を必要としない。

対象は §4.1 の表そのもの:
  | 使用中の端末が 1 台     | その画面の上限            | その画面のゾーン        |
  | 使用中の端末が複数      | 各画面の上限の最小値       | いずれかが IN なら IN   |
  | 途絶／使用中が 0 台     | 停止                      | NA。自動ブレーキ ON     |

特に「ゾーンの強弱（IN > NA > OUT）で合成してはいけない」（R3-39/R2-26）を直接検査する。
"""
import pytest

from th_state.zones import ScreenInput, derive_limits


_WINDOW_S = 5  # ui_active_window_s。テストの都合上の値（意味は「使用中とみなす窓」だけ）


def _screen(screen_id: str, now_ms: int, interacting: bool = True, age_ms: int = 0) -> ScreenInput:
    return ScreenInput(screen_id=screen_id, interacting=interacting,
                        last_input_ms=now_ms - age_ms)


# ============================================================
# 使用中が 0 台・途絶 → 停止側へフェイルセーフ（§4.1 の表の最終行）
# ============================================================
def test_no_active_screens_fails_to_stop():
    limits = derive_limits({}, now_ms=10_000, ui_active_window_s=_WINDOW_S)
    assert limits.zone == "NA"
    assert limits.speed_limit == "stop"
    assert limits.auto_brake is True


def test_all_screens_stale_fails_to_stop():
    now = 10_000
    screens = {
        "c1": _screen("S-10", now, interacting=True, age_ms=_WINDOW_S * 1000 + 1),
    }
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.zone == "NA"
    assert limits.speed_limit == "stop"
    assert limits.auto_brake is True


def test_screen_not_interacting_is_excluded():
    now = 10_000
    screens = {"c1": _screen("S-10", now, interacting=False)}
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.zone == "NA"
    assert limits.speed_limit == "stop"


# ============================================================
# 使用中が 1 台 → その画面の上限・ゾーンそのまま
# ============================================================
@pytest.mark.parametrize("screen_id,expected_zone,expected_limit", [
    ("S-10", "OUT", "v_max"),
    ("S-16", "OUT", "v_leash"),
    ("S-20", "IN", "v_slow"),
    ("S-21", "IN", "v_slow"),
    ("S-30", "NA", "v_check"),
    ("S-31", "NA", "stop"),
    ("S-40", "NA", "v_calib"),
])
def test_single_active_screen_matches_table(screen_id, expected_zone, expected_limit):
    now = 10_000
    screens = {"c1": _screen(screen_id, now)}
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.zone == expected_zone
    assert limits.speed_limit == expected_limit


# ============================================================
# 核心: ゾーンの強弱で合成してはいけない。速度上限は各画面の最小値。
# NA (S-40=v_calib) と IN (S-21=v_slow) が同時使用中 → zone は IN へ倒れて構わないが
# 速度上限は v_calib（NA 側）を採る。IN の v_slow に倒れたら R3-39/R2-26 の再発。
# ============================================================
def test_na_screen_limit_wins_over_in_when_stricter():
    now = 10_000
    screens = {
        "c1": _screen("S-40", now),  # NA / v_calib
        "c2": _screen("S-21", now),  # IN / v_slow
    }
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.zone == "IN"          # ゾーンは IN 側へ（強弱表現ではなく「存在すれば」）
    assert limits.speed_limit == "v_calib"  # だが上限は最小値＝v_calib（v_slow より厳しい）
    assert limits.speed_limit != "v_slow"


def test_na_stop_screen_wins_over_out_v_max():
    now = 10_000
    screens = {
        "c1": _screen("S-31", now),  # NA / stop（故障診断）
        "c2": _screen("S-10", now),  # OUT / v_max（追従走行）
    }
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.speed_limit == "stop"


def test_multiple_screens_take_minimum_speed_limit_not_zone_priority():
    now = 10_000
    # S-30 (NA/v_check) と S-16 (OUT/v_leash) の同時使用。
    # v_check と v_leash のどちらが厳しいかは _LIMIT_STRICTNESS の順序が決める
    # （テストは「NA 側の画面が居合わせても IN の緩い上限へは絶対に倒れない」
    #   ことだけを直接保証する。実際の厳しさ順は zones.py の定義そのものを見る）。
    screens = {
        "c1": _screen("S-30", now),  # NA / v_check
        "c2": _screen("S-20", now),  # IN / v_slow
    }
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.speed_limit == "v_check"  # v_slow ではない


# ============================================================
# 複数端末・同一ゾーン内の最小値
# ============================================================
def test_multiple_out_screens_take_minimum():
    now = 10_000
    screens = {
        "c1": _screen("S-10", now),  # OUT / v_max
        "c2": _screen("S-16", now),  # OUT / v_leash
    }
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.zone == "OUT"
    assert limits.speed_limit == "v_leash"  # v_max より厳しい


def test_mixed_freshness_only_active_screens_count():
    now = 10_000
    screens = {
        "c1": _screen("S-40", now, age_ms=_WINDOW_S * 1000 + 1),  # 期限切れ → 除外
        "c2": _screen("S-10", now),                                # 有効
    }
    limits = derive_limits(screens, now_ms=now, ui_active_window_s=_WINDOW_S)
    assert limits.zone == "OUT"
    assert limits.speed_limit == "v_max"


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
