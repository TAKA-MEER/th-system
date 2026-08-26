"""test_zones.py — derive_limits() / mode_speed_limit() / combine_speed_limits() の単体試験
（N-13・N-15）。

DetailedDesign-names.md §4.1「ゾーンの合成は『速度上限の最小値』で行う」を担保する。
`th_state/th_state/zones.py` は rclpy に依存しないので、この試験も ROS2 を必要としない。

対象は §4.1 の表そのもの:
  | 使用中の端末が 1 台     | その画面の上限            | その画面のゾーン        |
  | 使用中の端末が複数      | 各画面の上限の最小値       | いずれかが IN なら IN   |
  | 途絶／使用中が 0 台     | 停止                      | NA。自動ブレーキ ON     |

特に「ゾーンの強弱（IN > NA > OUT）で合成してはいけない」（R3-39/R2-26）を直接検査する。

N-15: DetailedDesign-safety.md §3.3.1「applied_limit_mps = min(画面由来, モード由来,
v_reverse)」のうち画面由来×モード由来の合成（`combine_speed_limits()`）と、モード由来の
決定（`mode_speed_limit()`。attributes.yaml の speed_limit / jog と AT_PANEL の
v_jog_panel 特例）を検査する。
"""
import pytest

from th_state.zones import (ScreenInput, combine_speed_limits, derive_limits,
                             mode_speed_limit)
from th_state.zones import LIMIT_STRICTNESS


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
    # v_check と v_leash のどちらが厳しいかは LIMIT_STRICTNESS の順序が決める
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


# ============================================================
# N-15: モード由来の速度上限（mode_speed_limit）と画面×モードの合成（combine_speed_limits）
# ============================================================
def test_mode_speed_limit_plain_modes_return_attribute_value():
    # jog_active に関係なく、speed_limit が stop でないモードはそのまま返す。
    assert mode_speed_limit({"speed_limit": "v_max", "jog": "allowed"}, jog_active=True) == "v_max"
    assert mode_speed_limit({"speed_limit": "v_slow", "jog": "denied"}, jog_active=False) == "v_slow"


def test_mode_speed_limit_at_panel_stop_without_jog():
    # attributes.yaml 冒頭コメント: AT_PANEL の speed_limit は停止（jog_active が false のとき）。
    attrs = {"speed_limit": "stop", "jog": "allowed"}
    assert mode_speed_limit(attrs, jog_active=False) == "stop"


def test_mode_speed_limit_at_panel_jog_active_returns_v_jog_panel():
    # attributes.yaml 冒頭コメント（※※※）: jog_active の間だけ v_jog_panel。
    attrs = {"speed_limit": "stop", "jog": "allowed"}
    assert mode_speed_limit(attrs, jog_active=True) == "v_jog_panel"


def test_mode_speed_limit_jog_denied_mode_stays_stop_even_if_jog_active():
    # IDLE 等 jog: denied のモードでは、万一 jog_active が立っていても stop のまま
    # （v_jog_panel は AT_PANEL のような「jog 許可」モード限定の特例）。
    attrs = {"speed_limit": "stop", "jog": "denied"}
    assert mode_speed_limit(attrs, jog_active=True) == "stop"


def test_combine_speed_limits_mode_side_stricter_wins():
    # 画面側 v_max（OUT 追従画面）でも、モード側が stop ならモード側が採られる。
    assert combine_speed_limits("v_max", "stop") == "stop"


def test_combine_speed_limits_screen_side_stricter_wins():
    # モード側 v_max でも、画面側が v_check（校正画面が同時使用中等）ならそちらが採られる。
    assert combine_speed_limits("v_check", "v_max") == "v_check"


def test_combine_speed_limits_at_panel_jog_beats_screen_v_slow():
    # names.md §3 の表: AT_PANEL の画面は S-21（IN / v_slow）のみ。ジョグ中は
    # モード由来 v_jog_panel が画面由来 v_slow より厳しく、常に v_jog_panel が勝つ
    # （§3.3.1 の「AT_PANEL のジョグだけは v_jog_panel を明示的に選ぶ」の担保）。
    mode_limit = mode_speed_limit({"speed_limit": "stop", "jog": "allowed"}, jog_active=True)
    assert combine_speed_limits("v_slow", mode_limit) == "v_jog_panel"


def test_combine_speed_limits_screen_disconnect_failsafe_beats_jog():
    # 画面が途絶（derive_limits のフェイルセーフで speed_limit=stop）した場合は、
    # jog_active が立っていても stop が勝つ（v_jog_panel より stop の方が厳しい）。
    mode_limit = mode_speed_limit({"speed_limit": "stop", "jog": "allowed"}, jog_active=True)
    assert combine_speed_limits("stop", mode_limit) == "stop"


# ============================================================
# N-15: attributes.yaml と LIMIT_STRICTNESS の乖離を機械で防ぐ
# ============================================================
def test_all_mode_speed_limits_are_known_to_limit_strictness(state_core_bundle):
    """attributes.yaml の全モードの speed_limit が LIMIT_STRICTNESS に載っていることを
    表明する（新しいモード／新しい speed_limit 名を足したときにここで落ちるようにする）。"""
    _, _, _, attributes = state_core_bundle
    unknown = {
        mode: attrs["speed_limit"]
        for mode, attrs in attributes.items()
        if attrs["speed_limit"] not in LIMIT_STRICTNESS
    }
    assert unknown == {}, (
        f"LIMIT_STRICTNESS に無い speed_limit 名が attributes.yaml にある: {unknown}")


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
