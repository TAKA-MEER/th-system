"""test_onsite_context.py — onsite_context 派生値の単体テスト

WP-ONSITE-F2。state_manager のコールバックが使う純関数の検証。
ROS2 なし・純粋 Python で実行可能。conftest.py が th_state を sys.path に足す。
"""
from th_state.onsite_context import derive_person_ctx, derive_pin_kinds


class TestDerivePersonCtx:
    def test_no_candidates_no_selection_no_confidence(self):
        assert derive_person_ctx(0, -1, 0.0, 0.5) == (0, False, False)

    def test_candidates_but_unselected_confident(self):
        """候補あり・未選択・信頼度十分 → target_confident は True（guard 側で
        target_selected and target_confident に合成される）。"""
        assert derive_person_ctx(3, -1, 0.9, 0.5) == (3, False, True)

    def test_selected_confident(self):
        assert derive_person_ctx(2, 1, 0.9, 0.5) == (2, True, True)

    def test_selected_index_zero_but_low_confidence(self):
        """selected_index=0（正規の選択）でも信頼度が足りなければ False。
        index>=0 の判定が `>0` に化けた場合にここが赤になる。"""
        assert derive_person_ctx(2, 0, 0.4, 0.5) == (2, True, False)

    def test_confidence_boundary_equal(self):
        assert derive_person_ctx(1, 0, 0.5, 0.5) == (1, True, True)


class TestDerivePinKinds:
    def test_dedupe_and_sort(self):
        assert derive_pin_kinds(["HOME", "PANEL", "PANEL"]) == ("HOME", "PANEL")

    def test_empty(self):
        assert derive_pin_kinds([]) == ()

    def test_skips_empty_string(self):
        assert derive_pin_kinds(["PANEL", "", "HOME"]) == ("HOME", "PANEL")