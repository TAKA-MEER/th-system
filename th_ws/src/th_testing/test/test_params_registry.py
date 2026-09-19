"""test_params_registry.py — registry.yaml が正本 names.md §7 と一致していること等を検証する
（WP-PARAM-01 §7）。

`names_md_file` フィクスチャは conftest.py の `resolve_docs_root()` / `detailed_design_path()`
を使う（WP-STATE-01 が Spec-modes.md 用に入れた 3 段解決の仕組みをそのまま流用したもの。
新しく作らない）。
"""
import re

from th_params import assertions


def _extract_param_names_from_names_md_section7(text: str) -> set[str]:
    """DetailedDesign-names.md の '## 7. パラメータ' 節から名前の集合を抽出する。

    抽出対象は次の 2 パターンだけ:
      1. テーブル行の先頭セル（`名前` 列）に書かれたバッククォート付き識別子
      2. 節内にまれにある、行頭がバッククォートで始まる列挙文（§7.4 の許容範囲の行）

    `speed_preset_low` ／ `_mid` ／ `_high` のような継続表記（前の名前の接尾辞だけを
    書く省略形）は、直前の完全な名前の prefix（最後の `_` の手前まで）と結合して復元する。
    """
    start = text.index("## 7. パラメータ")
    end = text.index("## 8. ", start)
    section = text[start:end]

    names: set[str] = set()
    last_prefix: str | None = None

    for raw_line in section.splitlines():
        line = raw_line.strip()
        if line.startswith("|"):
            cells = line.split("|")
            if len(cells) < 2:
                continue
            tokens = re.findall(r"`([^`]+)`", cells[1])
        elif line.startswith("`"):
            tokens = re.findall(r"`([^`]+)`", line)
        else:
            continue

        for tok in tokens:
            if tok.startswith("_"):
                if last_prefix is None:
                    continue
                full = last_prefix + tok
            else:
                full = tok
                last_prefix = full.rsplit("_", 1)[0] if "_" in full else full
            # 識別子っぽくないものは除外（念のための防御。実際には現れない想定）
            if re.fullmatch(r"[a-z][a-z0-9_]*", full):
                names.add(full)

    return names


def test_names_extraction_matches_known_examples(names_md_file):
    """パーサ自体の健全性チェック（既知のいくつかの名前が正しく取れること）。"""
    text = names_md_file.read_text(encoding="utf-8")
    names = _extract_param_names_from_names_md_section7(text)

    assert "v_max" in names
    assert "brake_accel_mps2" in names
    # 継続表記の復元
    assert "speed_preset_low" in names
    assert "speed_preset_mid" in names
    assert "speed_preset_high" in names
    # §7.4 の行頭バッククォート列挙
    assert "calib_linear_tolerance_ratio" in names
    assert "calib_rotation_tolerance_deg" in names
    assert "calib_blind_tolerance_deg" in names
    # ゾーン名・モード名など、パラメータ名でないものは含まれない
    assert "OUT" not in names
    assert "LEASH" not in names


def test_names_match_names_md(registry_rows, names_md_file):
    """registry.yaml の name 集合 == names.md §7 の名前集合。

    既知の不一致: DetailedDesign-params.md §6.1・§6.3 は names.md §7 に無い名前
    （fault_to_stop_ms・wheel_base_m・wheel_radius_m・camera_lift_tolerance_m/deg・
    panel_count・panel_spacing_m・panel_front_space_m・prep_baseline_min・floor_type・
    unmanned_permission・carry_baseline_min）を挙げているが、names.md §7 が「パラメータ名の正」
    と指定されているため、registry.yaml にはこれらを含めていない
    （WP-PARAM-01 の完了報告に明記した判断）。
    """
    text = names_md_file.read_text(encoding="utf-8")
    names_md_names = _extract_param_names_from_names_md_section7(text)
    registry_names = {row["name"] for row in registry_rows}

    missing_in_registry = names_md_names - registry_names
    extra_in_registry = registry_names - names_md_names

    assert missing_in_registry == set(), f"names.md にあるが registry.yaml に無い: {missing_in_registry}"
    assert extra_in_registry == set(), f"registry.yaml にあるが names.md に無い: {extra_in_registry}"


def test_consumers_nonempty(registry_rows):
    """A9: 全パラメータの consumers が空でない（死んだパラメータの検出）。"""
    errors = assertions.a9_consumers_nonempty(registry_rows)
    assert errors == [], f"consumers が空の行がある: {errors}"


def test_registry_has_at_least_one_tbd_measure(registry_rows):
    """未測定の全件が 1 コマンドで出ること（P-2）の前提: registry に sentinel（schema.TBD）が実在する。"""
    from th_params import schema
    tbd_count = sum(1 for row in registry_rows if row.get("value") == schema.TBD)
    assert tbd_count >= 1


def test_class_c_rows_all_have_blocking_and_stage(registry_rows):
    """S1 の帰結: class:c かつ status!=measured の行はすべて blocking_from_stage を持つ。"""
    for row in registry_rows:
        if row.get("class") == "c" and row.get("status") != "measured":
            assert row.get("blocking") is True, f"{row['name']} に blocking:true が無い"
            assert isinstance(row.get("blocking_from_stage"), int), \
                f"{row['name']} に blocking_from_stage が無い"
