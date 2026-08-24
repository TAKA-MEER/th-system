"""test_params_schema.py — th_params/schema.py の S1〜S5 を 1 本ずつ検証する（WP-PARAM-01 §7）。

sentinel（未測定値のマーカー）のリテラルはここでは書かない。完了条件②
（未測定 sentinel を `*.py` 内に直書きしていないか grep する検査）を
自分のテストコードで落とさないよう、必ず `schema.TBD` を import して使う
（WP-PARAM-01 の報告参照）。
"""
from th_params import schema


def _base_row(**overrides):
    row = {
        "name": "dummy_param",
        "unit": "m",
        "class": "b",
        "status": "given",
        "value": 1.0,
        "consumers": ["dummy_node"],
        "spec_ref": "dummy",
    }
    row.update(overrides)
    return row


def test_s1_class_c_requires_blocking_and_tbd_measure():
    # 違反: class c / status placeholder だが blocking が無い
    bad = _base_row(**{"class": "c", "status": "placeholder", "value": 0.5, "consumers": ["x"]})
    errors = schema.validate_row(bad)
    assert any("S1" in e for e in errors)

    # 違反: blocking はあるが value が sentinel でない
    bad2 = _base_row(**{"class": "c", "status": "placeholder", "value": 0.5,
                         "blocking": True, "consumers": ["x"]})
    errors2 = schema.validate_row(bad2)
    assert any("S1" in e for e in errors2)

    # 合格: class c / placeholder / blocking:true / value が sentinel
    good = _base_row(**{"class": "c", "status": "placeholder", "value": schema.TBD,
                         "blocking": True, "blocking_from_stage": 0, "consumers": ["x"]})
    assert schema.validate_row(good) == []

    # 合格: class c だが measured（blocking 不要）
    good_measured = _base_row(**{"class": "c", "status": "measured", "value": 0.6,
                                  "measured_at": "2026-08-01", "source": "brake test",
                                  "consumers": ["x"]})
    assert schema.validate_row(good_measured) == []


def test_s2_class_a_requires_placeholder_and_tbd_measure():
    # 違反: class a / status derived（許されるのは given か placeholder のみ）
    bad = _base_row(**{"class": "a", "status": "derived", "value": None, "consumers": ["x"]})
    errors = schema.validate_row(bad)
    assert any("S2" in e for e in errors)

    # 違反: class a / placeholder だが value が sentinel でない
    bad2 = _base_row(**{"class": "a", "status": "placeholder", "value": 0.3, "consumers": ["x"]})
    errors2 = schema.validate_row(bad2)
    assert any("S2" in e for e in errors2)

    # 合格: class a / placeholder / value が sentinel
    good = _base_row(**{"class": "a", "status": "placeholder", "value": schema.TBD,
                         "consumers": ["x"]})
    assert schema.validate_row(good) == []

    # 合格: class a / given（外部から値が確定している）
    good_given = _base_row(**{"class": "a", "status": "given", "value": 0.3, "consumers": ["x"]})
    assert schema.validate_row(good_given) == []


def test_s3_class_b_must_be_derived_or_given():
    # 違反: class b / status measured は許されない
    bad = _base_row(**{"class": "b", "status": "measured", "value": 1.0,
                        "measured_at": "t", "source": "s", "consumers": ["x"]})
    errors = schema.validate_row(bad)
    assert any("S3" in e for e in errors)

    # 違反: class b / status derived だが formula が無い
    bad2 = _base_row(**{"class": "b", "status": "derived", "value": None,
                         "derived_from": ["other_param"], "consumers": ["x"]})
    errors2 = schema.validate_row(bad2)
    assert any("S3" in e for e in errors2)

    # 違反: class b / status derived だが derived_from が無い
    bad3 = _base_row(**{"class": "b", "status": "derived", "value": None,
                         "formula": "some_formula", "consumers": ["x"]})
    errors3 = schema.validate_row(bad3)
    assert any("S3" in e for e in errors3)

    # 合格: class b / given
    good_given = _base_row(**{"class": "b", "status": "given", "value": 1.0, "consumers": ["x"]})
    assert schema.validate_row(good_given) == []

    # 合格: class b / derived（formula と derived_from あり）
    good_derived = _base_row(**{"class": "b", "status": "derived", "value": None,
                                 "formula": "some_formula", "derived_from": ["other_param"],
                                 "consumers": ["x"]})
    assert schema.validate_row(good_derived) == []


def test_s4_measured_requires_measured_at_and_source():
    # 違反: measured_at が無い
    bad = _base_row(**{"class": "c", "status": "measured", "value": 0.6,
                        "source": "brake test", "consumers": ["x"]})
    errors = schema.validate_row(bad)
    assert any("S4" in e for e in errors)

    # 違反: source が無い
    bad2 = _base_row(**{"class": "c", "status": "measured", "value": 0.6,
                         "measured_at": "2026-08-01", "consumers": ["x"]})
    errors2 = schema.validate_row(bad2)
    assert any("S4" in e for e in errors2)

    # 合格
    good = _base_row(**{"class": "c", "status": "measured", "value": 0.6,
                         "measured_at": "2026-08-01", "source": "brake test",
                         "consumers": ["x"]})
    assert schema.validate_row(good) == []


def test_s5_placeholder_propagates_to_derived_status():
    # 違反ケース相当: 依存の 1 つが placeholder なら 'placeholder' を返す（伝播）
    assert schema.resolved_status(["given", "placeholder", "derived"]) == "placeholder"
    assert schema.resolved_status(["placeholder"]) == "placeholder"

    # 合格ケース: 依存がすべて非 placeholder なら 'derived' を返す
    assert schema.resolved_status(["given", "derived", "measured"]) == "derived"
    assert schema.resolved_status([]) == "derived"


def test_p3_derived_row_must_not_have_value():
    bad = _base_row(**{"class": "b", "status": "derived", "value": 1.0,
                        "formula": "f", "derived_from": ["x"], "consumers": ["y"]})
    errors = schema.validate_row(bad)
    assert any("P-3" in e for e in errors)

    good = _base_row(**{"class": "b", "status": "derived", "value": None,
                         "formula": "f", "derived_from": ["x"], "consumers": ["y"]})
    assert schema.validate_row(good) == []


def test_validate_registry_reports_duplicate_names():
    rows = [
        _base_row(**{"name": "dup_name", "consumers": ["a"]}),
        _base_row(**{"name": "dup_name", "consumers": ["b"]}),
    ]
    errors = schema.validate_registry(rows)
    assert any("重複" in e for e in errors)


def test_actual_registry_has_no_schema_violations(registry_rows):
    """実際の registry.yaml が S1〜S5 / P-3 に違反していないこと。"""
    errors = schema.validate_registry(registry_rows)
    assert errors == [], f"registry.yaml にスキーマ違反がある: {errors}"
