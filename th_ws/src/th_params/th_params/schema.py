"""schema.py — registry.yaml の 1 行の型と検証（純粋）。

WP-PARAM-01 §1.1・§1.3。**ファイルも環境変数も読まない**（不変条件 P-1）。
呼び出し側（`export.py` / テスト）が `yaml.safe_load()` した dict のリストを渡す。

検証する規則（`DetailedDesign-params.md` §1.3）:

    S1  class: c かつ status != measured ⇒ blocking: true 必須、value は sentinel（`TBD`）のみ
    S2  class: a かつ status != given    ⇒ status は placeholder のみ、value は sentinel（`TBD`）
    S3  class: b は derived か given のみ。derived なら formula と derived_from が必須
    S4  status: measured は measured_at と source が必須
    S5  derived_from のいずれかが placeholder なら、出力も placeholder になる（伝播）

P-3（`status: derived` の行は `value` を持たない）もここで検査する。

sentinel（未測定値のマーカー）について: `DetailedDesign-params.md` §2.1 は
「grep 可能なマーカーは 1 種類だけ・`registry.yaml` にしか現れない」と定めている。
このモジュールは registry.yaml の値をそのまま検証するために sentinel の実体をここに
1 回だけ定義する。完了条件②の grep 検査はこの定義行 1 つにだけヒットする——
これは意図した唯一の例外であり、設計書側にその旨が明記されている
（WP-PARAM-01 の完了報告・指揮担当のレビュー参照）。
呼び出し側（テスト含む）はこの文字列を再度書かず、必ず `schema.TBD` を import して使うこと。
"""
from __future__ import annotations

from typing import Any, Iterable

# sentinel の実体。registry.yaml 以外でこの文字列が現れる唯一の場所（上の docstring 参照）。
TBD = "TBD_MEASURE"

VALID_CLASSES = {"a", "b", "c"}
VALID_STATUSES = {"measured", "derived", "given", "placeholder"}


class SchemaError(Exception):
    """registry の 1 行が S1〜S5 / P-3 のいずれかに違反しているときに送出する。"""


def validate_row(row: dict[str, Any]) -> list[str]:
    """1 行を検証し、違反メッセージのリストを返す（空なら OK）。例外は投げない。"""
    errors: list[str] = []
    name = row.get("name", "<unnamed>")
    cls = row.get("class")
    status = row.get("status")
    value = row.get("value", None)
    blocking = row.get("blocking", False)
    formula = row.get("formula")
    derived_from = row.get("derived_from")

    if cls not in VALID_CLASSES:
        errors.append(f"{name}: class '{cls}' は {sorted(VALID_CLASSES)} のいずれかでない")
    if status not in VALID_STATUSES:
        errors.append(f"{name}: status '{status}' は {sorted(VALID_STATUSES)} のいずれかでない")

    # S1: class c かつ status != measured ⇒ blocking: true 必須、value は sentinel（TBD）のみ
    if cls == "c" and status != "measured":
        if blocking is not True:
            errors.append(f"{name}: S1 違反 - class:c かつ status!=measured は blocking:true が必須")
        if value != TBD:
            errors.append(f"{name}: S1 違反 - class:c かつ status!=measured は value が '{TBD}' のみ許可")

    # S2: class a かつ status != given ⇒ status は placeholder のみ、value は sentinel（TBD）
    if cls == "a" and status != "given":
        if status != "placeholder":
            errors.append(f"{name}: S2 違反 - class:a かつ status!=given は status が 'placeholder' のみ許可")
        if value != TBD:
            errors.append(f"{name}: S2 違反 - class:a かつ status!=given は value が '{TBD}' のみ許可")

    # S3: class b は derived か given のみ。derived なら formula と derived_from が必須
    if cls == "b":
        if status not in ("derived", "given"):
            errors.append(f"{name}: S3 違反 - class:b は status が 'derived' か 'given' のみ許可")
        if status == "derived":
            if not formula:
                errors.append(f"{name}: S3 違反 - status:derived は formula が必須")
            if not derived_from:
                errors.append(f"{name}: S3 違反 - status:derived は derived_from が必須")

    # S4: status: measured は measured_at と source が必須
    if status == "measured":
        if not row.get("measured_at"):
            errors.append(f"{name}: S4 違反 - status:measured は measured_at が必須")
        if not row.get("source"):
            errors.append(f"{name}: S4 違反 - status:measured は source が必須")

    # P-3: status: derived の行は value を持たない（None のみ許可）
    if status == "derived" and value is not None:
        errors.append(f"{name}: P-3 違反 - status:derived の行は value を持たない（null のみ許可）")

    # consumers が空の行は無い（params.md §1.1 の必須項目。A9/CI 側でも検査する）
    if not row.get("consumers"):
        errors.append(f"{name}: consumers が空（死んだパラメータ）")

    # spec_ref は必須項目（params.md §1.1）
    if not row.get("spec_ref"):
        errors.append(f"{name}: spec_ref が無い")

    return errors


def validate_registry(rows: Iterable[dict[str, Any]]) -> list[str]:
    """registry 全体を検証し、違反メッセージのリストを返す（空なら OK）。"""
    errors: list[str] = []
    seen_names: set[str] = set()
    for row in rows:
        errors.extend(validate_row(row))
        name = row.get("name")
        if name in seen_names:
            errors.append(f"{name}: name が重複している")
        elif name:
            seen_names.add(name)
    return errors


def resolved_status(dependency_statuses: Iterable[str]) -> str:
    """S5: 依存先ステータスの列から、導出値のステータスを決める。

    いずれかが 'placeholder' なら 'placeholder' を返す（伝播）。
    それ以外なら 'derived' を返す。
    """
    if any(s == "placeholder" for s in dependency_statuses):
        return "placeholder"
    return "derived"
