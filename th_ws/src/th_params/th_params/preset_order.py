"""preset_order.py — S-50 速度プリセット3段階の順序規則（SG-C14。ROS2 非依存）。

Spec-webui.md §3.15「S-50 の項目の定義」: 低速 ≦ 中速 ≦ 高速を保つ。
各段の設定範囲は「下の段の値以上、上の段の値以下」（低速の下限と高速の
上限は上限そのもの＝100% が天井）。

`params_audit` の `/params/set`（サーバ側の二重ガード）が正本として使い、
WebUI の `presetOrder.js` は「押す前に分かる」ための写し。
"""

from typing import Mapping, Optional

PRESET_ORDER_NAMES = ("speed_preset_low", "speed_preset_mid", "speed_preset_high")
PRESET_MIN = 0.05
PRESET_MAX = 1.0


def check_preset_order(values: Mapping[str, object]) -> Optional[str]:
    """順序違反があれば理由文、無ければ None。

    3 段が数値で揃っていないときは判定しない（型の検査は schema が担う）。
    """
    triple = [values.get(n) for n in PRESET_ORDER_NAMES]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in triple):
        return None
    low, mid, high = triple
    if low < PRESET_MIN or mid < PRESET_MIN or high < PRESET_MIN:
        return (f"速度プリセットは {PRESET_MIN} 以上でなければならない"
                f"（低速={low}・中速={mid}・高速={high}）")
    if high > PRESET_MAX:
        return (f"速度プリセットの高速は天井 {PRESET_MAX * 100:.0f}% 以下でなければならない"
                f"（高速={high}）")
    if low > mid:
        return (f"速度プリセットは低速≦中速≦高速でなければならない"
                f"（低速={low} が中速={mid} を上回っている）")
    if mid > high:
        return (f"速度プリセットは低速≦中速≦高速でなければならない"
                f"（中速={mid} が高速={high} を上回っている）")
    return None
