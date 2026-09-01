"""zones.py — 画面からゾーンを導く純粋関数（WP-STATE-02）。

`rclpy` を import しない（`state_core.py` / `guards.py` と同じ制約。CLAUDE.md／
DetailedDesign-state.md §1）。`th_testing` から直接 pytest できること。

DetailedDesign-names.md §4.1 が「唯一の正」と明言している `derive_limits()` の実装。
DetailedDesign-state.md §6 も「ここに擬似コードを再掲しない」としたうえで同じ関数を指すので、
実装はここ 1 箇所だけに置く（`jog_gate` と `obstacle_limiter` はこの実装を将来 import する）。

数値リテラルを書かない（R2）。`ui_active_window_s` は呼び出し側（ノード）が
ROS2 パラメータとして受け取り、引数で渡す。画面 ID・ゾーン名・速度上限パラメータ名は
DetailedDesign-names.md §4 の表の転記であり、文字列の集合なので R2 の対象外
（state_core.py の MODES 集合と同じ扱い）。

`mode_speed_limit()` / `combine_speed_limits()`（N-15）: DetailedDesign-safety.md §3.3.1 の
`applied_limit_mps = min(画面由来の上限, モード由来の上限, v_reverse)` のうち、画面由来と
モード由来の合成を担う。モード由来は `attributes.yaml`（`th_state` の config）の
`speed_limit` から決まるので、ここに置くのが妥当（`LIMIT_STRICTNESS` を再利用できる）。

`LIMIT_STRICTNESS`（N-15 でレビュー指摘を受けて `_LIMIT_STRICTNESS` から改名。先頭 `_` を外した）:
`th_params` の `assertions.a13_speed_limit_strictness_order()` がこの並びと
`registry.yaml` の解決済みの数値を突き合わせる（`th_params/th_params/export.py`）ため、
パッケージをまたいで参照される公開シンボルになった。`th_params/package.xml` に
`<depend>th_state</depend>` を追加してある。
"""
from dataclasses import dataclass
from typing import Dict, Mapping

# ============================================================
# DetailedDesign-names.md §4 — 画面（15）とゾーン・速度上限の表そのもの。
# ============================================================
SCREEN_ZONE: Dict[str, str] = {
    "S-00": "NA",
    "S-01": "OUT",
    "S-10": "OUT",
    "S-11": "OUT",
    "S-12": "OUT",
    "S-13": "OUT",
    "S-14": "OUT",
    "S-15": "OUT",
    "S-16": "OUT",
    "S-20": "IN",
    "S-21": "IN",
    "S-30": "NA",
    "S-31": "NA",
    "S-40": "NA",
    "S-50": "OUT",
}

SCREEN_SPEED_LIMIT: Dict[str, str] = {
    "S-00": "stop",
    "S-01": "stop",
    "S-10": "v_max",
    "S-11": "v_max",
    "S-12": "v_max",
    "S-13": "v_max",
    "S-14": "v_max",
    "S-15": "v_max",
    "S-16": "v_leash",
    "S-20": "v_slow",
    "S-21": "v_slow",
    "S-30": "v_check",
    "S-31": "stop",
    "S-40": "v_calib",
    "S-50": "stop",
}

assert set(SCREEN_ZONE.keys()) == set(SCREEN_SPEED_LIMIT.keys())

# 3値の優先順位（zone の合成専用。速度上限の合成には使わない。§4.1 の禁止事項）。
# 複数端末が異なるゾーンの画面を同時に使用中のときだけ効く（通常運用は単一画面が大半）。
_ZONE_PRIORITY = ("IN", "NA", "OUT")

# speed_limit 名同士の「厳しさ」順（左ほど厳しい）。実際の数値は registry.yaml 側の責務だが、
# この並び自体は「registry.yaml の解決済みの数値の昇順と一致していること」が前提
# （このリストは数値の代理だから。stop は registry に行が無いが 0 として扱う）。
# 前提が崩れると combine_speed_limits() / derive_limits() が実際より高い（緩い）上限を
# 返しかねないため、`th_params.assertions.a13_speed_limit_strictness_order()` が
# registry.yaml の解決済みの値でこの並びを起動時に検査する（未測定＝placeholder の名前は
# 比較から除外。DetailedDesign-open.md N-15 のレビューで追加）。
#
# v_jog_panel（N-15 で追加）: registry.yaml の A5 は `v_jog_panel ≤ v_reverse ≤ v_slow ≤ v_max`
# を保証するので `v_slow` より厳しい（左）側にあることは確定している。`v_check`(0.05m/s)・
# `v_calib`(0.15m/s) との相対順は A5 の対象外で、`panel_clearance_m`（v_jog_panel の逆算元）は
# 現状 placeholder（未測定）。**当初 `stop` の直後に置いたが誤りだった**——このリストは
# 用途の重大さの順ではなく数値の代理であり、`panel_clearance_m` に現実的な値
# （例: 0.3m 前後）を入れて逆算すると v_jog_panel は 0.05 よりかなり大きくなる見込みで、
# `v_check` より前（厳しい側）に置くと合成結果が真の最小値より高くなる（危険側）。
# `v_calib`(0.15) と `v_slow` の間に置いた。measured 値が入って本当に想定と違えば
# 上記の A13 が起動を拒否するので、この位置自体が事故を起こすことはない。
LIMIT_STRICTNESS = (
    "stop", "v_check", "v_calib", "v_jog_panel", "v_slow", "v_leash", "v_max")


@dataclass(frozen=True)
class ScreenInput:
    """`/ui/active_screen` の1端末ぶんのスナップショット。"""
    screen_id: str
    interacting: bool
    last_input_ms: int


@dataclass(frozen=True)
class Limits:
    zone: str            # "IN" / "OUT" / "NA"
    speed_limit: str      # パラメータ名 or "stop"
    auto_brake: bool


def derive_limits(screens: Mapping[str, ScreenInput], now_ms: int,
                   ui_active_window_s: int) -> Limits:
    """DetailedDesign-names.md §4.1 の `derive_limits()`。

    「使用中」= interacting かつ最後の操作から ui_active_window_s 以内
    （Spec-safety.md §6.2 と同一定義）。途絶・使用中 0 台はフェイルセーフで
    zone=NA・speed_limit=stop・auto_brake=True に倒す。
    """
    window_ms = ui_active_window_s * 1000
    active = [s for s in screens.values()
              if s.interacting and now_ms - s.last_input_ms <= window_ms]

    if not active:
        return Limits(zone="NA", speed_limit="stop", auto_brake=True)

    zones_present = {SCREEN_ZONE[s.screen_id] for s in active}
    zone = next(z for z in _ZONE_PRIORITY if z in zones_present)

    # 速度上限は「各画面の上限の最小値」（§4.1）。名前の厳しさ順で最も厳しいものを採る。
    limits_present = {SCREEN_SPEED_LIMIT[s.screen_id] for s in active}
    speed_limit = next(l for l in LIMIT_STRICTNESS if l in limits_present)

    auto_brake = zone != "OUT"
    return Limits(zone=zone, speed_limit=speed_limit, auto_brake=auto_brake)


def mode_speed_limit(mode_attrs: Mapping[str, object], jog_active: bool) -> str:
    """`attributes.yaml` の該当モード行から求める「モード由来」の速度上限（N-15）。

    DetailedDesign-safety.md §3.3.1 の 2 系統のうちモード側。呼び出し側は
    `attributes.yaml` を読み込んだ辞書からそのモードの行（例: `attrs["AT_PANEL"]`）を
    渡すこと（`th_state.state_core.StateCore.attributes(mode)` が返す形と同じ）。

    attributes.yaml 冒頭コメント（※※※）: 「speed_limit は停止だが、jog_active の間だけ
    v_jog_panel」という行が AT_PANEL に 1 つある。ここではモード名を比較せず
    （CLAUDE.md N-1 と同種の理由。将来 speed_limit=stop かつ jog 許可のモードが増えても
    同じ規則で動くように）、`speed_limit == "stop"` かつ `jog` が `denied` でない
    （= このモードはそもそも jog できる）かつ `jog_active` の 3 条件で判定する。
    現状の attributes.yaml でこの3条件をすべて満たすのは AT_PANEL だけ。
    """
    limit = mode_attrs["speed_limit"]
    if limit == "stop" and jog_active and mode_attrs.get("jog") != "denied":
        return "v_jog_panel"
    return limit


def combine_speed_limits(screen_limit: str, mode_limit: str) -> str:
    """画面由来とモード由来のうち厳しい方を返す（DetailedDesign-safety.md §3.3.1）。

    `LIMIT_STRICTNESS` の名前の順序で比較する（数値では比較しない）。
    """
    return next(l for l in LIMIT_STRICTNESS if l in (screen_limit, mode_limit))
