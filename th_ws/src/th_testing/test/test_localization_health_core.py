"""test_localization_health_core.py — localization_health_core.evaluate() の純粋単体試験（WP-SAFE-05）。

ROS2 非依存（ノードを起動しない・最速）。Spec-safety.md §3.5.0 の A と C、
起動猶予、B を出さないことを検証する。
"""
import os

import pytest
import yaml

from th_state.localization_health_core import (
    MONITORED_MODES,
    PREP_MONITORED_STATES,
    REASON_INACTIVE,
    REASON_JUMP,
    REASON_NODE_DOWN,
    REASON_OK,
    REASON_RESTART_TIMEOUT,
    REASON_RESTARTING,
    REASON_STALE,
    HealthReport,
    Params,
    TransformSample,
    check_planned_restart,
    detect_jump,
    evaluate,
    is_localization_in_use,
)


def _params(**overrides):
    base = dict(
        stale_ms=2000,
        warmup_ms=30000,
        expected_nodes=("slam_toolbox", "amcl"),
        jump_window_ms=500,
        jump_translation_m=0.11,
        jump_rotation_rad=0.0175,
    )
    base.update(overrides)
    return Params(**base)


def _fresh_kwargs(**overrides):
    """健全な状態：ノード居る・変換は新しい・猶予過ぎ。"""
    kw = dict(
        now_ms=100_000,
        boot_ms=0,
        last_transform_ms=99_900,
        present_nodes=("slam_toolbox", "state_manager"),
    )
    kw.update(overrides)
    return kw


def test_fresh_transform_is_ok():
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs())
    assert r == HealthReport(ok=True, reason=REASON_OK,
                             node_present=True, transform_age_sec=0.1)


def test_stale_transform_is_ng_with_stale():
    """A: map→odom が stale_ms 更新されなければ ok=false/stale。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(last_transform_ms=90_000))
    assert r.ok is False
    assert r.reason == REASON_STALE
    assert r.node_present is True
    assert r.transform_age_sec == 10.0


def test_missing_node_is_ng_with_node_down():
    """C: 推定ノードが 1 つも居なければ ok=false/node_down。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(present_nodes=("state_manager",)))
    assert r.ok is False
    assert r.reason == REASON_NODE_DOWN
    assert r.node_present is False


def test_either_expected_node_counts_as_present():
    """amcl だけでも在席扱い（map_yaml 指定起動の休眠経路）。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(present_nodes=("amcl",)))
    assert r.ok is True
    assert r.node_present is True


def test_never_seen_transform_is_stale_after_warmup():
    """一度も見ていなければ猶予後は stale（inf は stale_ms を超える）。"""
    p = _params()
    r = evaluate(p=p, **_fresh_kwargs(last_transform_ms=None))
    assert r.ok is False
    assert r.reason == REASON_STALE


def test_warmup_reports_ok():
    """起動猶予の間はノード不在・変換なしでも ok（起動失敗は猶予後に捕まえる）。"""
    p = _params()
    r = evaluate(now_ms=5_000, boot_ms=0, last_transform_ms=None,
                 present_nodes=(), p=p)
    assert r.ok is True
    assert r.reason == REASON_OK


def test_warmup_expiry_enables_detection():
    """猶予が過ぎれば厳密に判定する（境界：warmup_ms ちょうどは猶予後）。"""
    p = _params(warmup_ms=30_000)
    ok_side = evaluate(now_ms=29_999, boot_ms=0, last_transform_ms=None,
                       present_nodes=(), p=p)
    assert ok_side.ok is True
    ng_side = evaluate(now_ms=30_000, boot_ms=0, last_transform_ms=None,
                       present_nodes=(), p=p)
    assert ng_side.ok is False
    assert ng_side.reason == REASON_NODE_DOWN


def test_boundary_stale_ms_is_ok():
    """stale_ms ちょうどはまだ ok（超えたら ng）。"""
    p = _params(stale_ms=2000)
    assert evaluate(p=p, **_fresh_kwargs(
        now_ms=100_000, last_transform_ms=98_000)).ok is True
    assert evaluate(p=p, **_fresh_kwargs(
        now_ms=100_000, last_transform_ms=97_999)).reason == REASON_STALE


def test_low_confidence_is_never_emitted():
    """B（low_confidence）は出さない。reason は 3 値のどれか。"""
    p = _params()
    cases = [
        _fresh_kwargs(),
        _fresh_kwargs(last_transform_ms=None),
        _fresh_kwargs(present_nodes=()),
        _fresh_kwargs(last_transform_ms=0),
    ]
    for kw in cases:
        r = evaluate(p=p, **kw)
        assert r.reason in (REASON_OK, REASON_STALE, REASON_NODE_DOWN)
        assert r.reason != "low_confidence"


# ============================================================================
# B′: detect_jump（WP-SAFE-05B）
# ============================================================================

def _sample(t_ms=100_000, x=0.0, y=0.0, yaw=0.0):
    return TransformSample(t_ms=t_ms, x=x, y=y, yaw=yaw)


def test_jump_fires_on_translation():
    """完了条件1（並進）: 比較周期のあいだに 0.11 m 超動いたら jump。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000), _sample(t_ms=100_500, x=0.2), p)
    assert r.is_jump is True
    assert r.trans_m == pytest.approx(0.2)
    assert r.rot_rad == pytest.approx(0.0)


def test_jump_fires_on_rotation():
    """完了条件1（回転）: 1°（0.0175 rad）超回ったら jump。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000), _sample(t_ms=100_500, yaw=0.05), p)
    assert r.is_jump is True
    assert r.trans_m == pytest.approx(0.0)
    assert r.rot_rad == pytest.approx(0.05)


def test_jump_fires_on_diagonal():
    """斜めの合成移動（hypot）で判定すること。x だけでは足りなくても組で超える。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000), _sample(t_ms=100_500, x=0.08, y=0.08), p)
    assert r.trans_m == pytest.approx(0.08 * 2 ** 0.5)
    assert r.is_jump is True   # 0.113 > 0.11


def test_jump_boundary_does_not_fire():
    """完了条件2: 閾値ぴったりでは出ない（`>` であり `>=` ではない）。
    浮動小数点の 1ulp を避けるため、2 進で正確な値（1.0）で境界を踏む。
    hypot(1.0, 0.0) は正確に 1.0 になる。"""
    p = _params(jump_translation_m=1.0, jump_rotation_rad=99.0)
    assert detect_jump(
        _sample(t_ms=100_000), _sample(t_ms=100_500, x=1.0), p).is_jump is False
    assert detect_jump(
        _sample(t_ms=100_000), _sample(t_ms=100_500, x=1.000001), p).is_jump is True


def test_jump_rotation_threshold_is_effective():
    """回転の閾値が効いている（deg/rad の取り違え等でずれていたら赤）。
    wrap の浮動小数点誤差（1ulp）を避けるため、閾値の前後で見る。"""
    p = _params()
    eps = 1e-9
    assert detect_jump(
        _sample(t_ms=100_000),
        _sample(t_ms=100_500, yaw=0.0175 * (1 - eps)), p).is_jump is False
    assert detect_jump(
        _sample(t_ms=100_000),
        _sample(t_ms=100_500, yaw=0.0175 * (1 + eps)), p).is_jump is True


def test_jump_yaw_wraps_around_pi():
    """±πまたぎ（3.14 → -3.14）は微小回転であり jump ではない。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000, yaw=3.14),
                    _sample(t_ms=100_500, yaw=-3.14), p)
    assert r.rot_rad == pytest.approx(0.0, abs=0.01)
    assert r.is_jump is False


def test_slow_continuous_correction_does_not_fire():
    """完了条件3: ゆっくりした連続補正では出ない（誤発火の本体）。
    毎 tick 0.02 m・0.003 rad ずつ 30 回（合計 0.6 m・0.09 rad）動かしても
    per-window では閾値未満なので ok のまま。"""
    p = _params()
    prev = _sample(t_ms=100_000)
    for i in range(1, 31):
        curr = _sample(t_ms=100_000 + i * 500, x=0.02 * i, yaw=0.003 * i)
        r = detect_jump(prev, curr, p)
        assert r.is_jump is False, f"tick {i} で誤発火: {r}"
        prev = curr


def test_first_sample_does_not_fire():
    """完了条件4: 初回（prev 無し）は出ない。"""
    p = _params()
    r = detect_jump(None, _sample(), p)
    assert r.is_jump is False


def test_gap_beyond_stale_does_not_fire():
    """不連続（時刻差が stale 超）は出さない。凍結→復帰の飛びは A の担当。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_000, x=0.0),
                    _sample(t_ms=103_000, x=5.0), p)
    assert r.is_jump is False


def test_backward_clock_does_not_fire():
    """時刻が戻っていたら出さない（時計の異常時は安全側）。"""
    p = _params()
    r = detect_jump(_sample(t_ms=100_500, x=0.0),
                    _sample(t_ms=100_000, x=5.0), p)
    assert r.is_jump is False


# ============================================================================
# O-e3: check_planned_restart（計画的な再起動の保留）
# ============================================================================

# 試験用の上限・猶予（短い値。registry の実値ではない。呼び出し側が渡す想定）。
_RESTART_MAX_MS = 9_000
_POST_RESTART_GRACE_MS = 2_000


def _restart(now_ms, restarting, true_since_ms, false_since_ms,
             restart_max_ms=_RESTART_MAX_MS,
             post_restart_grace_ms=_POST_RESTART_GRACE_MS):
    return check_planned_restart(
        now_ms, restarting, true_since_ms, false_since_ms,
        restart_max_ms, post_restart_grace_ms)


def test_restarting_is_ok_while_restarting():
    """再起動中は ng にならない（A・C・B′ を評価しない）。"""
    assert _restart(100_000, True, 99_000, None) == (True, REASON_RESTARTING)


def test_restart_timeout_after_max():
    """上限を超えたら restart_timeout（立て直しが終わらない本物の異常）。"""
    assert _restart(109_001, True, 100_000, None) == (False, REASON_RESTART_TIMEOUT)


def test_restart_max_boundary_is_still_restarting():
    """上限ちょうどはまだ保留（超えたら故障。stale と同じ向き）。"""
    assert _restart(109_000, True, 100_000, None) == (True, REASON_RESTARTING)


def test_post_restart_grace_is_ok():
    """再起動が終わって猶予以内は ok（最初の補正を待つ）。"""
    assert _restart(101_999, False, 90_000, 100_000) == (True, REASON_RESTARTING)


def test_post_restart_grace_expiry_resumes_detection():
    """猶予後は再び検知する（ちょうどは猶予明け＝通常評価＝None）。"""
    assert _restart(102_000, False, 90_000, 100_000) is None


def test_no_notice_detects_as_usual():
    """知らせが無いときは従来どおり検知する（node_down が出る）。
    変異「知らせが無いときに保留する」はここが赤くなる。"""
    assert _restart(100_000, None, None, None) is None


def test_plain_false_without_edge_detects_as_usual():
    """False だけ受信（再起動の edge なし）も従来どおり。起動時の
    false publish や重複 false で保留に入ってはいけない。"""
    assert _restart(100_000, False, None, None) is None


def test_duplicate_true_does_not_extend_restart():
    """重複 True で true_since が更新されない前提の確認: edge 時刻基準で
    上限が効く（ノードが edge でのみ記録することの裏付け）。"""
    # 同じ true_since のまま時間が進めば上限で切れる。
    assert _restart(109_001, True, 100_000, None) == (False, REASON_RESTART_TIMEOUT)


# ============================================================================
# WP-SAFE-05修正: is_localization_in_use（モードゲート）
# ============================================================================

def test_in_use_monitored_modes_whole_regardless_of_state():
    """監視する 4 モードはモード全体（状態は見ない）。"""
    assert MONITORED_MODES == {"REPLAY", "PANEL_NAV", "SUMMON", "HOME_NAV"}
    for mode in ("REPLAY", "PANEL_NAV", "SUMMON", "HOME_NAV"):
        for state in ("RUN", "NAV", "PAUSE", "ALIGN", "BLOCKED", "LOCALIZE",
                      "READY", "NONE", None):
            assert is_localization_in_use(mode, state) is True, (
                f"{mode}/{state} が監視対象にならない")


def test_in_use_prep_only_return():
    """PREP は RETURN のときだけ監視する。"""
    assert PREP_MONITORED_STATES == {"RETURN"}
    assert is_localization_in_use("PREP", "RETURN") is True
    for state in ("MAPPING", "REGISTER", "EDIT", "SAVED", "NONE", None):
        assert is_localization_in_use("PREP", state) is False, (
            f"PREP/{state} が監視対象になっている")


def test_in_use_others_not_monitored():
    """それ以外は監視しない（IDLE／MANUAL／CARRY／教示／停止中／点検／
    校正／起動中／ESTOP…）。"""
    cases = [
        ("IDLE", "NONE"),
        ("MANUAL", "RUN"),
        ("MANUAL", "PAUSE"),
        ("CARRY", "NONE"),
        ("TEACH_FOLLOW", "REC"),
        ("TEACH_MANUAL", "REC"),
        ("AT_PANEL", "IDLE_P"),
        ("AT_HOME", "IDLE_H"),
        ("OPCHECK", "LIST"),
        ("CALIB", "LIST"),
        ("INIT", "CHECK"),
        ("ESTOP", "NONE"),
        ("FOLLOW", "RUN"),
        ("LINE", "RUN"),
        ("LEASH", "RUN"),
    ]
    for mode, state in cases:
        assert is_localization_in_use(mode, state) is False, (
            f"{mode}/{state} が監視対象になっている")


def test_in_use_none_means_not_monitored():
    """/system/state 未受信（None）は監視しない（起動中は INIT なので同じ）。"""
    assert is_localization_in_use(None, None) is False
    assert is_localization_in_use(None, "RUN") is False
    assert is_localization_in_use("IDLE", None) is False


def test_in_use_estop_follows_prev_mode():
    """非常停止中は止まる直前のモードに従う（Spec-safety.md §3.5.0）。
    監視するモードで見失って ESTOP に入ったなら、推定が戻るまで解除しない。"""
    assert is_localization_in_use("ESTOP", "NONE", "REPLAY", "RUN") is True
    assert is_localization_in_use("ESTOP", "NONE", "PANEL_NAV", "NAV") is True
    assert is_localization_in_use("ESTOP", "NONE", "PREP", "RETURN") is True
    assert is_localization_in_use("ESTOP", "NONE", "PREP", "MAPPING") is False
    assert is_localization_in_use("ESTOP", "NONE", "IDLE", "NONE") is False
    assert is_localization_in_use("ESTOP", "NONE", "", "") is False
    assert is_localization_in_use("ESTOP", "NONE", None, None) is False
    assert is_localization_in_use("ESTOP", "NONE") is False


def test_in_use_carry_ignores_prev_mode():
    """手押し（CARRY）中は機体を人が動かすので監視しない（prev を見ない）。"""
    assert is_localization_in_use("CARRY", "NONE", "REPLAY", "RUN") is False


def test_inactive_reason_constant():
    """監視外の理由コードは "inactive"（msg コメントが正）。"""
    assert REASON_INACTIVE == "inactive"


def test_mode_state_names_exist_in_config():
    """表のモード名・状態名が実在すること（打ち間違いで監視が黙って切れるのを
    防ぐ）。attributes.yaml のモードキーと、transitions.yaml の PREP/RETURN を
    照合する。"""
    repo_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    with open(os.path.join(repo_src, "th_state", "config", "attributes.yaml"),
              encoding="utf-8") as f:
        attributes = yaml.safe_load(f)
    for mode in ("REPLAY", "PANEL_NAV", "SUMMON", "HOME_NAV", "PREP",
                 "IDLE", "MANUAL", "AT_PANEL", "AT_HOME"):
        assert mode in attributes, f"attributes.yaml にモード {mode} が無い"
    assert "RETURN" in (attributes["PREP"].get("prep_states") or []), (
        "attributes.yaml の PREP prep_states に RETURN が無い")

    with open(os.path.join(repo_src, "th_state", "config", "transitions.yaml"),
              encoding="utf-8") as f:
        transitions = yaml.safe_load(f)
    prep_states: set[str] = set()
    for row in transitions:
        if row.get("mode") != "PREP":
            continue
        state = row.get("state")
        if isinstance(state, list):
            prep_states.update(state)
        elif isinstance(state, str):
            prep_states.add(state)
    assert "RETURN" in prep_states, (
        "transitions.yaml に PREP の RETURN 状態が無い")
