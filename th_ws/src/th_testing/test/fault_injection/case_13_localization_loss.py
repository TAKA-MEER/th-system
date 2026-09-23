"""
case_13_localization_loss.py — 故障注入 13「自己位置喪失（SIGKILL）」
（ctest 登録名: fault_injection_13）
==========================================================================
`DetailedDesign-safety.md` §10 #13: `slam_toolbox` の推定プロセス
（`async_slam_toolbox_node`）を SIGKILL し、重大フォルト `LOCALIZATION_LOST`
→ `ESTOP` となることを確認する（WP-SAFE-05。自動化: Gazebo）。

────────────────────────────────────────────────────────────
【対象を有効化するためにこのパケットで行った本番launchの変更】
────────────────────────────────────────────────────────────
自己位置監視は「自己位置推定が起動するとき」だけ有効にする（Spec-safety.md
§3.5.0「使っていない間は監視しない」）。これは `slam_on`（SLAM）または
`map_yaml` 指定（AMCL）の起動ときにだけ安全監視が意味を持つためで、
`localization_health` の publish が `inactive`（監視外）になっていないことを
テスト側が事前確認する。`gazebo.launch.py` では

  - sim の `safety_monitor` をトップレベルから `_scenario_setup` の
    sim 分岐へ移し、`enabled_targets` の一時リストへ `slam_on or bool(map_v)`
    のときだけ `'localization'` を append する（`SAFETY_ENABLED_TARGETS_SIM`
    の固定リストそのものは変更しない。case_09 の静的読取が参照している）。
  - 同じ条件（`estimation_on`）のときだけ `localization_health.py` を
    `use_sim_time: True` 明示で起動する（sim は `/clock` 基準のため。
    渡さないと TF のスタンプとの比較が壊れる）。

トップレベルの `safety_sim_node` は「sim 分岐内のノードと二重起動になる」ため
撤去した。実機向け `safety_real_node` は不変（`bringup.launch.py` 側の
`localization_enabled` と同じ設計判断を実機側は持ち、sim 側だけをここで
揃えた）。

────────────────────────────────────────────────────────────
【検知時間上限の導出（このテストの判断基準。T-1: テストに数値を直書きしない）】
────────────────────────────────────────────────────────────
kill 後の最悪検知レイテンシは、registry / static の各パラメータから
次のように積算する（`_detect_upper_ms` 参照）:

  detection_ms = localization_stale_ms(=2000)
               + jump_window_ms(=500)        # health が publish する周期
               + critical_fault_hold_ms(=300)# safety_monitor の保持時間
               + 2×check_period_ms(=100)     # safety_monitor の監視周期の2倍
               + 500                          # DDS 配送ジッタの余裕
            ≈ 3500 ms

`localization_stale_ms` / `jump_window_ms` / `critical_fault_hold_ms` は
registry.yaml から（`limiter_param`）、`check_period_ms` は
`safety_monitor_sim.yaml` から読む。主経路は A（TF 停滞）: slam_toolbox は
`transform_publish_period: 0.02`（50Hz）で `map→odom` を配信し続けるため、
プロセスを SIGKILL すると TF が凍結し、spawn 静止状態でも確実に
`stale` 超過で検知される（mapping モードであることが前提）。

────────────────────────────────────────────────────────────
【ウォームアップ（localization_warmup_ms=30000）の論拠】
────────────────────────────────────────────────────────────
`localization_health` はノード構築時の sim 時刻（≈0〜数秒）から
`localization_warmup_ms`（30秒）の間、A・C・B′ を保留する。**実測より**、
`sim_stack` の準備完了（/scan 受信・/clock 開始）は sim 数秒で満たるため、
REPLAY 入り自体は sim ≈4 秒で済んでしまい、kill 時点がウォームアップ内
（0〜30秒）に落ちうる。ウォームアップ中は保留により ok=true のまま
（実測: transform_age_sec=2.38 > stale_ms=2.0 でも reason='' ok=true が残り
LOCALIZATION_LOST が立たなかった）。これに対し、kill **前**に
「health が発行したメッセージの header.stamp（sim 時刻）が
`localization_warmup_ms` を超えて 1 秒進む」まで明示的に待つ
（`_WARMUP_DONE_NS`）——これで保留は確実に終わり、kill 後に ng が
確実に出る。保留は false PASS にはならない（false FAIL 方向）が、待つ
ことで検知の起きない時間を原因の切り分けから排除する。

────────────────────────────────────────────────────────────
【IDLE 対照試験が必要な理由（変異②の捕捉）】
────────────────────────────────────────────────────────────
同一 launch が「IDLE（自律走行しない・監視外）では監視しない」ことを
壊す変異（`localization_health_core` の `is_localization_in_use` 常真化など）は、
REPLAY 側の主試験だけでは**検出できない**（主試験は「推定ありで監視が効く」
ことしか見ない）。`test_fault_injection_13_localization_loss_idle_contrast` は

  - IDLE のまま `async_slam_toolbox_node` を SIGKILL しても
    `LOCALIZATION_LOST` が **active にならない**こと、
  - ただし health は kill 後も `ok=true / reason=='inactive'` を出し続けた
    （＝監視が生きており、モードゲートで off になっていた「証拠」）こと

の両方を確認する。変異②が入ると「IDLE で inactive を出さない」または
「kill 後に LOCATION_LOST が active になる」のどちらかで即落ちる。
なお IDLE 対照は「推定が居ないのに監視ONだと node_down 誤発火する」方向の
変異（gazebo.launch.py の `estimation_on` 分岐破壊）も捕捉する——監視だけ
無条件 ON にして健康 publisher を出さない変異では、sim 起動直後の
`startup_grace_sec`（7秒）を過ぎると IDLE 到達後に stale/node_down で
`LOCALIZATION_LOST` が active になる。

────────────────────────────────────────────────────────────
【SIGKILL を使うテストについての既知の注意】
────────────────────────────────────────────────────────────
`case_11` と同じ（`kill -TERM` を既定にし、SIGKILL が要る 11・13 は毎回
コンテナを作り直す運用が前提）。`find_pids_by_exe_basename()` で argv[0] の
完全一致から PID を特定し、`os.kill(pid, SIGKILL)` する（`pkill -f` の
自滅誤爆を避ける）。`cleanup_dds_shm=True` を `sim_stack` に渡して後始末で
`/dev/shm/fastrtps_*` のアイドル分を掃除する。
"""
from __future__ import annotations

import os
import signal
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from th_system_msgs.msg import FaultStatus, LocalizationHealth, SystemState

from fault_injection.conftest import (
    TopicWatcher, _UiInputBootstrap, _pid_alive, _spin_until,
    find_pids_by_exe_basename,
)

_SRC_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..'))  # th_ws/src

# テストは sim_stack（slam 既定値=True・SLAM 起動）の上で動かす。
# SIGKILL を使うため後始末で /dev/shm の idle 分を掃除する（case_11 と同じ）。
_SIM_STACK_PARAM = {'cleanup_dds_shm': True}

# slam_toolbox の online_async_launch.py が起動する実行ファイルのベース名。
# gazebo.launch.py の SLAM 分岐は this include 経由なので、この名前で
# プロセスを特定できる（test_ekf_config.py:128 / prelaunch_guard.py:41 で確認）。
_SLAM_BASENAME = 'async_slam_toolbox_node'

# REPLAY（教示再生）は S-14・自己位置監視対象モード（Spec-safety.md §3.5.0
# の A/C/B′ 監視対象、Spec-webui.md の S-14 行）。mode_entry.yaml で IDLE から
# 入れる。入り口の guard は mode_entry.yaml の表のみ（guards.py
# `_mode_entry_allowed`）なので、画面はここで供給すれば良い。
_REPLAY_SCREEN_ID = 'S-14'
# 万一 REPLAY が拒否された場合の代替（すべて IDLE から入れる監視対象モード）。
_FALLBACK_MODES = ['REPLAY', 'PANEL_NAV', 'SUMMON', 'HOME_NAV']
_ESTOP_SCREEN_ID = 'S-14'

_IDLE_TIMEOUT_SEC = 60.0        # INIT→IDLE（connectivity_checker の link_ok 待ち）
_MODE_ENTRY_TIMEOUT_SEC = 30.0  # /system/trigger 受理後のモード遷移待ち
_HEALTH_ACTIVE_TIMEOUT_SEC = 15.0  # health が監視中(ok=true, reason!='inactive')になる待ち
# ウォームアップ（localization_warmup_ms。sim 30秒）が終わるまでに、health が
# 発行する 2Hz メッセージの header.stamp（sim 時刻）が境界を超えて 1 秒進む
# のを待つ上限。sim は実時間の約1倍速で進むため 90 秒もあれば十分。
_WARMUP_WAIT_TIMEOUT_SEC = 90.0
# warmup 境界に足す 1 秒（≈ jump_window_ms 2 周期分）。これで「保留が終わった
# 直後の最初の tick でもし異常なら ng が出たはず」の時間帯まで含めて待つ。
_WARMUP_DONE_PAD_NS = 1_000_000_000
_SLAM_DEATH_TIMEOUT_SEC = 5.0   # SIGKILL 後プロセスが消えることの一次確認

# 検知カスケード（LOCALIZATION_LOST → fault_lock → ESTOP）に追加でかける余裕。
_ESTOP_AFTER_FAULT_MS = 2500
# 対照試験で「監視が active なら必ずこの窓で検知される」はず、の追加余裕。
_CONTRAST_EXTRA_MS = 1500
_DDS_JITTER_MS = 500


def _sim_check_period_ms() -> int:
    """`safety_monitor_sim.yaml` の `check_period_ms`（safety_monitor の監視周期）。

    registry.yaml 管理のパラメータではない（static ファイルの値。conftest.py の
    `safety_monitor_sim_startup_grace_sec` と同じ流儀でファイルから読む）。
    """
    import yaml

    path = os.path.join(_SRC_ROOT, 'th_bringup', 'config', 'safety_monitor_sim.yaml')
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return int(data['safety_monitor']['ros__parameters']['check_period_ms'])


def _detect_upper_ms(get) -> int:
    """kill から LOCALIZATION_LOST が active になるまでの最悪レイテンシ。

    ファイル冒頭docstring「検知時間上限の導出」参照。registry の値は
    `limiter_param`（コールバック）で、static の監視周期は
    `_sim_check_period_ms()` でそれぞれ引く（数値をテストに直書きしない）。"""
    stale_ms = get('localization_stale_ms')
    window_ms = get('jump_window_ms')
    hold_ms = get('critical_fault_hold_ms')
    return (int(stale_ms) + int(window_ms) + int(hold_ms)
            + 2 * _sim_check_period_ms() + _DDS_JITTER_MS)


def _msg_sim_ns(msg) -> int:
    """health メッセージの header.stamp（sim 時刻）を ns に直す。"""
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _wait_mode(node, watcher: TopicWatcher, expected: str, timeout_sec: float) -> None:
    """`/system/state.mode` が `expected` になるまでスピンして待つ。"""
    deadline = time.monotonic() + timeout_sec
    ok = _spin_until(
        node,
        lambda: bool(watcher.records) and watcher.records[-1][1].mode == expected,
        deadline)
    last_mode = watcher.records[-1][1].mode if watcher.records else '受信なし'
    if not ok:
        pytest.fail(
            f"/system/state.mode が {timeout_sec}秒以内に {expected!r} に"
            f"ならなかった（直近: {last_mode!r}）")


def _request_mode(node, mode: str) -> tuple[bool, str]:
    """`/system/trigger` で `ui.enter_mode` を1回呼ぶ。`(accepted, reason)` を返す。"""
    from th_system_msgs.srv import UiTrigger

    cli = node.create_client(UiTrigger, '/system/trigger')
    try:
        if not cli.wait_for_service(timeout_sec=10.0):
            pytest.fail('/system/trigger サービスが見つからない '
                        '(state_manager が起動していない?)')
        req = UiTrigger.Request()
        req.trigger = 'ui.enter_mode'
        req.arg_json = f'{{"mode": "{mode}"}}'
        req.requester = 'fault_injection_13'
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            return False, '応答なし'
        if result.accepted:
            return True, ''
        return False, getattr(result, 'reject_reason_key', '')
    finally:
        node.destroy_client(cli)


def _enter_monitored_mode(node, state_watcher: TopicWatcher):
    """`IDLE` を待ち、監視対象モード（REPLAY を優先）へ実際に入れる。

    `_UiInputBootstrap`（`/safety/estop_hw` と `/ui/active_screen` の供給）は
    ここで作り、呼び出し側の `finally` で `bootstrap.stop()` してもらう。
    返り値は `(bootstrap, entered_mode)`。

    モードの入り口 guard が mode_entry.yaml の表のみ参照（guards.py）なため、
    表で IDLE から許可されているモードなら自由に入れる。REPLAY が万一拒否
    された場合に備えて代替候補を順に試す（どれも監視対象モード）。
    """
    bootstrap = _UiInputBootstrap(node, _REPLAY_SCREEN_ID, 'fault_injection_13')
    try:
        _wait_mode(node, state_watcher, 'IDLE', _IDLE_TIMEOUT_SEC)

        entered = None
        rejections: list[tuple[str, str]] = []
        for mode in _FALLBACK_MODES:
            accepted, reason = _request_mode(node, mode)
            if accepted:
                entered = mode
                break
            rejections.append((mode, reason))
        if entered is None:
            pytest.fail(
                f"IDLE からどの監視対象モード（{_FALLBACK_MODES}）にも入れなかった "
                f"(拒否履歴: {rejections})。mode_entry.yaml の IDLE からの"
                f"遷移表を見直す必要がある可能性。")
    except BaseException:
        bootstrap.stop()
        raise

    _wait_mode(node, state_watcher, entered, _MODE_ENTRY_TIMEOUT_SEC)
    return bootstrap, entered


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_13_localization_loss(
        sim_stack, ros_node, limiter_param,
        assert_fault_within, assert_true_within, assert_equals_within,
        assert_no_nonzero_after):
    """本体: REPLAY 中に slam 推定を SIGKILL → A検知 → LOCALIZATION_LOST → ESTOP。

    ファイル冒頭 docstring の手順・時間根拠を参照。ロボットは静止（スパウン
    位置のまま）でよく、走行を伴わない（SP 등 위치誤差は関与しない）。
    """
    detect_upper_ms = _detect_upper_ms(limiter_param)

    bootstrap = None
    state_watcher = TopicWatcher(ros_node, '/system/state', SystemState)
    health_watcher = TopicWatcher(
        ros_node, '/safety/localization_health', LocalizationHealth)
    cmd_watcher = TopicWatcher(ros_node, '/cmd_vel', Twist)
    fault_watcher = TopicWatcher(ros_node, '/safety/fault', FaultStatus)
    fault_lock_watcher = TopicWatcher(ros_node, '/safety/fault_lock', Bool)
    try:
        # ── 1) 監視対象モードへ入り、health が「監視中」になるのを待つ ──
        #        （ok=true かつ reason!='inactive'。inactive のまま進むと
        #        safety_monitor は何も検知せず、テストは「正しく通った」のではなく
        #        監視が off のまま空振りで合格することになるため必須の事前確認）。
        bootstrap, entered_mode = _enter_monitored_mode(ros_node, state_watcher)

        deadline = time.monotonic() + _HEALTH_ACTIVE_TIMEOUT_SEC
        ok = _spin_until(
            ros_node,
            lambda: any(m.ok and m.reason != 'inactive'
                        for _t, m in health_watcher.records),
            deadline)
        if not ok:
            last = (health_watcher.records[-1][1] if health_watcher.records
                    else '受信なし')
            pytest.fail(
                f"mode={entered_mode!r} に入ったのに/localization_health が"
                f"『監視中』(ok=false を報告しうる状態)にならなかった "
                f"(直近: {last!r})。reason='inactive' のままなら監視が offであり、"
                f"このテストは空振りで通る。gazebo.launch.py の enabled_targets "
                f"へ 'localization' が入っているか確認すること。")

        # ── 2) ウォームアップ（localization_warmup_ms）が終わるまで待つ ──
        #      ウォームアップ中は A・C・B′ が保留され、slam が死んでいても
        #      ok=true のまま（実測で failure を確認）。health のメッセージ
        #      header.stamp が sim 時刻なので、それが warmup 境界＋1秒を
        #      超えるまで待つ（ファイル冒頭docstring の論拠を参照）。
        warmup_done_ns = (int(limiter_param('localization_warmup_ms'))
                          * 1_000_000 + _WARMUP_DONE_PAD_NS)
        deadline = time.monotonic() + _WARMUP_WAIT_TIMEOUT_SEC
        ok = _spin_until(
            ros_node,
            lambda: any(_msg_sim_ns(m) >= warmup_done_ns
                        for _t, m in health_watcher.records),
            deadline)
        if not ok:
            last = (health_watcher.records[-1][1] if health_watcher.records
                    else '受信なし')
            pytest.fail(
                f"health の sim 時刻が warmup ({warmup_done_ns}ns) を超える"
                f"メッセージが {_WARMUP_WAIT_TIMEOUT_SEC}秒以内に届かなかった "
                f"(直近の stamp: {last.header.stamp if last else '受信なし'})。"
                f"ウォームアップ中に kill すると保留で ok=true のままになり、"
                f"LOCALIZATION_LOST が立たずに空振りすることに注意。")

        # ── 3) slam_toolbox の推定プロセスを一意に特定して SIGKILL ──
        pids = find_pids_by_exe_basename(_SLAM_BASENAME)
        assert len(pids) == 1, (
            f"async_slam_toolbox_node の PID が一意に特定できない: {pids} "
            "(0件=SLAMが起動していない、複数件=多重起動の可能性)")
        kill_time = time.monotonic()
        os.kill(pids[0], signal.SIGKILL)

        # SIGKILL が効いたことの一次確認（プロセスが本当に消えた）。
        slam_dead = _spin_until(
            ros_node,
            lambda: not _pid_alive(pids[0]),
            time.monotonic() + _SLAM_DEATH_TIMEOUT_SEC)
        assert slam_dead, (
            f"SIGKILL 後も async_slam_toolbox_node (pid={pids[0]}) が生存している")

        # ── 4) health が ng（ok=false）を出すことを先に確認 ──
        #        LOCALIZATION_LOST はこの ng 入力の継続でしか立たないため、
        #        これを先に観測してテスト経路が噛み合っている証明にする。
        deadline = time.monotonic() + detect_upper_ms / 1000.0
        ok = _spin_until(
            ros_node,
            lambda: any(t >= kill_time and not m.ok
                        for t, m in health_watcher.records),
            deadline)
        if not ok:
            last = (health_watcher.records[-1][1] if health_watcher.records
                    else '受信なし')
            pytest.fail(
                f"/safety/localization_health が {detect_upper_ms}ms 以内に "
                f"ok=false にならなかった（直近: {last!r}）。slam の map→odom "
                f"TF が止まったのに stale が立たない/立たないなら、use_sim_time の"
                f"取り扱いが誤っている疑いがある。")

        # ── 5) LOCALIZATION_LOST（重大）→ fault_lock → ESTOP → 駆動ゼロ ──
        assert_fault_within(fault_watcher, 'LOCALIZATION_LOST', ms=detect_upper_ms)
        assert_true_within(
            fault_lock_watcher, 'data', ms=detect_upper_ms + _ESTOP_AFTER_FAULT_MS)
        assert_equals_within(
            state_watcher, 'mode', 'ESTOP',
            ms=detect_upper_ms + _ESTOP_AFTER_FAULT_MS)
        # 死後に非ゼロ指令が再び現れないこと（自律系が別経路で走り出さない）。
        assert_no_nonzero_after(
            cmd_watcher, 'linear.x', since=kill_time,
            ms=detect_upper_ms + _ESTOP_AFTER_FAULT_MS)
    finally:
        if bootstrap is not None:
            bootstrap.stop()
        state_watcher.destroy()
        health_watcher.destroy()
        cmd_watcher.destroy()
        fault_watcher.destroy()
        fault_lock_watcher.destroy()


@pytest.mark.parametrize('sim_stack', [_SIM_STACK_PARAM], indirect=True)
def test_fault_injection_13_localization_loss_idle_contrast(sim_stack, ros_node, limiter_param):
    """対照: IDLE（自律走行しないモード）では SIGKILL しても検知されず、
    かつ health は kill 後も『監視外』を出し続けることを確認する。

    ファイル冒頭docstring「IDLE 対照試験が必要な理由」参照。変異②
    （`is_localization_in_use` 常真化など）が入ると fail する。
    """
    detect_upper_ms = _detect_upper_ms(limiter_param)

    bootstrap = None
    state_watcher = TopicWatcher(ros_node, '/system/state', SystemState)
    health_watcher = TopicWatcher(
        ros_node, '/safety/localization_health', LocalizationHealth)
    fault_watcher = TopicWatcher(ros_node, '/safety/fault', FaultStatus)
    try:
        bootstrap = _UiInputBootstrap(ros_node, _REPLAY_SCREEN_ID, 'fault_injection_13')
        _wait_mode(ros_node, state_watcher, 'IDLE', _IDLE_TIMEOUT_SEC)

        # IDLE では監視外（reason=='inactive'・ok=true）を報告し続けることを確認。
        deadline = time.monotonic() + _HEALTH_ACTIVE_TIMEOUT_SEC
        ok = _spin_until(
            ros_node,
            lambda: any(m.ok and m.reason == 'inactive'
                        for _t, m in health_watcher.records),
            deadline)
        if not ok:
            last = (health_watcher.records[-1][1] if health_watcher.records
                    else '受信なし')
            pytest.fail(
                f"IDLE 中に health が『監視外』(ok=true, reason=='inactive')を"
                f"報告しなかった（直近: {last!r}）。変異②（is_localization_in_use "
                f"常真化など）で監視がモード非依存になりうる。")

        pids = find_pids_by_exe_basename(_SLAM_BASENAME)
        assert len(pids) == 1, (
            f"async_slam_toolbox_node の PID が一意に特定できない: {pids}")
        kill_time = time.monotonic()
        os.kill(pids[0], signal.SIGKILL)

        # kill 後、検知上限＋余裕の間待ち、LOCALIZATION_LOST が active に
        # ならなかったことを確認する（変異②が入るとここで赤くなる）。
        contrast_ms = detect_upper_ms + _CONTRAST_EXTRA_MS
        deadline = time.monotonic() + contrast_ms / 1000.0
        _spin_until(ros_node, lambda: time.monotonic() >= deadline, deadline)
        fired = [(t, m.fault_type) for t, m in fault_watcher.records if m.active]
        assert not fired, (
            f"IDLE 中に SIGKILL したのにフォルトが active になった: {fired}")

        # 監視が「生きていたがゲートで off だった」ことの裏取り: kill 後にも
        # health は ok=true / reason=='inactive' を出し続ける（2Hz 想定）。
        ok = _spin_until(
            ros_node,
            lambda: any(t > kill_time and m.ok and m.reason == 'inactive'
                        for t, m in health_watcher.records),
            deadline)
        assert ok, (
            f"kill 後（since={kill_time:.3f}）に /safety/localization_health が"
            f"『監視外』を一度も出さなかった。監視ノード自体が死んでいるか、"
            f"モードゲートが暴走している可能性。")
    finally:
        if bootstrap is not None:
            bootstrap.stop()
        state_watcher.destroy()
        health_watcher.destroy()
        fault_watcher.destroy()


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])