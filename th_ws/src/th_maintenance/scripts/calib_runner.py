#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校正の実行部 `calib_runner`（WP-MAINT-02 / パケット 9-b）。

`/system/state` が `CALIB` のときだけ動くノード（設計書 §1「ノード側でも二重に確認」）。
進行は FSM（`state_manager`）が出す `/system/effect` の自分宛て effect で進み、
走行・実測の完了と検証の合否は `evt.calib_step_done` / `evt.calib_verify_ng` を
`/system/event` へ返す（T-CAL-01〜09）。

**始業点検（`opcheck_runner`）との違い**: 校正は**自律で走る・回る**（設計書 §1・Spec-checks.md §1）。
押している間だけ動くデッドマンではなく、止める手段は次のとおり。

  - 中断（`ui.abort` → `discard_calib`）・フォルト（`fault.recoverable` → `discard_calib`）
  - `/system/state` が `CALIB` でなくなる（非常停止・IDLE への強制遷移を含む）→ 即停止・適用前へ戻す
  - 物理／UI 非常停止の直接購読
  - `/system/state` が一定時間来なくなったら止める（`_STATE_STALE_S`）
  - `/odom` が途絶したら止める（自分の位置が分からないまま走らせない）

`BLIND`（LiDAR 死角マスク）は走らない。画面で選んだ角度帯（`/calib/submit` の `ranges`）を、
**幅の上限（`blind_max_*`）を検査してから** `obstacle_limiter`・`lidar_filter`・`opcheck_runner` の
3 ノードへ同時にランタイム反映し（1 つでも失敗すれば全部を適用前へ戻す）、適用後の `/scan` から
写り込みを推定して、選んだ範囲の外に残るずれが `calib_blind_tolerance_deg` 以内かを検証する。
`/scan_filtered` に実際にマスクが効いていること（選んだ範囲の内側が inf）も確かめる。確定は検証合格のときだけ。
正本は `calib/current.yaml`（起動時に `params_generation` が生成 yaml へ重ねる）。

動く項目は `LINEAR`（直進）と `ROTATION`（旋回）。速度は `v_calib`、出力先は
**`/cmd_vel_behavior`**（`/cmd_vel` へは出さない・`/cmd_vel_manual` も使わない）。
`IMU` は人が機体を 8 の字に動かすので走らせない（`/esp32/imu_calib_status` を見るだけ）。

適用先:
  - `LINEAR`   → esp32_bridge の `wheel_radius_scale`（§4。**A10 を超える値は適用しない**）
  - `ROTATION` → esp32_bridge の `wheel_base`
  どちらもランタイムの `set_parameters`（再起動不要）。**確定（`commit_calib`）まで永続化しない**。
  確定前に abort / フォルト / 非常停止 / 検証 NG になったら適用前の値へ戻す。

## 決めたこと（設計書に無い・曖昧だった点。報告に列挙する）
  - `/calib/start` は **再走行専用**（検証 NG で S2 に戻った後・走行失敗後）。通常の起動は
    `run_measurement` effect。二重起動は phase で拒否する。
  - `/calib/apply` は状態を進めない**冪等な確認**（適用と検証走行が進行中なら success）。
    進めるのは FSM（`ui.calib_next` → `apply_and_verify`）。
  - 履歴は項目ごと（`calib_store`）。許容範囲が placeholder の間は検証を**合格にしない**。
  - 起動時に `current.yaml` の値を esp32_bridge へ反映する（A10 を通ったものだけ）。
"""

import json
import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, UInt8
    from th_system_msgs.msg import CalibStatus, StateEffect, StateEvent, SystemState
    from th_system_msgs.srv import ApplyCalib, RollbackCalib, StartCalib, SubmitCalib
    RS_OK = True
except ImportError:
    # ホスト pytest（rclpy 無し）から PARS 等の純粋な定義だけを import できるようにする。
    rclpy = None
    Node = object
    RS_OK = False

from th_maintenance import blind_core, calib_core
from th_maintenance.check_core import BLIND_MIN_RUN_DEG, BLIND_NEAR_M
from th_maintenance.calib_store import CalibStore

# registry.yaml と一致させるパラメータの既定値（consumers に calib_runner を持つ行のうち値あり）。
PARS: dict = {
    "v_calib": 0.15,
    "wheel_radius_scale_max_dev": 0.10,
    # 死角マスクの幅の上限（registry の blind_max_*。obstacle_limiter も同じ値で独立に検査する）
    "blind_max_sector_deg": blind_core.MAX_SECTOR_DEG,
    "blind_max_total_deg": blind_core.MAX_TOTAL_DEG,
    "blind_max_sectors": blind_core.MAX_SECTORS,
}

# registry 未登録のノード局所の既定値（names.md §7 に名前が無いため registry へ足していない。
# 校正の「指定距離・指定角度」と走行の安全タイムアウト。値の正式な置き場は実装管理担当が決める）。
LOCAL_PARS: dict = {
    "calib_linear_distance_m": 1.0,
    "calib_rotation_deg": 180.0,
    "calib_run_timeout_s": 60.0,
    # BLIND の検証: 恒常的な写り込みを推定するために集める /scan のフレーム数と、待つ上限。
    "calib_blind_verify_frames": 15.0,
    "calib_blind_verify_timeout_s": 10.0,
}

# BLIND の反映先。obstacle_limiter（安全判定。上限を独立検査して拒否できる）を先頭にする。
BLIND_TARGET_NODES = ("/obstacle_limiter", "/lidar_filter", "/opcheck_runner")

# 許容範囲は未確定（registry は status: placeholder・生成 YAML には載らない）。
# 既定の -1.0 は「未確定」の意味で、検証は合格にならない（calib_core.verify_*）。
TOLERANCE_UNDEFINED = -1.0

ITEM_VALID = calib_core.ITEMS
MOTION_ITEMS = ("LINEAR", "ROTATION")
RUNTIME_ITEMS = MOTION_ITEMS + ("BLIND",)    # ランタイム反映と適用前への復帰が要る項目

# 実行 phase
IDLE = "IDLE"
GUIDE = "GUIDE"                  # begin_wizard 済み。S1 案内中
RUNNING = "RUNNING"              # 測定のための自律走行／IMU 監視
WAIT_MEASURED = "WAIT_MEASURED"  # 走行完了。実測値の入力待ち（S3）
PREVIEW = "PREVIEW"              # プレビューが sane。ui.calib_next 待ち
APPLYING = "APPLYING"            # 適用（ランタイム反映）中
VERIFY_RUNNING = "VERIFY_RUNNING"
VERIFY_WAIT = "VERIFY_WAIT"      # 検証走行完了。実測値の入力待ち（S4）
VERIFIED = "VERIFIED"            # 検証合格。commit_calib 待ち
RETRY_WAIT = "RETRY_WAIT"        # 検証 NG／走行失敗で S2 に戻った。/calib/start 待ち

_MOVING_PHASES = (RUNNING, VERIFY_RUNNING)

_STATE_STALE_S = 2.0     # /system/state がこれだけ来なかったら止める（state_manager は 10Hz）
_ODOM_STALE_S = 0.5      # /odom がこれだけ来なかったら止める
_CTRL_PERIOD_S = 0.05    # 20Hz
_STATUS_PERIOD_S = 0.5

_TAPER_DIST_M = 0.10     # 終端でこの距離から減速
_TAPER_ANGLE_RAD = math.radians(15.0)
_MIN_LINEAR_MPS = 0.04
_MIN_ANGULAR_RADS = 0.15
_MAX_ANGULAR_RADS = 1.0


def _json_str(text: str) -> str:
    return text or "{}"


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class CalibRunner(Node):
    """校正の実行部。CALIB のときだけ動作する。"""

    def __init__(self):
        if not RS_OK:
            raise ImportError("rclpy が無い環境で CalibRunner を実体化した")
        super().__init__("calib_runner")
        for name, value in {**PARS, **LOCAL_PARS}.items():
            self.declare_parameter(name, value)
        self.declare_parameter("calib_linear_tolerance_ratio", TOLERANCE_UNDEFINED)
        self.declare_parameter("calib_rotation_tolerance_deg", TOLERANCE_UNDEFINED)
        self.declare_parameter("calib_blind_tolerance_deg", TOLERANCE_UNDEFINED)
        # 起動時の死角マスク（生成 yaml。確定済みの校正値で上書き済み）。BLIND の「適用前」の値。
        self.declare_parameter("blind_angle_ranges", [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("blind_target_nodes", list(BLIND_TARGET_NODES))
        self.declare_parameter("calib_dir", "/root/th_data/calib")
        self.declare_parameter("params_digest_path", "/root/th_data/generated/params_digest.json")
        # esp32_bridge の params.yaml の既定値（校正済みなら current.yaml が優先される）。
        self.declare_parameter("nominal_wheel_base_m", 0.39)
        self.declare_parameter("bridge_node", "/esp32_bridge")
        self.declare_parameter("calib_operator", "webui")

        self._store = CalibStore(str(self.get_parameter("calib_dir").value))

        # ── 内部状態 ──────────────────────────────────────
        self._mode = "INIT"
        self._state = "NONE"
        self._state_last_s = None
        self._estop_ui = False
        self._estop_hw = False
        self._estop_raw = False

        self._item = None
        self._phase = IDLE
        self._reason = ""
        self._applied = False            # ランタイムに未確定の値を反映した（commit 前）
        self._pending = None             # {param: value}（S3 で承認待ち／適用した値）
        self._revert_to = None           # 適用前のランタイム値
        self._measured = None
        self._verify_measured = None
        self._operator = ""
        self._preview = ("", "")
        self._preview_sane = False
        # 直近の /calib/submit が sane でなかった（次の submit か項目の離脱まで、周期配信でも
        # result=PREVIEW_INSANE を出し続ける。S-40 が「補正が大きすぎます」を出し続けるため）。
        self._preview_insane = False

        # 走行
        self._cmd_active = False
        self._run_t0 = 0.0
        self._odom_last_s = None
        self._odom = None                # (x, y, yaw)
        self._run_start = None           # (x, y)
        self._run_yaw_acc = 0.0
        self._run_yaw_prev = None
        self._run_progress = 0.0

        # IMU
        self._imu_calib = 0

        # BLIND
        self._scan = None                # 最新の生スキャン (angle_min_rad, inc_rad, ranges, t)
        self._scan_filtered = None       # 最新の /scan_filtered (同上)
        self._verify_frames = []         # 検証中に集めた生スキャンの ranges
        self._verify_t0 = 0.0
        self._verify_applied_s = 0.0     # 3 ノードへの反映が終わった時刻

        self._startup_applied = False

        # ── QoS ───────────────────────────────────────────
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        effect_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                               history=QoSHistoryPolicy.KEEP_LAST)
        status_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        cmd_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)

        # ── Subscribers ───────────────────────────────────
        self.create_subscription(SystemState, "/system/state", self._on_state, state_qos)
        self.create_subscription(StateEffect, "/system/effect", self._on_effect, effect_qos)
        self.create_subscription(Bool, "/safety/estop_hw", self._on_estop_hw, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(UInt8, "/esp32/imu_calib_status", self._on_imu_calib, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_filtered", self._on_scan_filtered,
                                 qos_profile_sensor_data)

        # ── Publishers ────────────────────────────────────
        self._pub_status = self.create_publisher(CalibStatus, "/calib/status", status_qos)
        self._pub_event = self.create_publisher(StateEvent, "/system/event", event_qos)
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel_behavior", cmd_qos)

        # ── Services ──────────────────────────────────────
        self.create_service(StartCalib, "/calib/start", self._on_start)
        self.create_service(SubmitCalib, "/calib/submit", self._on_submit)
        self.create_service(ApplyCalib, "/calib/apply", self._on_apply)
        self.create_service(RollbackCalib, "/calib/rollback", self._on_rollback)

        # ── esp32_bridge のパラメータ反映 ─────────────────
        bridge = str(self.get_parameter("bridge_node").value).rstrip("/")
        self._cli_set = self.create_client(SetParameters, f"{bridge}/set_parameters")
        self._blind_clients = {
            str(n).rstrip("/"): self.create_client(SetParameters, f"{str(n).rstrip('/')}/set_parameters")
            for n in self.get_parameter("blind_target_nodes").value}

        # ── Timers ────────────────────────────────────────
        self.create_timer(_CTRL_PERIOD_S, self._ctrl_tick)
        self.create_timer(_STATUS_PERIOD_S, self._status_tick)
        self.create_timer(1.0, self._startup_apply_tick)

        self.get_logger().info(
            f"calib_runner 起動（v_calib={self._p('v_calib')}, "
            f"calib_dir={self.get_parameter('calib_dir').value}）")

    # ── ユーティリティ ───────────────────────────────────
    def _p(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _in_calib(self) -> bool:
        """CALIB のときだけ受け付ける共通ゲート（effect・サービスの入口が全てここを通る）。"""
        return self._mode == "CALIB"

    def _estopped(self) -> bool:
        return self._estop_ui or self._estop_hw or self._estop_raw

    @staticmethod
    def _tolerance(value: float):
        return value if value > 0.0 else None

    # ── /system/state ────────────────────────────────────
    def _on_state(self, msg: SystemState):
        self._mode = msg.mode
        self._state = msg.state
        self._state_last_s = self._now_s()
        self._estop_ui = msg.estop_ui
        self._estop_hw = msg.estop_hw
        if self._mode != "CALIB":
            if self._phase != IDLE or self._cmd_active:
                self.get_logger().warn(
                    f"CALIB を離脱（{self._mode}）→ 校正を中断し適用前へ戻す（phase={self._phase}）")
            self._abort_all("left_calib")

    def _on_estop_hw(self, msg: Bool):
        self._estop_raw = bool(msg.data)
        if self._estop_raw:
            self._abort_all("estop")

    # ── /system/effect ───────────────────────────────────
    def _on_effect(self, msg: StateEffect):
        if msg.dest != "calib_runner":
            return
        args = json.loads(_json_str(msg.args_json))
        item = str(args.get("item") or "")
        name = msg.name
        if name == "begin_wizard":
            self._begin_wizard(item)
        elif name == "run_measurement":
            self._start_run(item, verify=False)
        elif name == "build_preview":
            self.get_logger().info(f"build_preview({item}) → 実測値の入力待ち")
            self._publish_status()
        elif name == "apply_and_verify":
            self._apply_and_verify(item)
        elif name == "commit_calib":
            self._commit(item)
        elif name == "revert_calib":
            self._on_revert_effect(item)
        elif name == "discard_calib":
            self._discard(item)
        else:
            self.get_logger().debug(f"無視する effect: {name}")

    # ── 項目の開始 ───────────────────────────────────────
    def _begin_wizard(self, item: str):
        if not self._in_calib():
            self.get_logger().warn(f"CALIB 以外（{self._mode}）では begin_wizard を無視")
            return
        if item not in ITEM_VALID:
            self.get_logger().warn(f"不明な項目: {item}")
            return
        self._abort_all("restart_wizard")
        self._item = item
        self._phase = GUIDE
        self._reason = ""
        self._measured = None
        self._pending = None
        self._preview = ("", "")
        self._preview_sane = False
        self._preview_insane = False
        self.get_logger().info(f"校正開始（案内）: {item}")
        self._publish_status()

    def _commanded(self, item: str) -> float:
        if item == "LINEAR":
            return self._p("calib_linear_distance_m")
        if item == "ROTATION":
            return self._p("calib_rotation_deg")
        return 0.0

    def _current_value(self, item: str) -> dict:
        """いまの確定済みの補正値（保存値 → 公称）。未確定の適用中の値は含まない。"""
        entry = self._store.current_entry(item)
        if entry and isinstance(entry.get("values"), dict):
            return dict(entry["values"])
        if item == "LINEAR":
            return {"wheel_radius_scale": 1.0}
        if item == "ROTATION":
            return {"wheel_base": self._p("nominal_wheel_base_m")}
        if item == "BLIND":
            return {"blind_angle_ranges": self._startup_blind_flat()}
        return {}

    def _startup_blind_flat(self) -> list:
        """起動時の死角マスク（出荷値、または確定済みの校正値で上書きされた生成 yaml の値）。"""
        value = self.get_parameter("blind_angle_ranges").value
        return [float(v) for v in (value or [])]

    def _start_run(self, item: str, verify: bool) -> bool:
        if not self._in_calib():
            self.get_logger().warn(f"CALIB 以外（{self._mode}）では走行を開始しない")
            return False
        if item != self._item or item not in ITEM_VALID:
            self.get_logger().warn(f"run_measurement({item}) を無視（実行中の項目={self._item}）")
            return False
        allowed = (APPLYING,) if verify else (GUIDE, RETRY_WAIT)
        if self._phase not in allowed:
            self.get_logger().warn(f"phase={self._phase} では走行を開始しない（二重起動の拒否）")
            return False
        if self._estopped():
            self.get_logger().warn("非常停止中は走行を開始しない")
            return False
        self._reason = ""
        if not verify:
            self._pending = None
            self._preview = ("", "")
            self._preview_sane = False
            self._preview_insane = False
            self._measured = None
        if item == "IMU":
            self._phase = RUNNING
            self._publish_status()
            self._check_imu_done()
            return True
        if item == "BLIND":
            # 走らない。S2 で画面の選択（/calib/submit の ranges）を待つ。
            self._phase = RUNNING
            self._publish_status()
            return True
        # LINEAR / ROTATION: 自分の位置（/odom）が無いと走らせない
        if not self._odom_fresh():
            self._run_failed("no_odom")
            return False
        self._run_start = (self._odom[0], self._odom[1])
        self._run_yaw_prev = self._odom[2]
        self._run_yaw_acc = 0.0
        self._run_progress = 0.0
        self._run_t0 = self._now_s()
        self._phase = VERIFY_RUNNING if verify else RUNNING
        self.get_logger().info(f"自律走行を開始: {item}（verify={verify}）")
        self._publish_status()
        return True

    # ── 自律走行（20Hz）─────────────────────────────────
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._odom = (p.x, p.y, yaw)
        self._odom_last_s = self._now_s()
        if self._phase in _MOVING_PHASES and self._run_yaw_prev is not None:
            self._run_yaw_acc += _wrap(yaw - self._run_yaw_prev)
            self._run_yaw_prev = yaw

    def _odom_fresh(self) -> bool:
        return (self._odom is not None and self._odom_last_s is not None
                and self._now_s() - self._odom_last_s <= _ODOM_STALE_S)

    def _ctrl_tick(self):
        # /system/state が来なくなったら、CALIB のつもりのまま走り続けない。
        if (self._phase in _MOVING_PHASES and self._state_last_s is not None
                and self._now_s() - self._state_last_s > _STATE_STALE_S):
            self.get_logger().warn("/system/state が途絶 → 走行を止める")
            self._abort_all("state_stale")
            return
        if self._item == "BLIND" and self._phase == VERIFY_RUNNING:
            if self._estopped():
                self._abort_all("estop")
                return
            self._blind_verify_tick()
            return
        if self._phase not in _MOVING_PHASES or self._item not in MOTION_ITEMS:
            self._halt()
            return
        if self._estopped():
            self._abort_all("estop")
            return
        if not self._odom_fresh():
            self._run_failed("odom_stale")
            return
        if self._now_s() - self._run_t0 > self._p("calib_run_timeout_s"):
            self._run_failed("run_timeout")
            return

        target = self._commanded(self._item)
        twist = Twist()
        done = False
        if self._item == "LINEAR":
            dx = self._odom[0] - self._run_start[0]
            dy = self._odom[1] - self._run_start[1]
            travelled = math.hypot(dx, dy)
            remaining = target - travelled
            self._run_progress = min(1.0, travelled / target) if target > 0 else 1.0
            if remaining <= 0.0:
                done = True
            else:
                v = self._p("v_calib")
                if remaining < _TAPER_DIST_M:
                    v = max(_MIN_LINEAR_MPS, v * remaining / _TAPER_DIST_M)
                twist.linear.x = min(v, self._p("v_calib"))
        else:  # ROTATION
            target_rad = math.radians(target)
            turned = abs(self._run_yaw_acc)
            remaining = target_rad - turned
            self._run_progress = min(1.0, turned / target_rad) if target_rad > 0 else 1.0
            if remaining <= 0.0:
                done = True
            else:
                # 検証走行中は適用済みの値（bridge に入っている方）で角速度を計算する。
                base = (self._pending or self._current_value("ROTATION")).get("wheel_base", 0.39)
                w = min(_MAX_ANGULAR_RADS, self._p("v_calib") / (base / 2.0))
                if remaining < _TAPER_ANGLE_RAD:
                    w = max(_MIN_ANGULAR_RADS, w * remaining / _TAPER_ANGLE_RAD)
                twist.angular.z = w
        if done:
            self._halt()
            self._run_finished()
            return
        self._pub_cmd.publish(twist)
        self._cmd_active = True

    def _halt(self):
        if self._cmd_active:
            self._pub_cmd.publish(Twist())
            self._cmd_active = False
            self.get_logger().info("/cmd_vel_behavior を 0 にした")

    def _run_finished(self):
        item = self._item
        if self._phase == RUNNING:
            self._phase = WAIT_MEASURED
            self.get_logger().info(f"{item} の測定走行が完了 → 実測値の入力待ち")
            self._publish_status()
            self._emit("evt.calib_step_done", item=item)
        elif self._phase == VERIFY_RUNNING:
            self._phase = VERIFY_WAIT
            self.get_logger().info(f"{item} の検証走行が完了 → 実測値の入力待ち")
            self._publish_status()

    def _run_failed(self, reason: str):
        """走行できなかった／途中で失敗した。FSM は S2 に居るので /calib/start で再走行できる。"""
        self._halt()
        was_verify = self._phase in (VERIFY_RUNNING, APPLYING)
        self.get_logger().warn(f"走行失敗: {reason}")
        self._reason = reason
        if was_verify:
            # 検証走行の失敗は検証 NG と同じ扱い（適用前へ戻し、S2 へ）。
            self._fail_verify(reason)
            return
        self._phase = RETRY_WAIT
        self._publish_status(result="NG")

    # ── S3: 実測値の入力 ─────────────────────────────────
    def _on_submit(self, request, response):
        response.success = False
        response.preview_before = ""
        response.preview_after = ""
        item = request.item
        if not self._in_calib():
            response.preview_after = f"CALIB 以外のモード（{self._mode}）では受け付けません"
            return response
        accept = (WAIT_MEASURED, PREVIEW, VERIFY_WAIT)
        if item == "BLIND":
            # BLIND は S2（選択待ち＝RUNNING。検証 NG 後は RETRY_WAIT）でも選択を受ける。
            accept = (RUNNING, RETRY_WAIT, WAIT_MEASURED, PREVIEW)
        if item != self._item or self._phase not in accept:
            response.preview_after = f"今は実測値を受け付けない（item={self._item} phase={self._phase}）"
            return response
        try:
            extra = json.loads(_json_str(request.arg_json))
        except (ValueError, TypeError):
            extra = {}
        if isinstance(extra, dict) and extra.get("operator"):
            self._operator = str(extra["operator"])
        if item == "BLIND":
            return self._submit_blind(extra if isinstance(extra, dict) else {}, response)
        measured = float(request.measured)
        if self._phase == VERIFY_WAIT:
            return self._submit_verify(item, measured, response)
        return self._submit_preview(item, measured, response)

    def _submit_preview(self, item, measured, response):
        commanded = self._commanded(item)
        cur = self._current_value(item)
        self._measured = measured
        if item == "LINEAR":
            new = calib_core.corrected_wheel_radius_scale(
                cur["wheel_radius_scale"], commanded, measured)
            sane = calib_core.linear_preview_sane(
                commanded, measured, new, self._p("wheel_radius_scale_max_dev"))
            before = {"wheel_radius_scale": cur["wheel_radius_scale"]}
            after = {"wheel_radius_scale": new}
            pending = {"wheel_radius_scale": new}
        else:  # ROTATION
            new = calib_core.corrected_wheel_base(cur["wheel_base"], commanded, measured)
            sane = calib_core.rotation_preview_sane(commanded, measured, new, cur["wheel_base"])
            before = {"wheel_base": cur["wheel_base"]}
            after = {"wheel_base": new}
            pending = {"wheel_base": new}
        self._preview = (json.dumps(before), json.dumps(after, default=str))
        self._preview_sane = bool(sane)
        self._preview_insane = not sane
        self._pending = pending if sane else None
        self._phase = PREVIEW if sane else WAIT_MEASURED
        self.get_logger().info(
            f"{item} プレビュー: measured={measured} → {after} sane={sane}")
        self._publish_status(result="PREVIEW_OK" if sane else "PREVIEW_INSANE")
        response.success = bool(sane)
        response.preview_before, response.preview_after = self._preview
        return response

    def _submit_verify(self, item, measured, response):
        commanded = self._commanded(item)
        if item == "LINEAR":
            verdict = calib_core.verify_linear(
                commanded, measured, self._tolerance(self._p("calib_linear_tolerance_ratio")))
        else:
            verdict = calib_core.verify_rotation(
                commanded, measured, self._tolerance(self._p("calib_rotation_tolerance_deg")))
        self._verify_measured = measured
        response.preview_before = json.dumps({"commanded": commanded})
        response.preview_after = json.dumps({"measured": measured, "verdict": verdict.reason})
        response.success = bool(verdict.ok)
        if verdict.ok:
            self._phase = VERIFIED
            self._reason = verdict.reason
            self.get_logger().info(f"{item} 検証 OK（{verdict.reason}）")
            self._publish_status(result="OK")
            self._emit("evt.calib_step_done", item=item)
        else:
            self.get_logger().warn(f"{item} 検証 NG（{verdict.reason}）→ 適用前へ戻す")
            self._fail_verify(verdict.reason)
        return response

    # ── BLIND: 選択 → プレビュー ─────────────────────────
    def _on_scan(self, msg):
        self._scan = (msg.angle_min, msg.angle_increment, list(msg.ranges), self._now_s())
        if self._item == "BLIND" and self._phase == VERIFY_RUNNING \
                and self._now_s() >= self._verify_applied_s > 0.0:
            self._verify_frames.append(list(msg.ranges))

    def _on_scan_filtered(self, msg):
        self._scan_filtered = (msg.angle_min, msg.angle_increment, list(msg.ranges), self._now_s())

    def _blind_limits(self) -> dict:
        return {"max_sector_deg": self._p("blind_max_sector_deg"),
                "max_total_deg": self._p("blind_max_total_deg"),
                "max_sectors": int(self._p("blind_max_sectors"))}

    def _submit_blind(self, extra: dict, response):
        """選んだ角度帯（ranges=[[a0,a1],...]）を検査し、プレビューを作る。

        上限を超える選択・幅ゼロ・形の崩れは**適用に進めない**（PREVIEW_INSANE。LINEAR の A10 と同じ扱い）。
        """
        was_s2 = self._phase in (RUNNING, RETRY_WAIT)
        cur = self._current_value("BLIND")
        before = {"blind_angle_ranges": cur["blind_angle_ranges"]}
        sel = blind_core.validate_selection(extra.get("ranges"), **self._blind_limits())
        reason = sel.reason
        masked = 0
        if sel.ok:
            if self._scan is None:
                reason = "no_scan"
            else:
                a_min, a_inc, rng, _t = self._scan
                masked = blind_core.count_masked(sel.ranges, a_min, a_inc, rng)
        sane = bool(sel.ok and reason == "")
        flat = blind_core.flat_from_pairs(sel.ranges) if sel.ok else []
        after = {"blind_angle_ranges": flat, "masked_points": masked,
                 "total_deg": round(sel.total_deg, 2), "reason": reason}
        self._preview = (json.dumps(before), json.dumps(after))
        self._preview_sane = sane
        self._preview_insane = not sane
        self._pending = {"blind_angle_ranges": flat} if sane else None
        self._measured = None
        self._reason = reason
        self.get_logger().info(f"BLIND プレビュー: {after} sane={sane}")
        if sane:
            self._phase = PREVIEW
            self._publish_status(result="PREVIEW_OK")
            if was_s2:
                self._emit("evt.calib_step_done", item="BLIND")
        else:
            if not was_s2:
                self._phase = WAIT_MEASURED
            self._publish_status(result="PREVIEW_INSANE")
        response.success = sane
        response.preview_before, response.preview_after = self._preview
        return response

    # ── BLIND: 適用（3 ノードへ同時）と検証 ──────────────
    def _set_blind(self, flat: list, on_done):
        """`blind_angle_ranges` を obstacle_limiter・lidar_filter・opcheck_runner の**全部**へ送る。

        1 つでも届かない・拒否されたら on_done(False, 理由)。呼び出し側が全部を適用前へ戻す。
        """
        clients = self._blind_clients
        for name, cli in clients.items():
            if not cli.service_is_ready():
                if on_done:
                    on_done(False, f"node_unavailable:{name}")
                else:
                    self.get_logger().warn(f"{name} に届かず死角マスクを戻せなかった")
                return
        results = {}
        total = len(clients)

        def _one(name):
            def _done(fut):
                try:
                    res = fut.result().results
                    ok = bool(res) and all(r.successful for r in res)
                    reason = "; ".join(r.reason for r in res if not r.successful)
                except Exception as e:  # noqa: BLE001
                    ok, reason = False, str(e)
                results[name] = (ok, reason)
                if not ok:
                    self.get_logger().warn(f"{name} への死角マスク反映に失敗: {reason}")
                if len(results) == total:
                    bad = [f"{n}:{r}" for n, (o, r) in results.items() if not o]
                    if on_done:
                        on_done(not bad, ",".join(bad))
            return _done

        for name, cli in clients.items():
            req = SetParameters.Request()
            pm = ParameterMsg()
            pm.name = "blind_angle_ranges"
            pm.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                      double_array=[float(v) for v in flat])
            req.parameters.append(pm)
            cli.call_async(req).add_done_callback(_one(name))

    def _start_blind_verify(self):
        self._verify_frames = []
        self._verify_t0 = self._now_s()
        self._verify_applied_s = self._now_s()
        self._phase = VERIFY_RUNNING
        self._reason = ""
        self.get_logger().info("BLIND 検証を開始（/scan を集めて写り込みを推定する）")
        self._publish_status()

    def _blind_verify_tick(self):
        if self._now_s() - self._verify_t0 > self._p("calib_blind_verify_timeout_s"):
            self._fail_verify("verify_timeout:no_scan" if not self._verify_frames
                              else "verify_timeout:no_filtered_scan")
            return
        need = max(1, int(self._p("calib_blind_verify_frames")))
        filt = self._scan_filtered
        if len(self._verify_frames) < need or filt is None or filt[3] < self._verify_applied_s:
            return
        selected = [tuple(p) for p in blind_core.pairs_from_flat(
            (self._pending or {}).get("blind_angle_ranges", []))]
        a_min, a_inc, _r, _t = self._scan
        # 1) マスクが lidar_filter に実際に効いている（選んだ範囲の内側が /scan_filtered で inf）
        leaked = blind_core.unmasked_inside(selected, filt[0], filt[1], filt[2])
        if leaked > 0:
            self._fail_verify(f"mask_not_effective:{leaked}")
            return
        # 2) 恒常的な写り込みが、選んだ範囲の外に許容以上残っていない
        inc_deg = math.degrees(a_inc)
        persistent = blind_core.persistent_near_ranges(self._verify_frames, BLIND_NEAR_M)
        from th_maintenance.check_core import estimate_blind_sectors
        est = blind_core.shift_bands(
            estimate_blind_sectors(persistent, inc_deg, BLIND_NEAR_M, BLIND_MIN_RUN_DEG),
            math.degrees(a_min))
        verdict = blind_core.verify_blind(
            est, selected, self._tolerance(self._p("calib_blind_tolerance_deg")))
        self._verify_measured = None
        if verdict.ok:
            self._phase = VERIFIED
            self._reason = verdict.reason
            self.get_logger().info(f"BLIND 検証 OK（{verdict.reason}）")
            self._publish_status(result="OK")
            self._emit("evt.calib_step_done", item="BLIND")
        else:
            self.get_logger().warn(f"BLIND 検証 NG（{verdict.reason}）→ 適用前へ戻す")
            self._fail_verify(verdict.reason)

    # ── S4: 適用と検証 ───────────────────────────────────
    def _apply_and_verify(self, item: str):
        if not self._in_calib() or item != self._item or self._phase != PREVIEW \
                or not self._preview_sane:
            # FSM のガード（preview_sane）を通り抜けてきても、ここで二重に確認する。
            self.get_logger().warn(
                f"apply_and_verify({item}) を拒否（mode={self._mode} phase={self._phase} "
                f"sane={self._preview_sane}）")
            self._emit("evt.calib_verify_ng", item=item, reason="preview_not_sane")
            return
        if item == "IMU":
            self._phase = VERIFIED
            self._publish_status(result="OK")
            self._emit("evt.calib_step_done", item=item)
            return
        pending = dict(self._pending or {})
        if item == "BLIND":
            # プレビューで弾いているが、適用の直前にも上限を必ず通す（二重確認）。
            sel = blind_core.check_limits(
                blind_core.pairs_from_flat(pending.get("blind_angle_ranges", [])),
                **self._blind_limits())
            if not sel.ok:
                self.get_logger().warn(f"上限違反の死角マスクは適用しない（{sel.reason}）")
                self._fail_verify(f"blind_limit_violation:{sel.reason}")
                return
            self._phase = APPLYING
            self._revert_to = self._current_value(item)
            self._applied = True      # 一部のノードにだけ入った場合も戻せるよう、送る前から立てる
            self._publish_status(result="APPLYING")
            self._set_blind(pending["blind_angle_ranges"], self._on_applied)
            return
        # A10（LINEAR）。プレビューで弾いているが、適用の直前にも必ず通す。
        if item == "LINEAR" and not calib_core.a10_ok(
                pending.get("wheel_radius_scale", math.nan), self._p("wheel_radius_scale_max_dev")):
            self.get_logger().warn("A10 違反の値は適用しない")
            self._fail_verify("a10_violation")
            return
        self._phase = APPLYING
        self._revert_to = self._current_value(item)
        self._publish_status(result="APPLYING")
        self._set_bridge(item, pending, self._on_applied)

    def _on_applied(self, ok: bool, reason: str):
        if self._phase != APPLYING:
            # 適用の応答が返る前に中断された。反映済みの値を戻す。
            if ok and self._revert_to is not None and self._item is not None:
                self._set_runtime(self._item, self._revert_to)
            return
        item = self._item
        if not ok:
            self._fail_verify(f"apply_failed:{reason}")
            return
        self._applied = True
        self.get_logger().info(f"{item} をランタイムへ適用: {self._pending}（検証へ）")
        if item == "BLIND":
            self._start_blind_verify()
            return
        self._start_run(item, verify=True)

    def _fail_verify(self, reason: str):
        """検証 NG／適用失敗: **適用前の値へ戻し**、FSM へ検証 NG を伝える（T-CAL-06）。"""
        self._halt()
        self._reason = reason
        self._revert_runtime()
        self._phase = RETRY_WAIT
        self._publish_status(result="NG")
        self._emit("evt.calib_verify_ng", item=self._item, reason=reason)

    def _on_revert_effect(self, item: str):
        # 戻す本体は _fail_verify が済ませている。ここは phase を S2 の待機に揃えるだけ。
        if not self._in_calib() or item != self._item:
            return
        if self._phase not in (RETRY_WAIT, IDLE):
            self.get_logger().warn(f"revert_calib を受けたが phase={self._phase}。適用前へ戻す")
            self._revert_runtime()
            self._halt()
            self._phase = RETRY_WAIT
        self._publish_status(result="NG")

    def _revert_runtime(self):
        """ランタイムに入れた未確定の値を適用前へ戻す（冪等）。"""
        if self._applied and self._revert_to is not None and self._item in RUNTIME_ITEMS:
            self.get_logger().warn(f"{self._item} を適用前の値へ戻す: {self._revert_to}")
            self._set_runtime(self._item, self._revert_to)
        self._applied = False

    # ── 確定・破棄 ───────────────────────────────────────
    def _commit(self, item: str):
        """確定。**検証に合格した（VERIFIED）ときだけ**書く。"""
        if not self._in_calib() or item != self._item or self._phase != VERIFIED:
            self.get_logger().warn(
                f"commit_calib({item}) を拒否（mode={self._mode} phase={self._phase}）。"
                "検証に合格していない補正値は確定しない")
            return
        if item == "IMU":
            values = {"calib_status": float(self._imu_calib)}
            verification = {"result": "OK", "calib_status": int(self._imu_calib)}
        elif item == "BLIND":
            values = dict(self._pending or {})
            verification = {
                "result": "OK",
                "tolerance_deg": self._p("calib_blind_tolerance_deg"),
                "detail": self._reason,
            }
        else:
            values = dict(self._pending or {})
            verification = {
                "result": "OK",
                "commanded": self._commanded(item),
                "measured_before": self._measured,
                "measured_verify": self._verify_measured,
                "detail": self._reason,
            }
        calibrated_at = time.strftime("%Y-%m-%dT%H:%M:%S")
        self._store.commit(item, values, calibrated_at, verification,
                           operator=self._operator or str(self.get_parameter("calib_operator").value),
                           params_digest=self._params_digest())
        self.get_logger().info(f"{item} を確定した: {values}")
        self._applied = False        # 以後は確定値（戻さない）
        self._phase = IDLE
        self._item = None
        self._publish_status(result="COMMITTED")

    def _discard(self, item: str):
        """中断・フォルト: 補正値を確定せず、適用済みなら適用前へ戻す（§7 #3）。"""
        self.get_logger().warn(f"discard_calib({item}): 確定せず破棄する")
        self._abort_all("discard")

    def _abort_all(self, reason: str):
        """停止・適用前へ戻す・待機へ。CALIB を抜けたときも、中断・フォルトでもここへ来る。"""
        self._halt()
        self._revert_runtime()
        was_active = self._phase != IDLE
        self._phase = IDLE
        self._item = None
        self._pending = None
        self._preview = ("", "")
        self._preview_sane = False
        self._preview_insane = False
        self._measured = None
        self._verify_frames = []
        self._reason = reason
        if was_active:
            self._publish_status(result="ABORTED")

    # ── IMU ──────────────────────────────────────────────
    def _on_imu_calib(self, msg: UInt8):
        self._imu_calib = int(msg.data)
        if self._item == "IMU" and self._phase == RUNNING:
            self._check_imu_done()

    def _check_imu_done(self):
        if self._item != "IMU" or self._phase != RUNNING or not self._in_calib():
            return
        if calib_core.imu_all_calibrated(self._imu_calib):
            self._phase = PREVIEW
            self._preview_sane = True
            self._preview = ("", json.dumps({"calib_status": self._imu_calib}))
            self.get_logger().info("IMU 校正が全て 3 になった → S3 へ")
            self._publish_status(result="PREVIEW_OK")
            self._emit("evt.calib_step_done", item="IMU")

    # ── /calib/start（再走行専用）────────────────────────
    def _on_start(self, request, response):
        response.started = False
        response.message = ""
        if not self._in_calib():
            response.message = f"CALIB 以外のモード（{self._mode}）では実行できません"
            return response
        if request.item != self._item or request.item not in RUNTIME_ITEMS:
            response.message = f"今は {request.item} を再走行できない（実行中の項目={self._item}）"
            return response
        if self._phase != RETRY_WAIT or self._state != "S2":
            response.message = (f"再走行は検証 NG／走行失敗で S2 に戻った後だけ"
                                f"（phase={self._phase} state={self._state}）")
            return response
        if self._start_run(request.item, verify=False):
            response.started = True
            response.message = f"{request.item} の再走行を開始しました"
        else:
            response.message = "再走行を開始できなかった"
        return response

    # ── /calib/apply（冪等な確認。進めるのは FSM）─────────
    def _on_apply(self, request, response):
        response.success = bool(
            self._in_calib() and request.item == self._item
            and self._phase in (APPLYING, VERIFY_RUNNING, VERIFY_WAIT, VERIFIED))
        response.message = self._phase if response.success else \
            f"適用は進行していない（item={self._item} phase={self._phase}）"
        return response

    # ── /calib/rollback ──────────────────────────────────
    def _on_rollback(self, request, response):
        response.success = False
        if not self._in_calib() or self._state != "LIST" or self._phase != IDLE:
            self.get_logger().warn(
                f"rollback を拒否（mode={self._mode} state={self._state} phase={self._phase}）")
            return response
        if request.item not in ITEM_VALID:
            return response
        hist = self._store.history(request.item)
        gen = int(request.generation)
        if gen < 1 or gen > len(hist):
            return response
        target = hist[gen - 1].get("values") or {}
        if request.item == "LINEAR" and not calib_core.a10_ok(
                float(target.get("wheel_radius_scale", math.nan)),
                self._p("wheel_radius_scale_max_dev")):
            self.get_logger().warn("A10 違反の履歴値へは戻せない")
            return response
        if request.item == "BLIND":
            # 履歴の値も上限を通す（上限を後から変えた・壊れた履歴を安全判定へ入れない）。
            sel = blind_core.check_limits(
                blind_core.pairs_from_flat(list(target.get("blind_angle_ranges", []))),
                **self._blind_limits())
            if not sel.ok:
                self.get_logger().warn(f"上限違反の履歴値へは戻せない（{sel.reason}）")
                return response
        restored = self._store.rollback(request.item, gen)
        if restored is None:
            return response
        if request.item in RUNTIME_ITEMS:
            self._set_runtime(request.item, restored["values"])
        self.get_logger().info(f"{request.item} を世代 {gen} へ戻した: {restored['values']}")
        response.success = True
        self._publish_status(result="ROLLED_BACK")
        return response

    def _set_runtime(self, item: str, values: dict):
        """適用前への復帰・ロールバックの反映先を項目で振り分ける（結果は問わない）。"""
        if item == "BLIND":
            self._set_blind(list(values.get("blind_angle_ranges", [])), None)
        else:
            self._set_bridge(item, values, None)

    # ── esp32_bridge への反映 ────────────────────────────
    def _set_bridge(self, item: str, values: dict, on_done):
        """`wheel_radius_scale` / `wheel_base` をランタイムで設定する（非同期）。"""
        if not self._cli_set.service_is_ready():
            if on_done:
                on_done(False, "bridge_unavailable")
            else:
                self.get_logger().warn(f"esp32_bridge に届かず {item} の値を戻せなかった")
            return
        req = SetParameters.Request()
        for name, value in values.items():
            pm = ParameterMsg()
            pm.name = name
            pm.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                      double_value=float(value))
            req.parameters.append(pm)
        future = self._cli_set.call_async(req)

        def _done(fut):
            try:
                results = fut.result().results
                ok = bool(results) and all(r.successful for r in results)
                reason = "; ".join(r.reason for r in results if not r.successful)
            except Exception as e:  # noqa: BLE001
                ok, reason = False, str(e)
            if not ok:
                self.get_logger().warn(f"esp32_bridge への反映に失敗: {reason}")
            if on_done:
                on_done(ok, reason)

        future.add_done_callback(_done)

    def _startup_apply_tick(self):
        """起動時に current.yaml の LINEAR / ROTATION を bridge へ反映する（A10 を通ったものだけ）。"""
        if self._startup_applied:
            return
        if not self._cli_set.service_is_ready():
            return
        self._startup_applied = True
        for item in MOTION_ITEMS:
            entry = self._store.current_entry(item)
            if not entry or not isinstance(entry.get("values"), dict):
                continue
            values = entry["values"]
            if item == "LINEAR" and not calib_core.a10_ok(
                    float(values.get("wheel_radius_scale", math.nan)),
                    self._p("wheel_radius_scale_max_dev")):
                self.get_logger().warn("保存された LINEAR が A10 違反のため反映しない")
                continue
            self._set_bridge(item, values, None)
            self.get_logger().info(f"保存済みの {item} を esp32_bridge へ反映: {values}")

    # ── 出力 ─────────────────────────────────────────────
    def _emit(self, event: str, **arg):
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = event
        ev.source_node = "calib_runner"
        ev.arg_json = json.dumps(arg)
        self._pub_event.publish(ev)
        self.get_logger().info(f"/system/event 発行: {event} {ev.arg_json}")

    def _params_digest(self) -> str:
        try:
            with open(str(self.get_parameter("params_digest_path").value), encoding="utf-8") as f:
                return str(json.load(f).get("params_digest", ""))
        except (OSError, ValueError, AttributeError):
            return ""

    def _detail(self) -> dict:
        item = self._item
        detail = {"phase": self._phase, "reason": self._reason,
                  "calib_state": self._state if self._in_calib() else ""}
        if item in MOTION_ITEMS:
            detail["commanded"] = self._commanded(item)
            detail["current"] = self._current_value(item)
            detail["progress"] = round(self._run_progress, 3)
        if item == "BLIND":
            sel_flat = (self._pending or {}).get("blind_angle_ranges")
            detail["current"] = self._current_value("BLIND")
            detail["blind"] = {
                "selected": sel_flat,
                "limits": {"max_sector_deg": self._p("blind_max_sector_deg"),
                           "max_total_deg": self._p("blind_max_total_deg"),
                           "max_sectors": int(self._p("blind_max_sectors"))},
                "tolerance_deg": self._p("calib_blind_tolerance_deg"),
                "scan_received": self._scan is not None,
                "verify_frames": len(self._verify_frames),
            }
        if item == "IMU":
            c = self._imu_calib
            detail["imu"] = {"sys": (c >> 6) & 3, "gyro": (c >> 4) & 3,
                             "accel": (c >> 2) & 3, "mag": c & 3}
        last = {}
        history = {}
        for it in ITEM_VALID:
            entry = self._store.current_entry(it)
            last[it] = (entry or {}).get("calibrated_at", "")
            history[it] = [{"generation": i + 1, "values": h.get("values"),
                            "calibrated_at": h.get("calibrated_at", "")}
                           for i, h in enumerate(self._store.history(it))]
        detail["last_calibrated"] = last
        detail["history"] = history
        warnings = []
        if item == "ROTATION":
            lin, rot = last.get("LINEAR", ""), last.get("ROTATION", "")
            if not lin:
                warnings.append("linear_not_calibrated")
            elif rot and lin < rot:
                warnings.append("linear_older_than_rotation")
        detail["warnings"] = warnings
        return detail

    def _publish_status(self, result: str = ""):
        if not result:
            result = {
                IDLE: "IDLE", GUIDE: "GUIDE",
                RUNNING: "PREVIEW_INSANE" if self._preview_insane else "RUNNING",
                WAIT_MEASURED: "PREVIEW_INSANE" if self._preview_insane else "WAIT_MEASURED",
                PREVIEW: "PREVIEW_OK" if self._preview_sane else "WAIT_MEASURED",
                APPLYING: "APPLYING", VERIFY_RUNNING: "VERIFY_RUNNING",
                VERIFY_WAIT: "WAIT_VERIFY", VERIFIED: "OK",
                RETRY_WAIT: "PREVIEW_INSANE" if self._preview_insane else "RETRY_WAIT",
            }[self._phase]
        st = CalibStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.item = self._item or ""
        st.step = self._state if self._in_calib() else ""
        st.result = result
        st.preview_before, st.preview_after = self._preview
        st.detail = json.dumps(self._detail(), ensure_ascii=False)
        self._pub_status.publish(st)

    def _status_tick(self):
        self._publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = CalibRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
