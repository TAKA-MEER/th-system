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
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, UInt8
    from th_system_msgs.msg import CalibStatus, StateEffect, StateEvent, SystemState
    from th_system_msgs.srv import ApplyCalib, RollbackCalib, StartCalib, SubmitCalib
    RS_OK = True
except ImportError:
    # ホスト pytest（rclpy 無し）から PARS 等の純粋な定義だけを import できるようにする。
    rclpy = None
    Node = object
    RS_OK = False

from th_maintenance import calib_core
from th_maintenance.calib_store import CalibStore

# registry.yaml と一致させるパラメータの既定値（consumers に calib_runner を持つ行のうち値あり）。
PARS: dict = {
    "v_calib": 0.15,
    "wheel_radius_scale_max_dev": 0.10,
}

# registry 未登録のノード局所の既定値（names.md §7 に名前が無いため registry へ足していない。
# 校正の「指定距離・指定角度」と走行の安全タイムアウト。値の正式な置き場は実装管理担当が決める）。
LOCAL_PARS: dict = {
    "calib_linear_distance_m": 1.0,
    "calib_rotation_deg": 180.0,
    "calib_run_timeout_s": 60.0,
}

# 許容範囲は未確定（registry は status: placeholder・生成 YAML には載らない）。
# 既定の -1.0 は「未確定」の意味で、検証は合格にならない（calib_core.verify_*）。
TOLERANCE_UNDEFINED = -1.0

ITEM_VALID = calib_core.ITEMS
MOTION_ITEMS = ("LINEAR", "ROTATION")

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
        return {}

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
        if item != self._item or self._phase not in (WAIT_MEASURED, PREVIEW, VERIFY_WAIT):
            response.preview_after = f"今は実測値を受け付けない（item={self._item} phase={self._phase}）"
            return response
        try:
            extra = json.loads(_json_str(request.arg_json))
        except (ValueError, TypeError):
            extra = {}
        if isinstance(extra, dict) and extra.get("operator"):
            self._operator = str(extra["operator"])
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
                self._set_bridge(self._item, self._revert_to, None)
            return
        item = self._item
        if not ok:
            self._fail_verify(f"apply_failed:{reason}")
            return
        self._applied = True
        self.get_logger().info(f"{item} をランタイムへ適用: {self._pending}（検証走行へ）")
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
        if self._applied and self._revert_to is not None and self._item in MOTION_ITEMS:
            self.get_logger().warn(f"{self._item} を適用前の値へ戻す: {self._revert_to}")
            self._set_bridge(self._item, self._revert_to, None)
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
        if request.item != self._item or request.item not in MOTION_ITEMS:
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
        restored = self._store.rollback(request.item, gen)
        if restored is None:
            return response
        if request.item in MOTION_ITEMS:
            self._set_bridge(request.item, restored["values"], None)
        self.get_logger().info(f"{request.item} を世代 {gen} へ戻した: {restored['values']}")
        response.success = True
        self._publish_status(result="ROLLED_BACK")
        return response

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
                IDLE: "IDLE", GUIDE: "GUIDE", RUNNING: "RUNNING",
                WAIT_MEASURED: "PREVIEW_INSANE" if self._preview_insane else "WAIT_MEASURED",
                PREVIEW: "PREVIEW_OK" if self._preview_sane else "WAIT_MEASURED",
                APPLYING: "APPLYING", VERIFY_RUNNING: "VERIFY_RUNNING",
                VERIFY_WAIT: "WAIT_VERIFY", VERIFIED: "OK", RETRY_WAIT: "RETRY_WAIT",
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
