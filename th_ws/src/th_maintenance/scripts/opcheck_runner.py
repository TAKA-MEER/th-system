#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""始業点検の実行部 `opcheck_runner`（WP-MAINT-01 / パケット 10-a）。

`/system/state` が `OPCHECK` のときだけ動くノード（設計書 §1「ノード側でも二重に確認」）。
メインループは FSM（`state_manager`）が出す `/system/effect` の自分宛て effect で進み、
判定結果は `evt.check_result {"item", "result"}` を `/system/event` へ返す。

- 項目: `ESTOP`（物理非常停止ボタンの状態把握）・`MOTOR`（押している間だけ低速で動く）・
  `IMU`（データ生死・バイアス・校正状態）・`LIDAR`（死活・周期・全周の有効性・死角マスクとのズレ）
- `MOTOR` の「押している」状態の伝え方は設計書に無いため、**デッドマン方式**で作る: 未来の画面
  （S-30）が `/opcheck/motor_hold`（std_msgs/String、`NONE`/`FORWARD`/`BACK`/`LEFT`/`RIGHT`）を
  一定周期で送り続け、`opcheck_deadman_timeout_s` だけ途絶えたら「離し」扱いで即座に 0 にする。
  （トピック名・型・途絶時間は報告書に記載し、実装管理担当が設計書へ写す）
- `ESTOP` 項目の実行中に物理ボタンを押しても `CARRY` へ落ちないのは FSM 側（`T-OPC-05` の
  `feed_check_input`）が担う予定だが、ランタイム配線の初期値（引数なし）が現状は届かない。
  ノード側は受ける用意をしたうえで、実値は `/safety/estop_hw` の購読から取る（配線の改修は
  別パケット。報告書に記載）。
- フォルト・非常停止・`OPCHECK` 以外への遷移では即座に `/cmd_vel_behavior` を 0 にする。
"""

import json
import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy, qos_profile_sensor_data)
    from rcl_interfaces.msg import ParameterDescriptor
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, String, UInt8
    from th_system_msgs.msg import (CheckStatus, StateEffect, StateEvent,
                                    SystemState, WheelFeedback)
    from th_system_msgs.srv import AnswerCheck, RunCheck
    RS_OK = True
except ImportError:
    # ホスト pytest（rclpy 無し）からこのモジュールの純粋な定義
    # （PARS 等）だけを import できるようにするためのガード。
    rclpy = None
    Node = object
    RS_OK = False

from th_maintenance.check_core import (CheckParams, judge_estop, judge_gyro_unit,
                                       judge_imu, judge_lidar,
                                       judge_motor_samples, verdict_to_fsm_result)

# registry.yaml（WP-MAINT-01 ブロック）と一致させるパラメータの既定値。
# test_maintenance_registry_driven.py が registry と照合する。
PARS: dict = {
    "motor_deadband_mps": 0.03,
    "motor_follow_min_ratio": 0.5,
    "imu_bias_max_rad_s": 0.2,
    "imu_wz_implausible_rad_s": 10.0,
    "opcheck_imu_window_s": 2.0,
    "opcheck_deadman_timeout_s": 0.5,
    "opcheck_spin_w_rad_s": 0.3,
    "opcheck_blind_tolerance_deg": 5.0,
    "opcheck_scan_coverage_gap_deg": 2.0,
    "v_check": 0.05,
    "scan_stale_ms": 300.0,
}

ITEM_VALID = ("ESTOP", "MOTOR", "IMU", "LIDAR")

# NG の行き先（設計書 §2.1）。WARN は IMU だけ校正へ誘導する。
_NEXT_SCREEN = {
    "ESTOP": "repair",
    "MOTOR": "repair",
    "IMU": "imu_calib",
    "LIDAR": "lidar_calib",
}

_ESTOP_STALE_MS = 3000.0  # /safety/estop_hw がこれだけ来なかったら「届いていない」とみなす


def _json_str(text: str) -> str:
    return text or "{}"


class OpcheckRunner(Node):
    """始業点検の実行部。OPCHECK のときだけ動作する。"""

    def __init__(self):
        if not RS_OK:
            raise ImportError("rclpy が無い環境で OpcheckRunner を実体化した")
        super().__init__("opcheck_runner")
        for name, value in PARS.items():
            self.declare_parameter(name, value)
        self.declare_parameter("blind_angle_ranges", [],
                              ParameterDescriptor(dynamic_typing=True))

        self._p = CheckParams(
            motor_deadband_mps=float(self.get_parameter("motor_deadband_mps").value),
            motor_follow_min_ratio=float(self.get_parameter("motor_follow_min_ratio").value),
            imu_bias_max_rad_s=float(self.get_parameter("imu_bias_max_rad_s").value),
            imu_wz_implausible_rad_s=float(self.get_parameter("imu_wz_implausible_rad_s").value),
            opcheck_imu_window_s=float(self.get_parameter("opcheck_imu_window_s").value),
            opcheck_deadman_timeout_s=float(self.get_parameter("opcheck_deadman_timeout_s").value),
            opcheck_spin_w_rad_s=float(self.get_parameter("opcheck_spin_w_rad_s").value),
            opcheck_blind_tolerance_deg=float(self.get_parameter("opcheck_blind_tolerance_deg").value),
            opcheck_scan_coverage_gap_deg=float(self.get_parameter("opcheck_scan_coverage_gap_deg").value),
            v_check=float(self.get_parameter("v_check").value),
            scan_stale_ms=float(self.get_parameter("scan_stale_ms").value),
        )

        # ── 内部状態 ──────────────────────────────────────
        self._mode = "INIT"
        self._state = "NONE"
        self._estop_ui = False
        self._estop_hw = False

        self._item = None
        self._item_started_ms = 0.0

        # MOTOR
        self._hold_val = "NONE"
        self._hold_ms = 0.0
        self._cmd_last = (0.0, 0.0)
        self._motor_samples = []
        self._cmd_active = False

        # ESTOP
        self._estop_raw = False
        self._estop_last_ms = 0.0
        self._estop_alive = False
        self._estop_pressed = None
        self._estop_saw_press = False
        self._estop_saw_release = False
        self._estop_false_answers = 0
        self._last_estop_verdict = (None, None)

        # IMU
        self._imu_alive = False
        self._imu_last_ms = 0.0
        self._imu_wz = []
        self._imu_calib = 0
        self._imu_bias = 0.0
        self._imu_max_wz = None
        self._imu_final = False

        # LIDAR
        self._scan_alive = False
        self._scan_last_ms = 0.0
        self._scan_prev_stamp = None
        self._scan_period = None
        self._scan_ranges = []
        self._scan_angle_inc = math.radians(1.0)
        self._last_lidar_verdict = (None, None)

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
        self._sub_state = self.create_subscription(
            SystemState, "/system/state", self._on_state, state_qos)
        self._sub_effect = self.create_subscription(
            StateEffect, "/system/effect", self._on_effect, effect_qos)
        self._sub_estop = self.create_subscription(
            Bool, "/safety/estop_hw", self._on_estop_hw, 10)
        self._sub_wheel_cmd = self.create_subscription(
            WheelFeedback, "/esp32/wheel_cmd_speed", self._on_wheel_cmd, 10)
        self._sub_wheel_fb = self.create_subscription(
            WheelFeedback, "/esp32/wheel_feedback", self._on_wheel_feedback, 10)
        self._sub_imu = self.create_subscription(
            Imu, "/esp32/imu_data", self._on_imu, 10)
        self._sub_imu_calib = self.create_subscription(
            UInt8, "/esp32/imu_calib_status", self._on_imu_calib, 10)
        self._sub_scan = self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self._sub_hold = self.create_subscription(
            String, "/opcheck/motor_hold", self._on_motor_hold, 10)

        # ── Publishers ────────────────────────────────────
        self._pub_status = self.create_publisher(CheckStatus, "/opcheck/status", status_qos)
        self._pub_event = self.create_publisher(StateEvent, "/system/event", event_qos)
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel_behavior", cmd_qos)

        # ── Services ──────────────────────────────────────
        self._srv_run = self.create_service(RunCheck, "/opcheck/run_item", self._on_run_item)
        self._srv_answer = self.create_service(AnswerCheck, "/opcheck/answer", self._on_answer)

        # ── Timers ────────────────────────────────────────
        self._deadman_timer = self.create_timer(0.05, self._deadman_tick)
        self._monitor_timer = self.create_timer(0.1, self._monitor_tick)

        self.get_logger().info(
            f"opcheck_runner 起動（v_check={self._p.v_check}, "
            f"deadman={self._p.opcheck_deadman_timeout_s}s）")

    # ── 時計 ─────────────────────────────────────────────
    @staticmethod
    def _now_ms() -> float:
        return time.monotonic() * 1000.0

    # ── /system/state ────────────────────────────────────
    def _on_state(self, msg: SystemState):
        self._mode = msg.mode
        self._state = msg.state
        self._estop_ui = msg.estop_ui
        self._estop_hw = msg.estop_hw
        if self._mode != "OPCHECK":
            if self._item is not None:
                self.get_logger().warn(
                    f"OPCHECK を離脱（{self._mode}）→ 実行中の項目 {self._item} を中断する")
                self._abort_item()
        self._sync_command()

    # ── /system/effect ───────────────────────────────────
    def _on_effect(self, msg: StateEffect):
        if msg.dest != "opcheck_runner":
            return
        name = msg.name
        args = json.loads(_json_str(msg.args_json))
        if name == "start_monitor":
            item = str(args.get("item") or "")
            if not self._start_item(item):
                self.get_logger().warn(f"start_monitor({item}) を無視")
        elif name == "record_result":
            item = str(args.get("item") or "")
            if item == self._item:
                self.get_logger().info(f"record_result({item}) → 項目を閉じる")
                self._close_item()
        elif name == "feed_check_input":
            # T-OPC-05。ESTOP 項目中に FSM から押下/解除を受け取る。ランタイム配線の
            # 初期値が null のため現状は届かない想定で、実値は /safety/estop_hw から取る。
            if self._item == "ESTOP":
                self._note_estop(bool(args.get("pressed")))
        elif name == "abort_check":
            self._abort_item()
        else:
            self.get_logger().debug(f"無視する effect: {name}")

    # ── 項目の開始・終了 ─────────────────────────────────
    def _start_item(self, item: str) -> bool:
        if self._mode != "OPCHECK":
            self.get_logger().warn(f"OPCHECK 以外（{self._mode}）では項目を開始しない")
            return False
        if item not in ITEM_VALID:
            self.get_logger().warn(f"不明な項目: {item}")
            return False
        if self._item is not None:
            self.get_logger().warn(f"別の項目 {self._item} が実行中")
            return False
        self._item = item
        self._item_started_ms = self._now_ms()
        self._reset_accums(item)
        self.get_logger().info(f"項目開始: {item}")
        self._publish_status(detail=f"項目を開始しました ({item})")
        return True

    def _reset_accums(self, item: str):
        self._motor_samples = []
        if item == "ESTOP":
            self._estop_alive = False
            self._estop_pressed = None
            self._estop_saw_press = False
            self._estop_saw_release = False
            self._estop_false_answers = 0
            self._last_estop_verdict = (None, None)
        elif item == "IMU":
            self._imu_alive = False
            self._imu_wz = []
            self._imu_bias = 0.0
            self._imu_max_wz = None
            self._imu_final = False
        elif item == "LIDAR":
            self._scan_prev_stamp = None
            self._scan_period = None
            self._last_lidar_verdict = (None, None)

    def _close_item(self):
        self._publish_status(detail=f"項目 {self._item} を終了しました")
        self._item = None

    def _abort_item(self):
        if self._item is not None:
            self.get_logger().warn(f"項目 {self._item} を中断")
        self._item = None
        self._halt_motor()
        self._publish_status(detail="中断しました")

    # ── Services ─────────────────────────────────────────
    def _on_run_item(self, request, response):
        response.started = False
        response.message = ""
        if self._mode != "OPCHECK":
            response.message = f"OPCHECK 以外のモード（{self._mode}）では実行できません"
            return response
        if request.item not in ITEM_VALID:
            response.message = f"不明な項目: {request.item}"
            return response
        if self._item is not None:
            response.message = f"別の項目 {self._item} が実行中です"
            return response
        if self._start_item(request.item):
            response.started = True
            response.message = f"{request.item} を開始しました"
        return response

    def _on_answer(self, request, response):
        response.accepted = False
        if self._item != request.item:
            return response
        if request.item == "ESTOP":
            if not request.ok:
                self._estop_false_answers += 1
                self.get_logger().warn("ESTOP 項目: 目視回答が不一致（NG 扱い）")
                self._update_estop_verdict()
            response.accepted = True
        else:
            self.get_logger().debug(f"{request.item} は目視回答の対象外")
        return response

    # ── MOTOR: 押下（デッドマン）と指令 ──────────────────
    def _on_motor_hold(self, msg: String):
        val = msg.data.strip().upper() if msg.data else "NONE"
        if val not in ("NONE", "FORWARD", "BACK", "LEFT", "RIGHT"):
            self.get_logger().warn(f"unknown motor_hold: {msg.data}")
            return
        prev = self._hold_val
        self._hold_val = val
        self._hold_ms = self._now_ms()
        if val == "NONE":
            if prev != "NONE":
                self._maybe_finalize_motor()
        else:
            if prev == "NONE":
                self._motor_samples = []
                self.get_logger().info(f"MOTOR 押下開始: {val}")
        self._sync_command()

    def _deadman_tick(self):
        if self._hold_val != "NONE":
            if self._now_ms() - self._hold_ms > self._p.opcheck_deadman_timeout_s * 1000.0:
                self.get_logger().warn(
                    f"MOTOR 押下が {self._p.opcheck_deadman_timeout_s}s 途絶 → 離し扱い")
                self._maybe_finalize_motor()
                self._hold_val = "NONE"
        self._sync_command()

    def _on_wheel_cmd(self, msg: WheelFeedback):
        self._cmd_last = (msg.left_speed, msg.right_speed)

    def _on_wheel_feedback(self, msg: WheelFeedback):
        if self._item == "MOTOR" and self._mode == "OPCHECK" and self._hold_val != "NONE":
            self._motor_samples.append(
                (self._cmd_last[0], msg.left_speed, self._cmd_last[1], msg.right_speed))

    def _maybe_finalize_motor(self):
        if self._item != "MOTOR":
            return
        # 2026-09-25 修正: サンプルが1つも無い（指令すら通らなかった＝配線断・
        # ESP32未接続等）を早期 return で無視していたため、judge_motor_samples()
        # の NG("no_samples") 分岐に一度も到達せず、evt.check_result が
        # 永遠に出ないまま RUNNING_CHECK に固着していた（モーター全損を検知
        # できない最悪ケースを取り逃す）。常に judge_motor_samples() を呼ぶ。
        verdict = judge_motor_samples(self._motor_samples, self._p)
        self._motor_samples = []
        self.get_logger().info(f"MOTOR 判定: {verdict.result} ({verdict.reason})")
        self._publish_final("MOTOR", verdict)
        self._emit_result("MOTOR", verdict)

    def _twist_for(self, direction: str):
        t = Twist()
        if direction == "FORWARD":
            t.linear.x = float(self._p.v_check)
        elif direction == "BACK":
            t.linear.x = -float(self._p.v_check)
        elif direction == "LEFT":
            t.angular.z = float(self._p.opcheck_spin_w_rad_s)
        elif direction == "RIGHT":
            t.angular.z = -float(self._p.opcheck_spin_w_rad_s)
        else:
            return None
        return t

    def _sync_command(self):
        if self._mode != "OPCHECK" or (self._estop_ui or self._estop_hw or self._estop_raw):
            self._halt_motor()
            return
        direction = self._hold_val
        if direction == "NONE":
            self._halt_motor()
            return
        twist = self._twist_for(direction)
        if twist is None:
            self._halt_motor()
            return
        self._pub_cmd.publish(twist)
        if not self._cmd_active:
            self._cmd_active = True
            self.get_logger().info(f"/cmd_vel_behavior に指令（{direction}）")

    def _halt_motor(self):
        if self._cmd_active:
            self._pub_cmd.publish(Twist())
            self._cmd_active = False
            self.get_logger().info("/cmd_vel_behavior を 0 にした")

    # ── ESTOP 項目 ───────────────────────────────────────
    def _on_estop_hw(self, msg: Bool):
        self._estop_raw = msg.data
        self._estop_last_ms = self._now_ms()
        if self._item == "ESTOP":
            self._note_estop(msg.data)
        self._sync_command()

    def _note_estop(self, pressed: bool):
        if self._estop_pressed is None:
            self._estop_alive = True
            self._estop_pressed = pressed
            if pressed:
                self._estop_saw_press = True
            return
        if pressed and not self._estop_pressed:
            self._estop_saw_press = True
        elif not pressed and self._estop_pressed:
            self._estop_saw_release = True
        self._estop_pressed = pressed
        self._update_estop_verdict()

    def _estop_tick(self):
        if self._item != "ESTOP":
            return
        fresh = self._now_ms() - self._estop_last_ms <= _ESTOP_STALE_MS
        if self._estop_alive and not fresh:
            self._estop_alive = False
            self.get_logger().warn("/safety/estop_hw 途絶（ESTOP 項目）")
            self._update_estop_verdict()

    def _update_estop_verdict(self):
        verdict = judge_estop(self._estop_alive, self._estop_saw_press,
                              self._estop_saw_release)
        if verdict.result == "OK" and self._estop_false_answers:
            from th_maintenance.check_core import NG
            verdict = NG("answer_mismatch")
        key = (verdict.result, verdict.reason)
        if key != self._last_estop_verdict:
            self._last_estop_verdict = key
            self._publish_final("ESTOP", verdict)
            self._emit_result("ESTOP", verdict)

    # ── IMU 項目 ─────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        self._imu_alive = True
        self._imu_last_ms = self._now_ms()
        if self._item == "IMU" and not self._imu_final:
            self._imu_wz.append(msg.angular_velocity.z)

    def _on_imu_calib(self, msg: UInt8):
        self._imu_calib = int(msg.data)

    def _imu_tick(self):
        if self._item != "IMU" or self._imu_final:
            return
        if self._now_ms() - self._item_started_ms < self._p.opcheck_imu_window_s * 1000.0:
            return
        self._imu_final = True
        wz = self._imu_wz
        self._imu_bias = sum(wz) / len(wz) if wz else 0.0
        self._imu_max_wz = max((abs(w) for w in wz), default=None)
        verdict = judge_imu(self._imu_calib, self._imu_bias, self._imu_alive, self._p)
        unit = judge_gyro_unit(self._imu_max_wz, self._p)
        if unit.result == "NG":
            verdict = unit
        self.get_logger().info(
            f"IMU 判定: {verdict.result} ({verdict.reason}) bias={self._imu_bias:.3f} "
            f"maxwz={self._imu_max_wz}")
        self._publish_final("IMU", verdict)
        self._emit_result("IMU", verdict)

    # ── LIDAR 項目 ───────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self._scan_alive = True
        self._scan_last_ms = self._now_ms()
        stamp = msg.header.stamp
        if self._scan_prev_stamp is not None:
            dt = (stamp.sec - self._scan_prev_stamp.sec) + \
                 (stamp.nanosec - self._scan_prev_stamp.nanosec) / 1.0e9
            if 0.0 < dt < 5.0:
                self._scan_period = dt
        self._scan_prev_stamp = stamp
        self._scan_ranges = list(msg.ranges)
        if msg.angle_increment > 0.0:
            self._scan_angle_inc = msg.angle_increment
        if self._item == "LIDAR":
            self._update_lidar_verdict()

    def _lidar_tick(self):
        if self._item != "LIDAR" or not self._scan_alive:
            return
        if self._now_ms() - self._scan_last_ms > self._p.scan_stale_ms:
            self._scan_alive = False
            self.get_logger().warn("/scan 途絶（LIDAR 項目）")
            self._update_lidar_verdict()

    def _configured_blind(self):
        value = self.get_parameter("blind_angle_ranges").value
        return [float(v) for v in (value or [])]

    def _update_lidar_verdict(self):
        verdict = judge_lidar(
            alive=self._scan_alive,
            period_s=self._scan_period,
            ranges=self._scan_ranges,
            angle_increment_deg=math.degrees(self._scan_angle_inc),
            configured_ranges=self._configured_blind(),
            p=self._p)
        key = (verdict.result, verdict.reason)
        if key != self._last_lidar_verdict:
            self._last_lidar_verdict = key
            self._publish_final("LIDAR", verdict)
            self._emit_result("LIDAR", verdict)

    # ── 結果の出力 ───────────────────────────────────────
    def _emit_result(self, item: str, verdict):
        arg_json = json.dumps(
            {"item": item, "result": verdict_to_fsm_result(verdict)})
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = "evt.check_result"
        ev.source_node = "opcheck_runner"
        ev.arg_json = arg_json
        self._pub_event.publish(ev)
        self.get_logger().info(f"/system/event 発行: evt.check_result {arg_json}")

    def _publish_final(self, item: str, verdict):
        next_screen = _NEXT_SCREEN.get(item, "")
        if next_screen and verdict.result not in ("NG", "WARN"):
            next_screen = ""
        if next_screen == "imu_calib" and verdict.result == "NG":
            next_screen = "repair"
        self._publish_status(result=verdict.result, detail=verdict.reason,
                             next_screen=next_screen)

    def _publish_status(self, result: str = "UNKNOWN", detail: str = "",
                        next_screen: str = ""):
        st = CheckStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.item = self._item or ""
        st.result = result
        st.detail = detail
        st.next_screen = next_screen
        self._pub_status.publish(st)

    # ── 定期モニター（10 Hz）─────────────────────────────
    def _monitor_tick(self):
        if self._item is None:
            return
        self._estop_tick()
        self._imu_tick()
        self._lidar_tick()
        self._publish_status(result="UNKNOWN", detail=self._live_detail())

    def _live_detail(self) -> str:
        item = self._item
        if item == "MOTOR":
            cmd_l, cmd_r = self._cmd_last
            n = len(self._motor_samples)
            return (f"hold={self._hold_val} "
                    f"L {cmd_l:.2f} / R {cmd_r:.2f} サンプル {n}")
        if item == "IMU":
            return (f"alive={int(self._imu_alive)} "
                    f"bias={self._imu_bias:.3f} maxwz={self._imu_max_wz} "
                    f"calib={self._imu_calib}")
        if item == "ESTOP":
            return (f"alive={int(self._estop_alive)} pressed={self._estop_pressed} "
                    f"press={int(self._estop_saw_press)} release={int(self._estop_saw_release)}")
        if item == "LIDAR":
            gap = self._scan_period
            return (f"alive={int(self._scan_alive)} "
                    f"period={gap if gap is None else round(gap, 3)}s "
                    f"rays={len(self._scan_ranges)}")
        return ""


def main(args=None):
    rclpy.init(args=args)
    node = OpcheckRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # SIGTERM 時は既定シグナルハンドラが context を落としている
            rclpy.shutdown()


if __name__ == "__main__":
    main()