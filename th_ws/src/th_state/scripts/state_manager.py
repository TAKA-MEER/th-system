#!/usr/bin/env python3
"""state_manager.py — state_core を ROS2 に繋ぐ薄いノード（WP-STATE-02）。

DetailedDesign-state.md §2 のとおり、遷移の判断は `state_core.StateCore` が持つ。
このノードは次の 6 つ **だけ** を行う（DetailedDesign-wp1.md WP-STATE-02 §4.2）。

  1. 入力を `Context` に詰める
  2. `StateCore.step()` を呼ぶ
  3. `effects` を state.md §3.3 の宛先表に従って配送する（`th_state` 自身の分はここで実行）
  4. `prev_mode` / `prev_state` / `prev_sub` のラッチを保持する
  5. `sys.jog_lease_expired` / `sys.link_timeout` をタイマで生成する
  6. `/system/state` を publish する

★ N-1（最重要）: このファイルにモード名の文字列比較を **1 つも書かない**。
  書けば完了条件①（AST 検査）が落ちる。起動時モードは `state_core.BOOT_MODE` を
  import して使う（リテラルをここに書かない）。
"""
import json
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy)

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from th_system_msgs.msg import ActiveScreen, FaultStatus, StateEvent, SystemState
from th_system_msgs.srv import SetFlag, UiTrigger

from th_state import guards as guards_module
from th_state.state_core import BOOT_MODE, Context, StateCore
from th_state.zones import ScreenInput, derive_limits

# ============================================================
# state.md §3.3 の effect 宛先表（転記。ログ用）。
# th_state 自身が処理するもの（SELF_EFFECTS）以外は、宛先ノードがまだ存在しないため
# 「ログに残して捨てる」（WP-STATE-02 §11 既知の負債）。
# ============================================================
_EFFECT_DESTINATIONS = {
    "abort_check": "opcheck_runner",
    "set_target": "person_tracker",
    "clear_selection": "person_tracker",
    "face_target": "follow_runner",
    "open_window": "WebUI",
    "close_window": "WebUI",
    "guide": "WebUI",
    "ask_save_if_unsaved": "WebUI",
    "ask_save": "WebUI",
    "enable_main_menu": "WebUI",
    "show_resume": "WebUI",
    "restart_control_stack": "connectivity_checker",
    "start_record": "route_recorder",
    "resume_record": "route_recorder",
    "finalize_route": "route_recorder",
    "load_route": "replay_runner",
    "rotate_to_start_yaw": "replay_runner",
    "resume_path": "replay_runner",
    "widen_search": "replay_runner",
    "global_localize": "replay_runner",
    "commit_map_patch": "map_session",
    "commit_venue_map": "map_session",
    "begin_two_point": "pin_registrar",
    "place_pin": "pin_registrar",
    "reject_register": "pin_registrar",
    "disable_jog_ui": "WebUI+jog_gate",
    "enable_jog_ui": "WebUI+jog_gate",
    "cancel_follow_path": "venue_navigator",
    "resume_follow_path": "venue_navigator",
    "replan": "venue_navigator",
    "start_monitor": "opcheck_runner",
    "offer_calib": "WebUI",
    "offer_opcheck": "WebUI",
    "record_result": "opcheck_runner",
    "feed_check_input": "opcheck_runner",
    "begin_wizard": "calib_runner",
    "run_measurement": "calib_runner",
    "build_preview": "calib_runner",
    "apply_and_verify": "calib_runner",
    "commit_calib": "calib_runner",
    "revert_calib": "calib_runner",
    "discard_calib": "calib_runner",
}

# /system/set_flag で外から変更してよいフラグ。jog_active はリース方式（§5）で
# th_state が自分で管理するので対象外（Spec の「ui.jog.hold」経由でしか動かさない）。
_SETTABLE_FLAGS = ("working", "map_update", "tracker_enabled", "auto_brake")

# state.md §3.1 の固定レート。registry.yaml の tunable パラメータ（R2 対象）ではなく
# インターフェース契約そのものの値なので、ここでは定数として扱う。
_STATE_PUBLISH_PERIOD_S = 0.1  # 10 Hz


def _fault_event_name(active: bool, severity: str) -> str:
    """names.md §8.3: /safety/fault の active/severity から内部事象名を決める。"""
    if not active:
        return "fault.cleared"
    if severity == "CRITICAL":
        return "fault.critical"
    return "fault.recoverable"


def _ms_to_time(ms: int) -> TimeMsg:
    t = TimeMsg()
    t.sec = ms // 1000
    t.nanosec = (ms % 1000) * 1_000_000
    return t


def _stamp_to_ms(stamp) -> int:
    return stamp.sec * 1000 + stamp.nanosec // 1_000_000


class StateManager(Node):

    def __init__(self, **kwargs):
        # kwargs はそのまま rclpy.node.Node へ渡す（例: テストから
        # parameter_overrides=[...] を注入するため。WP-STATE-02 §11 の
        # 「テストではパラメータを直接注入する」を満たす）。
        super().__init__('state_manager', **kwargs)

        # --- R2: 裸の数値を書かない。4 つとも既定値なしで宣言し、外部から必ず渡す ---
        self.declare_parameter('jog_lease_ms', Parameter.Type.INTEGER)
        self.declare_parameter('link_wait_timeout_ms', Parameter.Type.INTEGER)
        self.declare_parameter('ui_active_window_s', Parameter.Type.INTEGER)
        self.declare_parameter('screen_stale_ms', Parameter.Type.INTEGER)

        transitions, attributes, mode_entry = self._load_config()
        guards = guards_module.build_guards(mode_entry)
        self.core = StateCore(transitions, mode_entry, attributes, guards)

        errors = self.core.validate()
        if errors:
            for e in errors:
                self.get_logger().fatal(e)
            raise RuntimeError(
                f"th_state の設定 (transitions/attributes/mode_entry) が不正: {len(errors)} 件")

        # --- ラッチ・フラグ類（ノードが保持する。§2） ---
        self.mode = BOOT_MODE
        self.state = self.core.initial_state(self.mode)
        self.prev_mode = ""
        self.prev_state = ""
        self.prev_sub = ""
        self._jog_active = False
        self._flags = {
            "working": False,
            "map_update": False,
            "tracker_enabled": False,
            "auto_brake": True,   # 6.2 フェイルセーフ既定
        }
        self._fault_active = False
        self._fault_severity = ""
        self._fault_type = ""
        self._hw_estop = False
        self._ui_estop = False
        self._screens = {}          # client_id -> ScreenInput
        self._unsaved = []          # このパケットでは常に空（記録系ノードは未実装）
        self._last_event = ""
        self._last_reject_reason = ""

        now = self._now_ms()
        self._boot_ms = now
        self._since_ms = now
        self._last_jog_ms = now
        self._last_screen_msg_ms = now
        self._link_timeout_fired = False

        self._self_effect_handlers = {
            "set_jog": self._eff_set_jog,
            "latch_prev": self._eff_latch_prev,
            "clear_prev": self._eff_clear_prev,
            "set_screen": self._eff_set_screen,
            "mark_arrived": self._eff_mark_arrived,
            "keep_all": self._eff_keep_all,
        }

        self._setup_io()
        self._publish_state()  # N-5: 起動直後から出す

    # ------------------------------------------------------------
    # 起動時セットアップ
    # ------------------------------------------------------------
    def _load_config(self):
        config_dir = os.path.join(get_package_share_directory('th_state'), 'config')
        with open(os.path.join(config_dir, 'transitions.yaml'), encoding='utf-8') as f:
            transitions = yaml.safe_load(f)
        with open(os.path.join(config_dir, 'attributes.yaml'), encoding='utf-8') as f:
            attributes = yaml.safe_load(f)
        with open(os.path.join(config_dir, 'mode_entry.yaml'), encoding='utf-8') as f:
            mode_entry = yaml.safe_load(f)
        return transitions, attributes, mode_entry

    def _setup_io(self):
        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST)
        self._pub_state = self.create_publisher(SystemState, '/system/state', state_qos)

        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(StateEvent, '/system/event', self._on_event, event_qos)

        screen_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(
            ActiveScreen, '/ui/active_screen', self._on_active_screen, screen_qos)

        jog_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/ui/jog_lease', self._on_jog_lease, jog_qos)

        fault_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(FaultStatus, '/safety/fault', self._on_fault, fault_qos)

        bool_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/safety/estop_hw', self._on_estop_hw, bool_qos)
        self.create_subscription(Bool, '/safety/estop_ui', self._on_estop_ui, bool_qos)

        self.create_service(UiTrigger, '/system/trigger', self._on_trigger)
        self.create_service(SetFlag, '/system/set_flag', self._on_set_flag)
        self.create_service(Trigger, '/shutdown/prepare', self._on_shutdown_prepare)
        self.create_service(Trigger, '/shutdown/execute', self._on_shutdown_execute)

        self.create_timer(_STATE_PUBLISH_PERIOD_S, self._on_timer)

    # ------------------------------------------------------------
    # 責務1・2・3・4: Context を詰めて step() を呼び、effects を配送してラッチする
    # ------------------------------------------------------------
    def _now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def _current_limits(self):
        """derive_limits() を1回だけ呼ぶ（N-13: zone と speed_limit を別呼び出しで
        求めると、途中で now_ms が動いて別時刻の結果になりうる）。呼び出し側は
        必要なフィールド（.zone / .speed_limit）をこの1回の結果から取り出すこと。"""
        window_s = self.get_parameter('ui_active_window_s').value
        return derive_limits(self._screens, self._now_ms(), window_s)

    def _build_context(self, arg: dict) -> Context:
        flags = dict(self._flags)
        flags["jog_active"] = self._jog_active
        return Context(
            prev_mode=self.prev_mode,
            prev_state=self.prev_state,
            prev_sub=self.prev_sub,
            flags=flags,
            zone=self._current_limits().zone,
            candidate_count=0,
            target_selected=False,
            target_confident=False,
            fault_active=self._fault_active,
            fault_severity=self._fault_severity,
            fault_type=self._fault_type,
            hw_estop=self._hw_estop,
            ui_estop=self._ui_estop,
            route_ids=(),
            pin_kinds=(),
            leash_present=False,
            leash_taut=False,
            line_visible=False,
            camera_present=False,
            check_item="",
            check_result="",
            calib_item="",
            calib_preview_sane=False,
            map_update_available=False,
            now_ms=self._now_ms(),
            arg=arg or {},
        )

    def _process(self, event: str, arg: dict, requester: str):
        """§4.2 の6責務のうち 1〜4。呼び出し元が publish する（責務6）。"""
        # N-2: リース時刻は accepted の採否と無関係に更新する（§5・パケット§4.2の例示コード）。
        if event == "ui.jog.hold":
            self._last_jog_ms = self._now_ms()

        ctx = self._build_context(arg)
        decision = self.core.step(self.mode, self.state, event, ctx)

        # prev_sub のラッチ（§7 の latch_prev とは別物。PAUSE に新規で入るときだけ記録する。
        # PREP が PAUSE から $prev_sub で戻れるようにするための一般化ルール。
        # mode 名を見ずに to_state だけで判定するので N-1 に触れない）。
        if decision.to_state == "PAUSE" and self.state != "PAUSE":
            self.prev_sub = self.state

        self._apply_effects(decision.effects, requester)

        self._last_reject_reason = "" if decision.accepted else decision.reject_reason_key

        self.mode = decision.to_mode
        self.state = decision.to_state

        return decision

    def _apply_effects(self, effects, requester: str):
        for eff in effects:
            handler = self._self_effect_handlers.get(eff.name)
            if handler is not None:
                handler(eff.args, requester)
                continue
            dest = _EFFECT_DESTINATIONS.get(eff.name, "不明")
            self.get_logger().info(
                f"effect '{eff.name}' args={eff.args} -> 宛先 '{dest}' は未実装のため "
                "配送せず捨てる（DetailedDesign-wp1.md WP-STATE-02 §11 既知の負債）")

    # --- self effects（th_state 自身が処理する。state.md §3.3） ---
    def _eff_set_jog(self, args, requester):
        self._jog_active = bool(args.get('on'))

    def _eff_latch_prev(self, args, requester):
        # mode ∈ {ESTOP, CARRY} での抑制は StateCore._decide() が既に行っている（N-3）。
        # ここでは pre-transition の self.mode/self.state をそのまま記録するだけでよい。
        self.prev_mode = self.mode
        self.prev_state = self.state

    def _eff_clear_prev(self, args, requester):
        self.prev_mode = ""
        self.prev_state = ""

    def _eff_set_screen(self, args, requester):
        # ui.screen トリガ（/system/trigger、requester を client_id とみなす）による
        # ゾーン再計算の宣言。/ui/active_screen トピックと同じ辞書を更新する。
        screen_id = args.get('screen_id')
        if not screen_id or not requester:
            return
        now = self._now_ms()
        self._screens[requester] = ScreenInput(
            screen_id=screen_id, interacting=True, last_input_ms=now)
        self._last_screen_msg_ms = now

    def _eff_mark_arrived(self, args, requester):
        kind = args.get('kind') or ""
        self._last_event = f"evt.arrived:{kind}" if kind else "evt.arrived"

    def _eff_keep_all(self, args, requester):
        pass  # 意図的な no-op（state.md §3.3: 「何もしない」）。

    # ------------------------------------------------------------
    # 入口: /system/trigger（ui.* のみ）
    # ------------------------------------------------------------
    def _on_trigger(self, req, res):
        if not req.trigger.startswith("ui."):
            res.accepted = False
            res.reject_reason_key = "not_allowed"
            res.mode = self.mode
            res.state = self.state
            return res

        arg = self._parse_arg_json(req.arg_json)
        decision = self._process(req.trigger, arg, req.requester)
        self._publish_state()

        res.accepted = decision.accepted
        res.reject_reason_key = decision.reject_reason_key
        res.mode = self.mode
        res.state = self.state
        return res

    # ------------------------------------------------------------
    # 入口: /system/event（evt.* のみ）
    # ------------------------------------------------------------
    def _on_event(self, msg):
        if not msg.event.startswith("evt."):
            self.get_logger().warn(
                f"/system/event に名前空間外のイベント '{msg.event}' "
                f"(source={msg.source_node}) が来た。名前空間の分離により無視する（§11-7）")
            return
        arg = self._parse_arg_json(msg.arg_json)
        self._process(msg.event, arg, msg.source_node)
        self._publish_state()

    # ------------------------------------------------------------
    # 入口: /ui/jog_lease（ui.jog.hold のリース。§8.4）
    # ------------------------------------------------------------
    def _on_jog_lease(self, msg):
        self._process("ui.jog.hold", {}, msg.data)
        self._publish_state()

    # ------------------------------------------------------------
    # /ui/active_screen（ゾーン用の使用中トラッキング。イベントではない）
    # ------------------------------------------------------------
    def _on_active_screen(self, msg):
        self._screens[msg.client_id] = ScreenInput(
            screen_id=msg.screen_id,
            interacting=msg.interacting,
            last_input_ms=_stamp_to_ms(msg.last_input))
        self._last_screen_msg_ms = self._now_ms()

    # ------------------------------------------------------------
    # /safety/* の購読 → fault.* / hw.* / ui.estop.* の内部生成
    # ------------------------------------------------------------
    def _on_fault(self, msg):
        self._fault_active = msg.active
        self._fault_severity = msg.severity
        self._fault_type = msg.fault_type
        event = _fault_event_name(msg.active, msg.severity)
        self._process(event, {}, "")
        self._publish_state()

    def _on_estop_hw(self, msg):
        if bool(msg.data) == self._hw_estop:
            return
        self._hw_estop = bool(msg.data)
        event = "hw.estop.press" if self._hw_estop else "hw.estop.release"
        self._process(event, {}, "")
        self._publish_state()

    def _on_estop_ui(self, msg):
        if bool(msg.data) == self._ui_estop:
            return
        self._ui_estop = bool(msg.data)
        event = "ui.estop.press" if self._ui_estop else "ui.estop.release"
        self._process(event, {}, "")
        self._publish_state()

    # ------------------------------------------------------------
    # /system/set_flag
    # ------------------------------------------------------------
    def _on_set_flag(self, req, res):
        if req.flag not in _SETTABLE_FLAGS:
            res.accepted = False
            res.reject_reason_key = "not_allowed"
            return res
        self._flags[req.flag] = bool(req.value)
        res.accepted = True
        res.reject_reason_key = ""
        self._publish_state()
        return res

    # ------------------------------------------------------------
    # /shutdown/prepare, /shutdown/execute（state.md §3.2。詳細手順は §12.5 の対象外）
    # ------------------------------------------------------------
    def _on_shutdown_prepare(self, req, res):
        res.success = True
        res.message = json.dumps(self._unsaved)
        return res

    def _on_shutdown_execute(self, req, res):
        if self._unsaved:
            res.success = False
            res.message = "unsaved_remains"
            return res
        res.success = True
        res.message = ""
        return res

    # ------------------------------------------------------------
    # 責務5: sys.* のタイマ生成 ／ 責務6: publish
    # ------------------------------------------------------------
    def _on_timer(self):
        now = self._now_ms()

        if not self._link_timeout_fired:
            timeout_ms = self.get_parameter('link_wait_timeout_ms').value
            if now - self._boot_ms >= timeout_ms:
                self._link_timeout_fired = True
                self._process("sys.link_timeout", {}, "")

        if self._jog_active:
            lease_ms = self.get_parameter('jog_lease_ms').value
            if now - self._last_jog_ms >= lease_ms:
                self._process("sys.jog_lease_expired", {}, "")

        if self._screens:
            stale_ms = self.get_parameter('screen_stale_ms').value
            if now - self._last_screen_msg_ms >= stale_ms:
                self._screens = {}

        self._publish_state()

    def _publish_state(self):
        if (self.mode, self.state) != getattr(self, "_last_published_mode_state", (None, None)):
            self._since_ms = self._now_ms()
            self._last_published_mode_state = (self.mode, self.state)

        msg = SystemState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        msg.state = self.state
        msg.prev_mode = self.prev_mode
        msg.prev_state = self.prev_state
        limits = self._current_limits()
        msg.zone = limits.zone
        msg.speed_limit = limits.speed_limit
        msg.jog_active = self._jog_active
        msg.estop_ui = self._ui_estop
        msg.estop_hw = self._hw_estop
        msg.tracker_enabled = self._flags["tracker_enabled"]
        msg.auto_brake = self._flags["auto_brake"]
        msg.working = self._flags["working"]
        msg.map_update = self._flags["map_update"]
        msg.unsaved = list(self._unsaved)
        msg.since = _ms_to_time(self._since_ms)
        msg.last_event = self._last_event
        msg.last_reject_reason = self._last_reject_reason
        self._pub_state.publish(msg)

    @staticmethod
    def _parse_arg_json(arg_json: str) -> dict:
        if not arg_json:
            return {}
        try:
            parsed = json.loads(arg_json)
        except (json.JSONDecodeError, TypeError):
            return {}
        return parsed if isinstance(parsed, dict) else {}


def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
