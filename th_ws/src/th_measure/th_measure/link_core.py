"""link_core.py — 測定機能（fk-system）との生存確認の純粋コア。

正本は docs/plan/detailed/DetailedDesign-measure.md §3.4・§6.1。

- `/measure/heartbeat` のうち `interface_version == "1"` のものだけを受信に数える
  （版が食い違うものは数えない。§1.2 の版照合）。
- 最後に数えた受信から `measure_link_timeout_ms` を超えたら途絶。
- 途絶理由は `never`（一度も版一致を受信していない）／`timeout`（版一致を
  受けたあと途絶）／`version_mismatch`（版一致は一度も無く、版不一致だけ来た）。
- `/measure/status` の速度上限の欄の組み立て（`speed_limit` 名→数値、
  `stop`→0。§6.1。`v_cap` を混ぜるのは P5c）。

ROS2 を import しない（`th_state/state_core.py` と同じ制約）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

INTERFACE_VERSION = "1"

REASON_NEVER = "never"
REASON_TIMEOUT = "timeout"
REASON_VERSION_MISMATCH = "version_mismatch"
REASON_OK = "ok"


@dataclass
class LinkTracker:
    """受信時刻（ms。呼び出し側の時計）だけから生存判定を行う。"""

    timeout_ms: int
    _last_counted_ms: int | None = None
    _saw_mismatch: bool = False

    def on_heartbeat(self, interface_version: str, now_ms: int) -> bool:
        """heartbeat 1 件分。数えたら True。版不一致は数えず、記録だけする。"""
        if interface_version == INTERFACE_VERSION:
            self._last_counted_ms = now_ms
            return True
        self._saw_mismatch = True
        return False

    @property
    def ever_counted(self) -> bool:
        return self._last_counted_ms is not None

    def link_ok(self, now_ms: int) -> bool:
        if self._last_counted_ms is None:
            return False
        return (now_ms - self._last_counted_ms) <= self.timeout_ms

    def reason(self, now_ms: int) -> str:
        """いまの途絶理由。生きていれば `ok`。"""
        if self.link_ok(now_ms):
            return REASON_OK
        if self._last_counted_ms is not None:
            return REASON_TIMEOUT
        if self._saw_mismatch:
            return REASON_VERSION_MISMATCH
        return REASON_NEVER


def resolve_speed_limit(name: str, table: dict) -> float:
    """`speed_limit` の名前 → 数値（m/s）。

    `stop` と表に無い名前は 0（安全側。`obstacle_limiter` の
    `resolve_speed_limit_name` と同じ倒し方）。表の値は
    `measure_link` の生成 yaml 由来の速度パラメータ。
    """
    if name == "stop":
        return 0.0
    try:
        return float(table[name])
    except (KeyError, TypeError, ValueError):
        return 0.0


def effective_v_limit(speed_limit_name: str, table: dict, v_cap_mps: float) -> float:
    """`v_limit_mps`（§3.3・§6.6）。本システムの上限と `v_cap` の小さい方。

    8-g P5c: 指定がある（正）ときだけ min を採る。上げる方向には効かない。"""
    system = resolve_speed_limit(speed_limit_name, table)
    if v_cap_mps and v_cap_mps > 0:
        return min(system, float(v_cap_mps))
    return system


def effective_w_limit(w_cap_radps: float, w_max_radps: float = 0.0) -> float:
    """`w_limit_radps`（§3.3・§6.6）。min(本システムの `w_max`, `w_cap`)。

    指定が無い（0）ときは本システムの上限だけを返す。`w_max_radps` を
    渡さない旧呼び出しでは、指定値をそのまま返す（後方互換）。"""
    if w_max_radps and w_max_radps > 0:
        if w_cap_radps and w_cap_radps > 0:
            return min(float(w_max_radps), float(w_cap_radps))
        return float(w_max_radps)
    if w_cap_radps and w_cap_radps > 0:
        return float(w_cap_radps)
    return 0.0


@dataclass
class StatusFields:
    """`/measure/status` を組み立てるための材料（§6.1 の表）。"""

    mode: str = ""
    state: str = ""
    working: bool = False
    link_ok: bool = False
    pose_valid: bool = False
    map_instance_id: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    pending_command_id: str = ""
    pending_command: str = ""
    pause_reason: str = ""
    blocked: bool = False
    reentry_required: bool = False
    speed_limit_name: str = "stop"
    speed_table: dict = field(default_factory=dict)
    v_cap_mps: float = 0.0
    w_cap_radps: float = 0.0
    w_max_radps: float = 0.0

    def as_dict(self) -> dict:
        return {
            "interface_version": INTERFACE_VERSION,
            "mode": self.mode,
            "state": self.state,
            "working": bool(self.working),
            "link_ok": bool(self.link_ok),
            "pose_valid": bool(self.pose_valid),
            "map_instance_id": self.map_instance_id,
            "x": float(self.x),
            "y": float(self.y),
            "yaw": float(self.yaw),
            "pending_command_id": self.pending_command_id,
            "pending_command": self.pending_command,
            "pause_reason": self.pause_reason,
            "blocked": bool(self.blocked),
            "reentry_required": bool(self.reentry_required),
            "v_limit_mps": effective_v_limit(
                self.speed_limit_name, self.speed_table, self.v_cap_mps),
            "w_limit_radps": effective_w_limit(self.w_cap_radps, self.w_max_radps),
        }
