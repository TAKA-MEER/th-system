"""measure_ledger.py — 測定機能の指示の結果台帳（純粋コア）。

正本は docs/plan/detailed/DetailedDesign-measure.md §3.1・§3.2・§6.1。

- 受けた `command_id` を記録し、`/measure/result` を **1 指示につき必ず
  1 回だけ**出す（`accept`／`complete(result, reason)`／`cancel_all(reason)`。
  二重の complete は無視して記録する）。
- すぐ終わる指示（`DONE` を即出すもの。§3.1 の「終わりの結果」列）は
  `IMMEDIATE_MEASURE_COMMANDS` に集約し、呼び出し側（node）が受理後に
  `complete(..., "DONE")` を呼べるようにする。
- `/measure/command` の判定のうち純粋な部分（① `bad_argument`。§6.1 の表）
  もここに置く。ROS を import しない（`th_state/state_core.py` と同じ制約）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

# §3.1 の表の `meas.*`（names.md §8.5 と同じ集合）。
KNOWN_MEASURE_COMMANDS = frozenset({
    "meas.goal",
    "meas.reroute",
    "meas.start",
    "meas.stop",
    "meas.abort",
    "meas.finish",
    "meas.turn",
    "meas.translate",
    "meas.set_limits",
    "meas.done",
})

# §3.1 の「終わりの結果」列が「すぐ DONE」のもの。
IMMEDIATE_MEASURE_COMMANDS = frozenset({
    "meas.reroute",
    "meas.start",
    "meas.stop",
    "meas.abort",
    "meas.finish",
    "meas.set_limits",
    "meas.done",
})

# `meas.translate` の `side` の値（§3.1。LEFT / RIGHT は 2026-11-15 に廃止）。
TRANSLATE_SIDES = frozenset({"FORWARD", "BACKWARD"})

RESULT_ARRIVED = "ARRIVED"
RESULT_DONE = "DONE"
RESULT_CANCELLED = "CANCELLED"


def validate_measure_command(command_id: str, command: str, side: str = "") -> str:
    """① の判定。問題があれば `bad_argument` を返す（§6.1 の表の順）。

    `command` が `meas.` で始まらない（`ui.*` を含む）・`command_id` が空・
    未知の `meas.*`・`meas.translate` の `side` が FORWARD／BACKWARD 以外。
    通れば ""。
    """
    if not command or not command.startswith("meas."):
        return "bad_argument"
    if not command_id:
        return "bad_argument"
    if command not in KNOWN_MEASURE_COMMANDS:
        return "bad_argument"
    if command == "meas.translate" and side not in TRANSLATE_SIDES:
        return "bad_argument"
    return ""


@dataclass(frozen=True)
class ResultRecord:
    """`/measure/result` 1 件分（ROS 非依存の形）。"""

    command_id: str
    command: str
    result: str
    reason: str = ""


@dataclass
class MeasureLedger:
    """受けた指示の台帳。出す結果はちょうど 1 回だけ。"""

    _pending: dict = field(default_factory=dict)      # command_id -> command
    _completed: set = field(default_factory=set)      # 結果を出した command_id
    _ignored: list = field(default_factory=list)      # 無視した complete の記録

    @property
    def pending_command_id(self) -> str:
        for command_id in self._pending:
            return command_id
        return ""

    @property
    def pending_command(self) -> str:
        for command in self._pending.values():
            return command
        return ""

    @property
    def ignored(self) -> list:
        return list(self._ignored)

    def accept(self, command_id: str, command: str) -> bool:
        """指示を受けた記録。すでに進行中・完了済みの id は受けない。"""
        if command_id in self._pending or command_id in self._completed:
            return False
        self._pending[command_id] = command
        return True

    def complete(self, command_id: str, result: str, reason: str = ""):
        """終わりの結果を出す。出すものがあれば `ResultRecord`、無ければ None。

        二重の complete（完了済み・未知の id）は無視して記録する（§3.2 の
        「必ず 1 回だけ」）。
        """
        command = self._pending.pop(command_id, None)
        if command is None:
            self._ignored.append(
                ResultRecord(command_id=command_id, command="",
                             result=result, reason=reason))
            return None
        self._completed.add(command_id)
        return ResultRecord(command_id=command_id, command=command,
                            result=result, reason=reason)

    def cancel_all(self, reason: str) -> list:
        """進行中の指示をすべて `CANCELLED` で終える（§3.2 の表）。"""
        out = []
        for command_id, command in list(self._pending.items()):
            self._pending.pop(command_id)
            self._completed.add(command_id)
            out.append(ResultRecord(command_id=command_id, command=command,
                                    result=RESULT_CANCELLED, reason=reason))
        return out
