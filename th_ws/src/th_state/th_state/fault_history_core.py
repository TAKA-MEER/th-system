"""fault_history_core.py — フォルト履歴の記録（Spec-safety.md §3.5.5）。ROS2 非依存の純コア。

入力は 2 つだけ（どちらも読むだけ。モード遷移・停止・再開には一切関与しない）:

  * 立っているフォルト種別の集合（`/safety/active_faults`）。集合の差分で
    種別ごとの「開始」「解消」を取る。`/safety/fault` は使わない: あちらは
    代表 1 件しか出さず、複数同時や重大継続中の回復フォルトが見えないため。
  * `/system/state` の mode・state・pause_reason。発生時のモード・状態と、
    「止まったか（outcome）」の判定に使う。

時刻はすべて呼び出し側が渡す「機体の時計（epoch 秒）」。メッセージの header.stamp は
使わない（受信時刻で足りる。ミリ秒の差）。

outcome の判定窓（OUTCOME_WINDOW_S は registry の fault_outcome_window_ms が正）:
  `state_manager` はフォルトを受けると直ちに PAUSE（pause_reason=fault）か ESTOP へ
  遷移する。遷移の通知は /safety/active_faults より少し遅れて届くことがあり、0.1 秒で
  解消するフォルトでも解消の **後** に届き得る。そこで
    * 継続中のフォルト: 開始から窓以内に届いた遷移の「エッジ」を結果とみなす
    * 解消済みのフォルト: 解消から窓以内に届いた遷移のエッジを結果とみなす
  とし、解消済みの記録は解消から窓が過ぎるまで確定させない（その間は outcome が
  更新され得る）。窓は 1 秒: 通常の遅れ（数十ミリ秒）の 10 倍以上で、無関係な
  ESTOP／PAUSE を拾う確率が小さい長さ。逆順（遷移が先に届く）も、直前の窓以内の
  エッジを覚えておいて拾う。
  「エッジ」とは「PAUSE(fault)／ESTOP でなかった状態からそうなった」こと。
  すでに止まっている最中に立ったフォルトは NONE（新たに止めたわけではない）。
  PAUSE のあとに ESTOP へ進んだら ESTOP に上書きする（重い方）。
"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# 画面に残す件数（Spec-safety.md §3.5.5 で固定）
HISTORY_MAX = 50

OUTCOME_NONE = 'NONE'
OUTCOME_PAUSE = 'PAUSE'
OUTCOME_ESTOP = 'ESTOP'


@dataclass
class _Rec:
    fault_type: str
    started_at: float
    severity: str
    mode: str
    state: str
    ended_at: Optional[float] = None
    outcome: str = OUTCOME_NONE
    finalized: bool = False

    def to_dict(self) -> dict:
        ongoing = self.ended_at is None
        return {
            'fault_type': self.fault_type,
            'started_at': self.started_at,
            'duration_s': -1.0 if ongoing else max(0.0, self.ended_at - self.started_at),
            'ongoing': ongoing,
            'mode': self.mode,
            'state': self.state,
            'severity': self.severity,
            'outcome': self.outcome,
        }


def _stop_kind(mode: str, state: str, pause_reason: str) -> str:
    """止まっている種類。'' なら止まっていない（フォルト由来でない PAUSE も含む）。"""
    if mode == 'ESTOP':
        return OUTCOME_ESTOP
    if state == 'PAUSE' and pause_reason == 'fault':
        return OUTCOME_PAUSE
    return ''


class FaultHistoryCore:

    def __init__(self, outcome_window_s: float, max_records: int = HISTORY_MAX):
        self._window = float(outcome_window_s)
        self._max = int(max_records)
        self._recs: List[_Rec] = []          # 古い順
        self._open: Dict[str, _Rec] = {}
        self._unfinal: List[_Rec] = []       # 解消済みで確定待ち
        self._writes: List[dict] = []
        self._mode = ''
        self._state = ''
        self._stop = ''
        # 直近の止まりエッジ: (時刻, 種類, エッジ直前のモード, 状態)
        self._last_edge: Optional[Tuple[float, str, str, str]] = None

    # ── 入力 ───────────────────────────────────────────
    def on_active(self, now: float, types: List[str], severities: List[str]) -> bool:
        sev = {t: (severities[i] if i < len(severities) else 'RECOVERABLE')
               for i, t in enumerate(types)}
        changed = False
        for t in sorted(sev):
            if t in self._open:
                continue
            mode, state, outcome = self._mode, self._state, OUTCOME_NONE
            e = self._last_edge
            if e is not None and now - e[0] <= self._window:
                # 遷移が先に届いた（逆順）。エッジ直前の状態を発生時の状態とする
                outcome, mode, state = e[1], e[2], e[3]
            rec = _Rec(t, now, sev[t], mode, state, outcome=outcome)
            self._open[t] = rec
            self._recs.append(rec)
            self._trim()
            self._writes.append(self._line(rec))
            changed = True
        for t in sorted(self._open):
            if t in sev:
                continue
            rec = self._open.pop(t)
            rec.ended_at = now
            self._unfinal.append(rec)
            changed = True
        return changed

    def on_state(self, now: float, mode: str, state: str, pause_reason: str) -> bool:
        stop = _stop_kind(mode, state, pause_reason)
        changed = False
        if stop and stop != self._stop and self._stop != OUTCOME_ESTOP:
            # エッジ（PAUSE→ESTOP も含む）
            self._last_edge = (now, stop, self._mode, self._state)
            for rec in self._recs:
                if rec.finalized or not self._eligible(rec, now):
                    continue
                if stop == OUTCOME_ESTOP or rec.outcome == OUTCOME_NONE:
                    if rec.outcome != stop:
                        rec.outcome = stop
                        changed = True
        elif stop and stop != self._stop:
            pass  # ESTOP → PAUSE(fault) は軽くなる方向。結果にしない
        self._mode, self._state, self._stop = mode, state, stop
        return changed

    def tick(self, now: float) -> None:
        """解消から窓が過ぎた記録を確定してファイル行にする。"""
        keep = []
        for rec in self._unfinal:
            if now - rec.ended_at > self._window:
                rec.finalized = True
                self._writes.append(self._line(rec))
            else:
                keep.append(rec)
        self._unfinal = keep

    # ── 出力 ───────────────────────────────────────────
    def records(self) -> List[dict]:
        """新しい順。最大 max_records 件。"""
        return [r.to_dict() for r in reversed(self._recs)]

    def drain_writes(self) -> List[dict]:
        w, self._writes = self._writes, []
        return w

    def flush_all(self) -> List[dict]:
        """終了時: 確定待ちを書く。継続中は ongoing のまま（開始時に書いた行が残る）。"""
        for rec in self._unfinal:
            rec.finalized = True
            self._writes.append(self._line(rec))
        self._unfinal = []
        return self.drain_writes()

    # ── 内部 ───────────────────────────────────────────
    def _eligible(self, rec: _Rec, now: float) -> bool:
        ref = rec.started_at if rec.ended_at is None else rec.ended_at
        return now - ref <= self._window

    def _trim(self) -> None:
        while len(self._recs) > self._max:
            old = self._recs.pop(0)
            # 画面から消えるだけ。確定待ちはファイルへ書く対象として残す
            if old.ended_at is None:
                self._open.pop(old.fault_type, None)

    @staticmethod
    def _line(rec: _Rec) -> dict:
        d = rec.to_dict()
        d['final'] = rec.finalized
        return d
