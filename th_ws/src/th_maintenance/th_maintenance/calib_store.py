"""calib_store.py — 校正の補正値の永続化（現在値＋履歴 3 世代。ROS2 非依存）。

`DetailedDesign-maintenance.md` §3.6 の写実。

```
<root>/                      既定 /root/th_data/calib
├── current.yaml             いま適用されている補正値（項目ごと）＋ robot_id
└── history/
    ├── LINEAR/001.yaml      新しい順に最大 3 世代（項目ごと）
    ├── LINEAR/002.yaml
    ├── LINEAR/003.yaml
    └── ROTATION/…
```

**履歴は項目ごとに持つ**（設計書の図は `history/001.yaml` だが、全項目共通の連番だと
IMU を 3 回校正しただけで直進の履歴が押し出される。「項目ごとに戻せる」を満たすため分けた）。

- 書き込みは一時ファイル → `os.replace`（途中で落ちても current.yaml が壊れない）。
- **確定（commit）は 1 回の呼び出しで完結する。** 途中状態を書く関数は無い。
  校正中の非常停止・フォルトで確定しない（§7 #3）のは、呼び出し側が commit を呼ばないことで
  成り立つ。ここには「確定しかけ」を残す API を置かない。
- ロールバックは「履歴の世代 N を current にし、置き換えられた current を履歴の先頭へ積む」。
  取り消しも効く（戻しすぎても元へ戻せる）。
"""
from __future__ import annotations

import copy
import os
from typing import Any, Dict, List, Optional

import yaml

MAX_GENERATIONS = 3


class CalibStore:
    def __init__(self, root: str, robot_id: str = ""):
        self._root = root
        self._robot_id = robot_id

    # ── パス ─────────────────────────────────────────────
    @property
    def _current_path(self) -> str:
        return os.path.join(self._root, "current.yaml")

    def _history_dir(self, item: str) -> str:
        return os.path.join(self._root, "history", item)

    def _history_path(self, item: str, generation: int) -> str:
        return os.path.join(self._history_dir(item), f"{generation:03d}.yaml")

    # ── 低水準 I/O ───────────────────────────────────────
    @staticmethod
    def _write_atomic(path: str, doc: Any) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)

    @staticmethod
    def _read(path: str) -> Optional[Any]:
        try:
            with open(path, encoding="utf-8") as f:
                return yaml.safe_load(f)
        except (OSError, yaml.YAMLError):
            return None

    # ── 現在値 ───────────────────────────────────────────
    def load_current(self) -> Dict[str, Any]:
        """`items`（項目名 → エントリ）。ファイルが無い・壊れていれば空。"""
        doc = self._read(self._current_path)
        if not isinstance(doc, dict):
            return {}
        items = doc.get("items")
        return dict(items) if isinstance(items, dict) else {}

    def robot_id(self) -> str:
        doc = self._read(self._current_path)
        if isinstance(doc, dict):
            return str(doc.get("robot_id") or "")
        return ""

    def _save_current(self, items: Dict[str, Any]) -> None:
        self._write_atomic(self._current_path,
                           {"robot_id": self._robot_id or self.robot_id(), "items": items})

    def current_entry(self, item: str) -> Optional[Dict[str, Any]]:
        entry = self.load_current().get(item)
        return dict(entry) if isinstance(entry, dict) else None

    # ── 履歴 ─────────────────────────────────────────────
    def history(self, item: str) -> List[Dict[str, Any]]:
        """新しい順（先頭が 1 世代前）。"""
        out: List[Dict[str, Any]] = []
        for gen in range(1, MAX_GENERATIONS + 1):
            entry = self._read(self._history_path(item, gen))
            if isinstance(entry, dict):
                out.append(entry)
        return out

    def _write_history(self, item: str, entries: List[Dict[str, Any]]) -> None:
        entries = entries[:MAX_GENERATIONS]
        for gen in range(1, MAX_GENERATIONS + 1):
            path = self._history_path(item, gen)
            if gen <= len(entries):
                self._write_atomic(path, entries[gen - 1])
            elif os.path.exists(path):
                os.remove(path)

    def _push_history(self, item: str, entry: Dict[str, Any]) -> None:
        self._write_history(item, [entry] + self.history(item))

    # ── 確定・ロールバック ───────────────────────────────
    def commit(self, item: str, values: Dict[str, float], calibrated_at: str,
               verification: Dict[str, Any], operator: str = "",
               params_digest: str = "") -> Dict[str, Any]:
        """校正結果を確定する。置き換えられる current は履歴の先頭へ積む。"""
        entry = {
            "values": {k: float(v) for k, v in values.items()},
            "calibrated_at": calibrated_at,
            "verification": copy.deepcopy(verification),
            "operator": operator,
            "params_digest": params_digest,
        }
        items = self.load_current()
        previous = items.get(item)
        if isinstance(previous, dict):
            self._push_history(item, previous)
        items[item] = entry
        self._save_current(items)
        return entry

    def rollback(self, item: str, generation: int) -> Optional[Dict[str, Any]]:
        """履歴の世代 `generation`（1＝1 つ前）を current にする。無ければ None（何も変えない）。

        置き換えられた current は履歴の先頭へ積む（取り消し可能）。他の項目は触らない。
        """
        hist = self.history(item)
        if generation < 1 or generation > len(hist):
            return None
        target = hist[generation - 1]
        items = self.load_current()
        previous = items.get(item)
        remaining = hist[:generation - 1] + hist[generation:]
        if isinstance(previous, dict):
            remaining = [previous] + remaining
        self._write_history(item, remaining)
        items[item] = target
        self._save_current(items)
        return dict(target)
