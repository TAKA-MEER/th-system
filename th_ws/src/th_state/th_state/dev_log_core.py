"""dev_log_core.py — 開発モード「ログの選択記録」の ROS2 非依存の純粋コア（WP-DEV-01C）。

rclpy を import しない（`th_testing` から直接 pytest できる。connectivity_core.py /
localization_health_core.py と同じ制約・流儀）。

Spec-webui.md §5「ログの選択記録」: **選択した特定のログのみ**を、
タイムスタンプ付きで記録する。選択の正本は ROS 側のパラメータ
（`dev_log_state` / `dev_log_fault` / `dev_log_cmdvel`。既存の `dev_ignore_*` と
同じ流儀。`registry.yaml` には載せない）で、**開発モードが OFF のときは何も
記録しない**（通常運用で勝手にファイルが増えないこと）。

記録先は ROS のログ（`get_logger().info`）であり、`/root/th_data/` 配下の
ファイルには書かない。理由:
  1. ファイルだとローテーション・掃除・容量管理が要るが、ROS ログなら
     OFF で無音になるだけで通常運用の副作用が無い。
  2. 既存の開発モード ON/OFF ログ（`connectivity_checker._log_dev_state`）と
     同じ場所に残るので、「dev で通した結果か」の取り違え防止が 1 箇所で済む。
  3. `/root/th_data`（=`th_ws/data`）はバインドマウントでホストと共有しており、
     置きっぱなしの危険（brief の禁止事項）・root 所有化の問題を避けられる。
     長時間セッションの永続化が必要になったら別作業で足す。

書式は 1 行 1 イベントの JSON（`format_line`）。`t` に ISO8601 UTC の
タイムスタンプを必ず載せ（spec の要求）、残りは `kind` 別の固定キーなので
あとから機械で読める（`sort_keys=True` で安定化）。

量の考え方（候補の取捨選択。詳細はコミットメッセージ）:
  - 状態遷移・フォルト変化は変化時のみ（低頻度なのでそのまま出せる）。
  - `/cmd_vel` は 20 Hz のフルレートでは出さない。要約（1 Hz 上限＋変化ゲート
    のスナップショット）にする。`should_record_cmdvel` が間引きを決める。
  - `/scan`（720 点 @10 Hz）・wheel 系は対象外。量に対して「あとから追いたい」
    検証用途（何モードだったか・一瞬のフォルト・走行の概形）に要らない。
"""

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Dict, Optional

# 選択パラメータの項目名。`dev_log_<item>` に対応する。
# connectivity_checker.py の宣言・_DEV_LOG_ITEMS と揃えること。
LOG_ITEMS = ('state', 'fault', 'cmdvel')

# /cmd_vel 要約の間引き既定。数値リテラルを呼ばわり側に書かせないため
# （R2）ここに置く。呼び出し側は registry ではなくノード内定数として扱う
# （計測で詰める前の暫定値。registry.yaml には載せない）。
CMDVEL_MIN_INTERVAL_MS = 1000   # 変化があるときの最短間隔（1 Hz 上限）
CMDVEL_HEARTBEAT_MS = 10000     # 変化が無いときも「生きている」を示す最長間隔
CMDVEL_EPS_LINEAR = 0.01        # これ未満の linear.x 差は「同じ」とみなす [m/s]
CMDVEL_EPS_ANGULAR = 0.01       # これ未満の angular.z 差は「同じ」とみなす [rad/s]


@dataclass(frozen=True)
class StateSnap:
    """`/system/state` の記録に使う抜粋。mode/state の組が変わったら記録する。"""
    mode: str
    state: str


@dataclass(frozen=True)
class FaultSnap:
    """`/safety/fault` の記録に使う抜粋。3 項目のいずれかが変わったら記録する。"""
    active: bool
    fault_type: str
    severity: str


@dataclass(frozen=True)
class TwistSum:
    """`/cmd_vel` の要約。フル Twist ではなく前後・旋回だけ抜く。

    20 Hz の Twist 全量を出すと量が多いので（brief §1）、記録するのは
    この 2 要素のスナップショットだけにする。
    """
    v: float   # linear.x [m/s]
    w: float   # angular.z [rad/s]


def effective_selection(dev_mode: bool, selected: Dict[str, bool]) -> Dict[str, bool]:
    """実効選択 = マスタ（dev_mode）AND 項目別選択。

    OFF のときは何も記録しない（完了条件 2）。`_dev_effective()` と同じ形。
    """
    return {item: (bool(dev_mode) and bool(selected.get(item, False)))
            for item in LOG_ITEMS}


def state_changed(prev: Optional[StateSnap], curr: StateSnap) -> bool:
    """状態遷移の有無。初見（prev が無い）は「変わった」とみなす。"""
    if prev is None:
        return True
    return (prev.mode, prev.state) != (curr.mode, curr.state)


def fault_changed(prev: Optional[FaultSnap], curr: FaultSnap) -> bool:
    """フォルト変化の有無。発生・解除・種別/深刻度の切替をすべて拾う。"""
    if prev is None:
        return True
    return ((prev.active, prev.fault_type, prev.severity)
            != (curr.active, curr.fault_type, curr.severity))


def _twist_same(a: TwistSum, b: TwistSum) -> bool:
    return (abs(a.v - b.v) <= CMDVEL_EPS_LINEAR
            and abs(a.w - b.w) <= CMDVEL_EPS_ANGULAR)


def should_record_cmdvel(now_ms: int, last_emit_ms: Optional[int],
                         prev: Optional[TwistSum], curr: TwistSum) -> bool:
    """`/cmd_vel` 要約をいま出すか（間引き判定）。

    - 初回（prev/last_emit が無い）は出す。
    - 値が変わった（停止⇄走行の遷移を含む）: 最短間隔が空いたら出す。
    - 値が同じ（停止中のゼロ連打を含む）: ハートビート間隔が空くまで出さない。
    - 時刻が戻っている（時計の異常）: 出さない（安全側ではなく静音側。
      記録は安全動作ではないので、変な時刻の行を増やさない）。
    """
    if prev is None or last_emit_ms is None:
        return True
    elapsed = now_ms - last_emit_ms
    if elapsed < 0:
        return False
    if _twist_same(prev, curr):
        return elapsed >= CMDVEL_HEARTBEAT_MS
    return elapsed >= CMDVEL_MIN_INTERVAL_MS


def format_timestamp(now_ms: int) -> str:
    """ms  epoch → ISO8601 UTC（ミリ秒まで。`2026-09-20T12:34:56.789Z`）。"""
    dt = datetime.fromtimestamp(now_ms / 1000.0, tz=timezone.utc)
    return dt.strftime('%Y-%m-%dT%H:%M:%S.') + f'{dt.microsecond // 1000:03d}Z'


def format_line(now_ms: int, kind: str, payload: dict) -> str:
    """1 行 1 イベントの JSON。`t` と `kind` を必ず持ち、あとから機械で読める。

    `payload` のキーは kind 別の固定キー（state/fault/cmdvel/dev。呼び出し側が
    `StateSnap` 等から組み立てる）。時刻は引数で受け取る（試験で確定できるよう
    ノードの時計を読まない。localization_health_core.py と同じ流儀）。
    """
    body = {'t': format_timestamp(now_ms), 'kind': kind}
    body.update(payload)
    return json.dumps(body, sort_keys=True)
