#!/usr/bin/env python3
"""params_audit.py — 監査ノード（WP-PARAM-02）。

DetailedDesign-params.md §5「②監査」の実体。**生成はしない**
（`registry.yaml` → ROS2 パラメータ YAML の生成は
`th_bringup/launch/params_generation.py` の `OpaqueFunction` が
ノード起動より前に済ませている。G-1）。

このノードの責務（§5 の「`params_audit` の責務」表）:
  - `/system/params_status`（ParamsStatus。transient_local）を publish
  - `/params/get` `/params/set` `/params/save` を提供
    （`/params/set` `/params/save` は停止中＝IDLE・PREP の地図作業中かつ
    非ジョグのときだけ受理。Spec.md SD-9。1b-5 で条件を SD-9 に寄せた。
    以前の IDLE/MANUAL 条件は MANUAL を許しており SD-9 に反していた）
  - 校正値の取り込み（`/root/th_data/calib/current.yaml`。§5 の責務表）

`/params/get` `/system/params_status` は「今読める最新の実効値」
（registry → calib → overrides の順に重ね書きした値。§5.3「読み込み順は 1→2→3」）を返す。
これは launch が起動時に一度だけ書いた `generated/*.yaml`（G-1 の時点のスナップショット）
とは別物になりうる ——
`/params/set` の受理後は generated/ を再生成せず（このパケットの実装範囲。後述）、
現在稼働中のノードは古い値のままになりうる（「要再起動」。§5.3.2）。
`ParamsStatus.msg` にはその状態を運ぶフィールドが無い（`placeholder_count` /
`placeholder_names` / `digest` のみ）ため、この情報は message や export.py へのログでしか
伝えられない。**message に明記する**（本ファイル `_cb_set` 参照）。

§5.3 現場調整の実装範囲について（完了報告に詳細）:
  実装した: PT-1（当てる前に検査。部分適用しない）／PT-2（given 以外は拒否）／
            PT-3（IDLE/MANUAL 以外は拒否）／PT-4（overrides.yaml に set_at/set_by/reason）
  実装しなかった: §5.3.2 の「走っているノードへ反映する」（set_parameters 経由のライブ反映）。
            停止中しか許可されない操作である以上、次回起動時の再生成で確実に反映される。
            ライブ反映は対象ノードの発見・失敗時のロールバックなど別途設計が要る範囲であり、
            このパケットの §2 に参照節が無い（R1）ため実装しない。常に「再起動が必要」として
            案内する（安全側 = 黙って古い値のまま走らせない）。
"""
from __future__ import annotations

import copy
import json
import os
import time
from datetime import datetime, timezone

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_srvs.srv import Trigger

from th_system_msgs.msg import ParamsStatus, SystemState
from th_system_msgs.srv import GetParams, SetParams

from th_config_manager.stop_only_guard import STATE_STALE_SEC, stop_only_allows
from th_params import export, schema

OVERRIDES_PATH = "/root/th_data/params/overrides.yaml"
CALIB_PATH = "/root/th_data/calib/current.yaml"
# SG-B8: 起動時の設定生成が書いた出どころ記録。/params/get が「いま効いている値
# （前回起動時に適用されたもの）」と「保存済みで次回から効く上書き値」を
# 区別するための正本（Spec-params.md §6）。
GENERATED_DIR = "/root/th_data/generated"
PROVENANCE_FILENAME = "params_provenance.json"


class ParamsAudit(Node):

    def __init__(self, **kwargs):
        # kwargs はそのまま rclpy.node.Node へ渡す（テストが parameter_overrides=[...] を
        # 注入できるようにする。state_manager.py / connectivity_checker.py と同じ流儀）。
        super().__init__("params_audit", **kwargs)

        # params_audit 自身のパラメータは持たない（registry のパスだけ。§3.3）。
        # D4: overrides_path / calib_path も registry_path と同じ範疇（値ではなく
        # ファイルの置き場所という「配線情報」）として追加する。テストが実データの
        # 置き場（/root/th_data/...）を書き換えないよう一時パスを注入できるようにする
        # ためで、既定は空文字列（空ならモジュール定数 OVERRIDES_PATH / CALIB_PATH を
        # 使う。registry_path と同じ流儀）。完了報告に「パケット §3.3 からの逸脱」と
        # して明記する。
        self.declare_parameter("registry_path", "")
        self.declare_parameter("overrides_path", "")
        self.declare_parameter("calib_path", "")
        # 1b-5: /system/state の鮮度の上限。registry の state_stale_ms=1500 と
        # 同じ既定（jog_gate / obstacle_limiter / safety_monitor と同じ値）。
        self.declare_parameter("state_stale_ms", int(STATE_STALE_SEC * 1000.0))
        # SG-B8: 生成物の置き場所（params_provenance.json を読むため）。
        # 既定は空文字列（空ならモジュール定数 GENERATED_DIR を使う。
        # registry_path と同じ流儀）。テストが一時パスを注入できるようにする。
        self.declare_parameter("generated_dir", "")

        # /system/state の最新像。未受信・古い間は拒否する（安全側。
        # 途絶時に /params/set を許してしまう方が危険なので、既知の許可条件だけを通す）。
        self._state_mode: str | None = None
        self._state_name: str = ""
        self._jog_active: bool = False
        self._state_at: float | None = None

        self._setup_io()
        self._publish_status()

    # ------------------------------------------------------------
    def _setup_io(self):
        state_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(SystemState, "/system/state", self._on_state, state_qos)

        status_qos = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_status = self.create_publisher(ParamsStatus, "/system/params_status", status_qos)

        self.create_service(GetParams, "/params/get", self._cb_get)
        self.create_service(SetParams, "/params/set", self._cb_set)
        self.create_service(Trigger, "/params/save", self._cb_save)

    def _on_state(self, msg: SystemState):
        self._state_mode = msg.mode
        self._state_name = msg.state
        self._jog_active = bool(msg.jog_active)
        self._state_at = time.monotonic()

    def _stop_only_reject_reason(self) -> str | None:
        """停止中だけの条件を満たさなければ拒否理由の文言、満たせば None。"""
        received = self._state_at is not None
        age = (time.monotonic() - self._state_at) if received else None
        stale_ms = self.get_parameter("state_stale_ms").value
        allowed, reason = stop_only_allows(
            self._state_mode, self._state_name, self._jog_active,
            received, age, stale_ms / 1000.0)
        if allowed:
            return None
        detail = {
            "state_not_received": "/system/state を受信していないため",
            "state_stale": "/system/state が古いため",
            "jog_active": "ジョグ中のため",
        }.get(
            reason,
            f"いまの状態（{self._state_mode}/{self._state_name}）では")
        return f"停止中（IDLE・PREP の地図作業中）のみ操作できます（{detail}拒否）"

    # ------------------------------------------------------------
    # 実効値の解決（registry → calib → overrides の順に重ね書き。§5.3「読み込み順」）
    # ------------------------------------------------------------
    def _registry_path(self) -> str:
        override = self.get_parameter("registry_path").value
        if override:
            return override
        return os.path.join(get_package_share_directory("th_params"), "config", "registry.yaml")

    def _overrides_path(self) -> str:
        override = self.get_parameter("overrides_path").value
        return override or OVERRIDES_PATH

    def _calib_path(self) -> str:
        override = self.get_parameter("calib_path").value
        return override or CALIB_PATH

    def _generated_dir(self) -> str:
        override = self.get_parameter("generated_dir").value
        return override or GENERATED_DIR

    def _load_provenance(self) -> dict:
        """起動時の設定生成が書いた出どころ記録を読む（SG-B8）。

        無ければ（初回起動前・テスト等）空を返す。その場合は「前回起動時に
        適用された上書き」は不明なので、/params/get は空扱いにする。"""
        try:
            with open(os.path.join(self._generated_dir(), PROVENANCE_FILENAME),
                      encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except (FileNotFoundError, OSError):
            return {}

    def _load_registry_rows(self) -> list[dict]:
        with open(self._registry_path(), encoding="utf-8") as f:
            rows = yaml.safe_load(f)
        return rows

    def _apply_calib(self, rows: list[dict]) -> None:
        """§5「校正値の取り込み」: calib/current.yaml の該当行を measured へ上書きする。"""
        try:
            with open(self._calib_path(), encoding="utf-8") as f:
                calib = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return
        rows_by_name = {r["name"]: r for r in rows}
        for name, entry in (calib or {}).items():
            row = rows_by_name.get(name)
            if row is None:
                continue
            is_dict = isinstance(entry, dict)
            row["value"] = entry.get("value") if is_dict else entry
            row["status"] = "measured"
            row["measured_at"] = entry.get("measured_at", "") if is_dict else ""
            row["source"] = entry.get("source", "calib/current.yaml") if is_dict else "calib/current.yaml"

    def _load_overrides(self) -> dict:
        try:
            with open(self._overrides_path(), encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}

    def _apply_overrides(self, rows: list[dict], overrides: dict) -> None:
        """§5.3 PT-2: given の行だけが対象。他は無視する（/params/set 側でも拒否済みのはずだが
        overrides.yaml を手で編集された場合の保険として二重に守る）。"""
        rows_by_name = {r["name"]: r for r in rows}
        for name, entry in (overrides or {}).items():
            row = rows_by_name.get(name)
            if row is None or row.get("status") != "given":
                continue
            row["value"] = entry.get("value") if isinstance(entry, dict) else entry

    def _effective_rows(self) -> list[dict]:
        rows = self._load_registry_rows()
        self._apply_calib(rows)
        self._apply_overrides(rows, self._load_overrides())
        return rows

    def _resolved(self, clamp_warnings: list | None = None,
                  clamp_errors: list | None = None) -> dict:
        return export.resolve_registry(self._effective_rows(),
                                        clamp_warnings=clamp_warnings, clamp_errors=clamp_errors)

    # ------------------------------------------------------------
    # /system/params_status
    # ------------------------------------------------------------
    #
    # N-7: A1 のクランプ（`export._apply_v_max_clamp`）が実際に何をしたかは
    # `ParamsStatus.msg` に運ぶフィールドが無い（`placeholder_count` / `placeholder_names` /
    # `digest` のみ。本ファイル冒頭のモジュール docstring 参照）ため、ログに出す
    # （R5: 沈黙禁止）。`/system/params_status` トピック自体への反映は本パケットの範囲外——
    # メッセージ定義の変更は th_system_msgs の再ビルドを要する別範囲の変更になる。
    def _publish_status(self):
        clamp_warnings: list[str] = []
        clamp_errors: list[str] = []
        resolved = self._resolved(clamp_warnings=clamp_warnings, clamp_errors=clamp_errors)
        for w in clamp_warnings:
            self.get_logger().warn(w)
        for e in clamp_errors:
            self.get_logger().error(e)
        placeholder_names = sorted(
            name for name, (status, _value) in resolved.items() if status == "placeholder")
        msg = ParamsStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.placeholder_count = len(placeholder_names)
        msg.placeholder_names = placeholder_names
        msg.digest = export.compute_digest(resolved)
        self._pub_status.publish(msg)

    # ------------------------------------------------------------
    # /params/get
    # ------------------------------------------------------------
    #
    # D1-b: サービスコールバック内の例外を外へ漏らさない。rclpy の executor まで
    # 伝播すると `params_audit` プロセスごと死ぬ（実際に KeyError で発生した。
    # `colcon test` の `test_params_audit_node` が検出）。3コールバックとも
    # 「どんな例外が起きても応答を返してノードは生き続ける」ようにし、例外内容は
    # 必ず `get_logger().error()` に残す（R5: 沈黙禁止）。起動時
    # （`__init__` → `_publish_status()`）はここに含めない ——
    # registry が読めないなら launch 時点で落ちるのが正しい。
    def _cb_get(self, request, response):
        try:
            return self._cb_get_impl(request, response)
        except Exception as e:  # noqa: BLE001 — ノードを生かすための最終防壁
            self.get_logger().error(f"/params/get で例外が発生した: {e!r}")
            response.json = json.dumps({"error": str(e)}, ensure_ascii=False)
            return response

    def _cb_get_impl(self, request, response):
        # SG-B8（Spec-params.md §6）: 「いま効いている値（生成物＝前回起動時に
        # 適用されたもの）」と「保存済みで次回から効く上書き値」を区別して返す。
        # 走行中の値は変えない（次の起動から効く）ため、/params/set の受理直後は
        # 両者がずれる。画面には「再起動後に反映」と出す。
        # 計算本体は export.split_effective_pending（純粋関数。host 試験で縛る）。
        rows = self._load_registry_rows()
        self._apply_calib(rows)
        base_resolved = export.resolve_registry(copy.deepcopy(rows))

        overrides = self._load_overrides()
        provenance = self._load_provenance()
        effective, pending, origins = export.split_effective_pending(
            rows, overrides, provenance)
        effective_pub = export.public_values(effective)

        names = list(request.names) if request.names else sorted(base_resolved.keys())

        def _pick(mapping: dict) -> dict:
            return {name: mapping[name] for name in names if name in mapping}

        # 後方互換: names を指定した呼び出しには従来どおりフラットな実効値も残す
        # （`effective` が正本。フラット部は将来削る）。
        flat = _pick(export.public_values(self._resolved()))
        response.json = json.dumps({
            **flat,
            "effective": _pick(effective_pub),
            "pending": _pick(pending),
            "origins": {name: origins[name] for name in names if name in origins},
            "restart_required": any(name in pending for name in names),
        }, ensure_ascii=False)
        return response

    # ------------------------------------------------------------
    # /params/set — DetailedDesign-params.md §5.3.1。
    #
    # request.json の形は SetParams.srv 自体には規定が無い（`string json` のみ）ため、
    # PT-4（set_at/set_by/reason 必須）を満たせる形として本パケットで以下に決めた
    # （完了報告に明記。names.md / DetailedDesign-params.md のどちらにも規定が無い）:
    #     {"values": {"<name>": <value>, ...}, "set_by": "<誰が>", "reason": "<なぜ>"}
    # set_by / reason を欠いた要求は拒否する（PT-4: 出どころの無い数値を作らない）。
    # ------------------------------------------------------------
    def _cb_set(self, request, response):
        try:
            return self._cb_set_impl(request, response)
        except Exception as e:  # noqa: BLE001 — 安全側 = 値を当てない。ノードは生かす
            self.get_logger().error(f"/params/set で例外が発生した: {e!r}")
            response.success = False
            response.message = f"内部エラー: {e}"
            return response

    def _cb_set_impl(self, request, response):
        rejected = self._stop_only_reject_reason()
        if rejected is not None:
            response.success = False
            response.message = rejected.replace("操作できます", "変更できます")
            return response

        try:
            payload = json.loads(request.json)
        except (TypeError, ValueError) as e:
            response.success = False
            response.message = f"json のパースに失敗した: {e}"
            return response

        if not isinstance(payload, dict):
            response.success = False
            response.message = "json は object でなければならない"
            return response

        values = payload.get("values")
        set_by = payload.get("set_by")
        reason = payload.get("reason")
        if not isinstance(values, dict) or not values:
            response.success = False
            response.message = "values が空、または object でない"
            return response
        if not set_by or not reason:
            response.success = False
            response.message = "PT-4 違反: set_by と reason は必須（出どころの無い数値を作らない）"
            return response

        rows = self._load_registry_rows()
        self._apply_overrides(rows, self._load_overrides())  # 既存の重ね書きの上に足す
        rows_by_name = {r["name"]: r for r in rows}

        # PT-2: status: given の行だけが対象（derived / measured は拒否）
        for name in values:
            row = rows_by_name.get(name)
            if row is None:
                response.success = False
                response.message = f"未知のパラメータ: {name}"
                return response
            if row.get("status") != "given":
                response.success = False
                response.message = (
                    f"{name} は status:{row.get('status')} のため変更できません"
                    "（given の行のみ調整可能。derived は元の値を変えること）")
                return response

        # PT-1: 当てる前にコピーの上で全アサーションを検査する（★部分適用しない）
        patched_rows = copy.deepcopy(rows)
        for row in patched_rows:
            if row["name"] in values:
                row["value"] = values[row["name"]]

        schema_errors = schema.validate_registry(patched_rows)
        if schema_errors:
            response.success = False
            response.message = "; ".join(schema_errors)
            return response

        clamp_warnings: list[str] = []
        clamp_errors: list[str] = []
        resolved = export.resolve_registry(patched_rows,
                                            clamp_warnings=clamp_warnings, clamp_errors=clamp_errors)
        # params_audit は「現在の起動段階」を知らない。安全側に倒し、常に最大段階・
        # 全ノードを対象に検査する（DD-6 の「stage 既定は最大値」と同じ思想）。
        # D2: include_a8=False。A8（blocking placeholder の完全性の門）は「これから
        # 起動できるか」を守るものであり launch の責務。/params/set はまだ起動して
        # いない値を当てようとしているだけなので、registry 全体に残る無関係な
        # 未測定値（例: link_gap_p99_ms）で毎回拒否されてはいけない
        # （実際、現行 registry.yaml では A8 を回すと常に失敗し現場調整が成立しない）。
        # ここで見るのは A1〜A7・A10・A11 ＝ 当てようとしている値そのものの物理的整合性。
        assertion_errors, assertion_warnings = export.run_assertions(
            patched_rows, resolved, stage=8, nodes=None, include_a8=False,
            clamp_warnings=clamp_warnings, clamp_errors=clamp_errors)
        if assertion_errors:
            response.success = False
            response.message = "; ".join(assertion_errors)
            return response

        self._write_overrides(values, set_by=set_by, reason=reason)
        self._publish_status()

        # 警告（A6 等。params.md §4）は拒否理由にしないが、握り潰さない——ログと
        # response.message の両方に出す（R5: 沈黙禁止）。
        if assertion_warnings:
            self.get_logger().warn("/params/set 受理（警告あり）: " + "; ".join(assertion_warnings))

        response.success = True
        message = (
            "受理した。overrides.yaml に反映した。稼働中のノードへのライブ反映は行っていない"
            "（このパケットの実装範囲外）。次回起動で反映されるまで再起動が必要")
        if assertion_warnings:
            message += " / 警告: " + "; ".join(assertion_warnings)
        response.message = message
        return response

    def _write_overrides(self, values: dict, set_by: str, reason: str):
        overrides_path = self._overrides_path()
        os.makedirs(os.path.dirname(overrides_path), exist_ok=True)
        current = self._load_overrides()
        now = datetime.now(timezone.utc).isoformat()
        for name, value in values.items():
            current[name] = {
                "value": value,
                "set_at": now,
                "set_by": set_by,
                "reason": reason,
            }
        with open(overrides_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(current, f, allow_unicode=True, sort_keys=True)

    # ------------------------------------------------------------
    # /params/save — overrides.yaml への永続化は /params/set の時点で既に完了している
    # （§5.3.1 手順5）。ここでは同じモードゲートを再検査するだけの確認応答にする
    # （config_manager の set/save 二段構え——ライブ反映のみの set と YAML 書き戻しの
    # save——とは異なり、params_audit の /params/set は最初から永続化まで行う設計に
    # した。判断は完了報告に明記）。
    # ------------------------------------------------------------
    def _cb_save(self, request, response):
        try:
            return self._cb_save_impl(request, response)
        except Exception as e:  # noqa: BLE001 — 安全側 = 保存されたと嘘をつかない
            self.get_logger().error(f"/params/save で例外が発生した: {e!r}")
            response.success = False
            response.message = f"内部エラー: {e}"
            return response

    def _cb_save_impl(self, request, response):
        rejected = self._stop_only_reject_reason()
        if rejected is not None:
            response.success = False
            response.message = rejected.replace("操作できます", "保存できます")
            return response
        response.success = True
        response.message = "変更は /params/set の時点で overrides.yaml に保存済み"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ParamsAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
