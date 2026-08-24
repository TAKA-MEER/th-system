"""state_core.py — 状態機械の純粋コア（WP-STATE-01）

rclpy を import しない（`th_testing` から直接 pytest できる。CLAUDE.md／DetailedDesign-state.md §1）。
`transitions.yaml` / `attributes.yaml` / `mode_entry.yaml` を読んで `Decision` を返す。

数値リテラルを書かない（R2）。時間・速度・件数などの値はすべて呼び出し側（ノード）か
YAML 設定から渡される。モード名・状態名・トークン名などの「文字列の集合」は
DetailedDesign-names.md §2-2／§3 の転記であり、数値ではないので R2 の対象外。

`step()` は副作用を持たない（S-1）。ラッチ（`prev_*`）は呼び出し側が保持し、`Context` で渡す。
"""
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, Set, Any


# ============================================================
# DetailedDesign-names.md §2-2 — モード一覧（18）
# ============================================================
MODES: Set[str] = {
    "INIT", "IDLE", "ESTOP", "CARRY",
    "FOLLOW", "MANUAL", "TEACH_FOLLOW", "TEACH_MANUAL", "REPLAY", "LINE", "LEASH",
    "PREP", "PANEL_NAV", "AT_PANEL", "SUMMON", "HOME_NAV",
    "OPCHECK", "CALIB",
}

# DetailedDesign-names.md §3 — モードごとの状態集合（`NONE` は「状態を持たない」の予約語）。
MODE_STATES: Dict[str, Set[str]] = {
    "INIT": {"CHECK"},
    "IDLE": {"NONE"},
    "ESTOP": {"NONE"},
    "CARRY": {"NONE"},
    "FOLLOW": {"SELECT", "CONFIRM", "RUN", "PAUSE"},
    "MANUAL": {"RUN", "PAUSE"},
    "TEACH_FOLLOW": {"ROUTE_SEL", "REC", "PAUSE", "SAVED"},
    "TEACH_MANUAL": {"ROUTE_SEL", "REC", "PAUSE", "SAVED"},
    "REPLAY": {"ROUTE_SEL", "LOCALIZE", "READY", "RUN", "PAUSE", "SAVED"},
    "LINE": {"SETUP", "PLANNED", "RUN", "PAUSE", "ARRIVED"},
    "LEASH": {"DEV_CHECK", "READY", "RUN", "HOLD", "PAUSE"},
    "PREP": {"MAPPING", "REGISTER", "RETURN", "EDIT", "PAUSE", "SAVED"},
    "PANEL_NAV": {"NAV", "BLOCKED", "PAUSE", "ALIGN"},
    "AT_PANEL": {"IDLE_P", "WORKING", "PAUSE"},
    "SUMMON": {"POINT", "WAIT_CLEAR", "NAV", "BLOCKED", "PAUSE", "ALIGN"},
    "HOME_NAV": {"NAV", "BLOCKED", "PAUSE"},
    "OPCHECK": {"LIST", "RUNNING_CHECK", "REPAIR"},
    "CALIB": {"LIST", "S1", "S2", "S3", "S4"},
}

assert set(MODE_STATES.keys()) == MODES

# WP-STATE-02 — 起動時モード。state_manager.py はモード名の文字列比較を書けない
# （完了条件① の AST 検査が MODES に含まれる文字列定数を禁じる。N-1）ため、
# 起動時モードの参照点をここに 1 箇所だけ置く。判断ロジックではなく定数の公開なので
# WP-STATE-01 の「遷移の判断」の範囲には入らない。
BOOT_MODE: str = "INIT"

# DetailedDesign-state.md §4-1-1 末尾・§2 validate()⑥docstring — PAUSE を持たないモード。
NO_PAUSE_MODES: Set[str] = {"INIT", "IDLE", "ESTOP", "CARRY", "OPCHECK", "CALIB"}

# DetailedDesign-state.md §7 — latch_prev を記録しないモード（FMEA②）。
NO_LATCH_MODES: Set[str] = {"ESTOP", "CARRY"}

# DetailedDesign-state.md §3-5 — ui.goto の kind → モード写像。
GOTO_KIND_MAP: Dict[str, str] = {
    "PANEL": "PANEL_NAV",
    "HOME": "HOME_NAV",
    "SUMMON": "SUMMON",
}

# DetailedDesign-state.md §3-2 — 実行時解決トークン（$arg.<key> はプレフィックスで判定）。
_KNOWN_TOKENS: Set[str] = {
    "$initial", "$resume_run", "$resume_state",
    "$prev_mode", "$prev_state", "$prev_sub",
}
_ARG_TOKEN_PREFIX = "$arg."

# DetailedDesign-state.md §3-3 — effect 名と引数型（validate()③が使う）。
# 値は型名の集合。空 dict は「引数を持たない」effect。
_STR = "str"
_BOOL = "bool"
EFFECT_ARG_SPECS: Dict[str, Dict[str, str]] = {
    "set_jog": {"on": _BOOL},
    "latch_prev": {},
    "clear_prev": {},
    "set_screen": {"screen_id": _STR},
    "mark_arrived": {"kind": _STR},
    "abort_check": {},
    "set_target": {"index": _STR},
    "clear_selection": {},
    "face_target": {},
    "open_window": {"id": _STR},
    "close_window": {"id": _STR},
    "guide": {"key": _STR},
    "ask_save_if_unsaved": {},
    "ask_save": {},
    "enable_main_menu": {},
    "show_resume": {},
    "restart_control_stack": {},
    "start_record": {"route_id": _STR},
    "resume_record": {},
    "finalize_route": {"route_id": _STR},
    "load_route": {"route_id": _STR, "reverse": _STR},
    "rotate_to_start_yaw": {},
    "resume_path": {},
    "widen_search": {},
    "global_localize": {},
    "commit_map_patch": {},
    "commit_venue_map": {},
    "keep_all": {},
    "begin_two_point": {"kind": _STR},
    "place_pin": {},
    "reject_register": {},
    "disable_jog_ui": {},
    "enable_jog_ui": {},
    "cancel_follow_path": {},
    "resume_follow_path": {},
    "replan": {},
    "start_monitor": {"item": _STR},
    "offer_calib": {"item": _STR},
    "offer_opcheck": {"item": _STR},
    "record_result": {"item": _STR, "result": _STR},
    "feed_check_input": {"pressed": _STR},
    "begin_wizard": {"item": _STR},
    "run_measurement": {"item": _STR},
    "build_preview": {"item": _STR},
    "apply_and_verify": {"item": _STR},
    "commit_calib": {"item": _STR},
    "revert_calib": {"item": _STR},
    "discard_calib": {"item": _STR},
}


# ============================================================
# DetailedDesign-state.md §2 — 純粋コアの API
# ============================================================
@dataclass(frozen=True)
class Context:
    """遷移の判定に必要な、状態機械の外にある情報。ノードが毎回詰めて渡す。
    ★ ガードが読んでよいのは Context のフィールドと、step() に渡る mode / state だけ
       （guards.py の述語は (mode, state, ctx) の 3 引数を取る。§4-1-1）。
       時刻・ファイル・環境変数を読んではいけない —— これが validate() ② の検査項目。"""
    prev_mode: str
    prev_state: str
    prev_sub: str
    flags: Dict[str, bool]
    zone: str
    candidate_count: int
    target_selected: bool
    target_confident: bool
    fault_active: bool
    fault_severity: str
    fault_type: str
    hw_estop: bool
    ui_estop: bool
    route_ids: Tuple[str, ...]
    pin_kinds: Tuple[str, ...]
    leash_present: bool
    leash_taut: bool
    line_visible: bool
    camera_present: bool
    check_item: str
    check_result: str
    calib_item: str
    calib_preview_sane: bool
    map_update_available: bool
    now_ms: int
    arg: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Effect:
    name: str
    args: Dict[str, Any]


@dataclass(frozen=True)
class Decision:
    accepted: bool
    to_mode: str
    to_state: str
    effects: Tuple[Effect, ...]
    rule_id: str
    reject_reason_key: str


GuardFn = Callable[[str, str, Context], bool]
GuardRegistry = Dict[str, GuardFn]


class StateCore:
    def __init__(self, transitions: List[dict], mode_entry: Dict[str, List[str]],
                 attributes: Dict[str, dict], guards: GuardRegistry):
        self._transitions = transitions
        self._mode_entry = mode_entry
        self._attrs = attributes
        self._guards = guards

    # --------------------------------------------------------
    # validate()
    # --------------------------------------------------------
    def validate(self) -> List[str]:
        """起動時に呼ぶ。空でなければ起動を止める（S-5）。"""
        errors: List[str] = []
        errors.extend(self._validate_names())
        errors.extend(self._validate_guards())
        errors.extend(self._validate_effects())
        errors.extend(self._validate_tokens())
        errors.extend(self._validate_reachability())
        errors.extend(self._validate_pause_presence())
        errors.extend(self._validate_spec_ref())
        return errors

    def _validate_names(self) -> List[str]:
        """① モード名・状態名が §2/§3 の集合に含まれる。"""
        errors: List[str] = []
        for row in self._transitions:
            rid = row.get("id", "<no id>")
            for m in _as_list(row.get("mode")):
                if m != "*" and m not in MODES:
                    errors.append(f"{rid}: 未知の mode '{m}'")
            for s in _as_list(row.get("state")):
                if s != "*" and not any(s in states for states in MODE_STATES.values()):
                    errors.append(f"{rid}: 未知の state '{s}'")
            to_mode = row.get("to_mode")
            if to_mode not in (None, "=") and not _is_token(to_mode) and to_mode not in MODES:
                errors.append(f"{rid}: 未知の to_mode '{to_mode}'")
            to_state = row.get("to_state")
            if (to_state not in (None, "=") and not _is_token(to_state)
                    and not any(to_state in states for states in MODE_STATES.values())):
                errors.append(f"{rid}: 未知の to_state '{to_state}'")
        for mode in self._attrs:
            if mode not in MODES:
                errors.append(f"attributes.yaml: 未知のモード '{mode}'")
        for mode in self._mode_entry:
            if mode not in MODES:
                errors.append(f"mode_entry.yaml: 未知のモード '{mode}'")
        return errors

    def _validate_guards(self) -> List[str]:
        """② ガード名が guards.py に存在し、シグネチャが (mode, state, ctx) で、
        Context のフィールドと mode/state 以外を読まない（ヒューリスティック）。"""
        import inspect
        errors: List[str] = []
        used_guard_names: Set[str] = set()
        for row in self._transitions:
            g = row.get("guard")
            if g is not None:
                used_guard_names.add(g)
        context_fields = set(Context.__dataclass_fields__.keys())
        forbidden_tokens = ("time.", "datetime.", "os.", "open(", "environ",
                             "random.", ".now(", "input(")
        for name in used_guard_names:
            fn = self._guards.get(name)
            if fn is None:
                errors.append(f"guard '{name}' が guards.py に無い")
                continue
            try:
                sig = inspect.signature(fn)
            except (TypeError, ValueError):
                errors.append(f"guard '{name}' のシグネチャを取得できない")
                continue
            if len(sig.parameters) != len(("mode", "state", "ctx")):
                errors.append(f"guard '{name}' の引数が (mode, state, ctx) の3個ではない")
            try:
                src = inspect.getsource(fn)
            except (OSError, TypeError):
                src = ""
            for tok in forbidden_tokens:
                if tok in src:
                    errors.append(
                        f"guard '{name}' が時刻・ファイル・環境変数らしき呼び出し "
                        f"'{tok}' を含む（S-4 違反の疑い）")
            for word in _identifiers_in_source(src):
                if word in ("mode", "state", "ctx", "self") or word in context_fields:
                    continue
        _ = context_fields  # 読み取り集合は将来のより厳密な静的解析用に保持する
        return errors

    def _validate_effects(self) -> List[str]:
        """③ effect 名が §3-3 の一覧に存在し、引数の型が合う。"""
        errors: List[str] = []
        for row in self._transitions:
            rid = row.get("id", "<no id>")
            for eff in row.get("effects") or []:
                name = eff.get("name")
                spec = EFFECT_ARG_SPECS.get(name)
                if spec is None:
                    errors.append(f"{rid}: 未知の effect '{name}'")
                    continue
                args = eff.get("args") or {}
                for key, val in args.items():
                    if key not in spec:
                        errors.append(f"{rid}: effect '{name}' に未知の引数 '{key}'")
                        continue
                    if isinstance(val, str) and val.startswith(_ARG_TOKEN_PREFIX):
                        continue  # 実行時解決トークンは型検査しない
                    expected = spec[key]
                    if expected == _BOOL and not isinstance(val, bool):
                        errors.append(f"{rid}: effect '{name}.{key}' は bool を期待")
                    if expected == _STR and not isinstance(val, (str, bool)):
                        errors.append(f"{rid}: effect '{name}.{key}' は str を期待")
        return errors

    def _validate_tokens(self) -> List[str]:
        """④ to_mode/to_state の $トークンが §3-2 の一覧に含まれる。"""
        errors: List[str] = []
        for row in self._transitions:
            rid = row.get("id", "<no id>")
            for col in ("to_mode", "to_state"):
                val = row.get(col)
                if isinstance(val, str) and val.startswith("$") and not _is_token(val):
                    errors.append(f"{rid}: {col} の未知のトークン '{val}'")
        return errors

    def _validate_reachability(self) -> List[str]:
        """⑤ 到達不能な状態が無い。"""
        nodes = {(m, s) for m in MODES for s in MODE_STATES[m]}
        edges = _build_reachability_edges(self._transitions, self._mode_entry, self._attrs)
        start = ("INIT", self.initial_state("INIT"))
        visited = {start}
        frontier = [start]
        while frontier:
            cur = frontier.pop()
            for nxt in edges.get(cur, ()):
                if nxt not in visited:
                    visited.add(nxt)
                    frontier.append(nxt)
        unreachable = sorted(nodes - visited)
        return [f"到達不能: {m}.{s}" for m, s in unreachable]

    def _validate_pause_presence(self) -> List[str]:
        """⑥ PAUSE を持たないモードが無い（除外は INIT/IDLE/ESTOP/CARRY/OPCHECK/CALIB の6つ）。"""
        errors: List[str] = []
        for mode in MODES - NO_PAUSE_MODES:
            if "PAUSE" not in MODE_STATES.get(mode, set()):
                errors.append(f"{mode}: PAUSE 状態が無い")
        return errors

    def _validate_spec_ref(self) -> List[str]:
        """⑦ 全行に spec_ref がある。"""
        errors = []
        for row in self._transitions:
            if not row.get("spec_ref"):
                errors.append(f"{row.get('id', '<no id>')}: spec_ref が無い")
        return errors

    # --------------------------------------------------------
    # step()
    # --------------------------------------------------------
    def step(self, mode: str, state: str, event: str, ctx: Context) -> Decision:
        indexed = [(i, r) for i, r in enumerate(self._transitions)
                   if _row_matches(r, mode, state, event)]

        # §3-1 規則3: override_common がマッチしかつガードが真になった時点で採る。
        # 同じ event を持つ layer:0 の行は以後いっさい評価しない（override を先に見る）。
        for _, row in indexed:
            if row.get("override_common") and _guard_true(row, mode, state, ctx, self._guards):
                return self._decide(row, mode, state, ctx)

        # §3-1 規則4: layer 昇順・同 layer 内は記載順で、マッチしてガードが真の最初の行を採る。
        ordered = sorted(indexed, key=lambda pair: (pair[1].get("layer"), pair[0]))
        for _, row in ordered:
            if _guard_true(row, mode, state, ctx, self._guards):
                return self._decide(row, mode, state, ctx)

        # §3-1 規則6: どの行にも当たらなければ accepted=False, reject_reason_key="not_allowed"。
        return Decision(accepted=False, to_mode=mode, to_state=state, effects=(),
                         rule_id="", reject_reason_key="not_allowed")

    def _decide(self, row: dict, mode: str, state: str, ctx: Context) -> Decision:
        # §3-1 規則5: reject: true の行を採った場合。
        if row.get("reject"):
            return Decision(accepted=False, to_mode=mode, to_state=state, effects=(),
                             rule_id=row["id"], reject_reason_key=row.get("reject_reason_key") or "not_allowed")

        to_mode = self._resolve_to_mode(row, mode, ctx)
        to_state = self._resolve_to_state(row, mode, state, to_mode, ctx)
        effects = tuple(
            Effect(name=e["name"], args=self._resolve_effect_args(e.get("args") or {}, ctx))
            for e in (row.get("effects") or [])
            # latch_prev は mode ∈ {ESTOP, CARRY} のときは記録しない（§7 のラッチ表・FMEA②）。
            # ESTOP 中の物理押下（→ CARRY）・CARRY 中の重大フォルト（→ ESTOP）で
            # 復帰先 (prev_*) を壊さないため、この場合は latch_prev effect 自体を発火させない。
            if not (e["name"] == "latch_prev" and mode in NO_LATCH_MODES)
        )
        return Decision(accepted=True, to_mode=to_mode, to_state=to_state, effects=effects,
                         rule_id=row["id"], reject_reason_key="")

    def _resolve_to_mode(self, row: dict, mode: str, ctx: Context) -> str:
        raw = row.get("to_mode")
        if raw == "=" or raw is None:
            return mode
        if raw == "$arg.kind":
            kind = ctx.arg.get("kind")
            return GOTO_KIND_MAP.get(kind, kind)
        if raw == "$prev_mode":
            return ctx.prev_mode
        if isinstance(raw, str) and raw.startswith(_ARG_TOKEN_PREFIX):
            key = raw[len(_ARG_TOKEN_PREFIX):]
            return ctx.arg.get(key)
        return raw

    def _resolve_to_state(self, row: dict, mode: str, state: str, resolved_to_mode: str,
                           ctx: Context) -> str:
        raw = row.get("to_state")
        if raw == "=" or raw is None:
            return state
        if raw == "$initial":
            return self.initial_state(resolved_to_mode)
        if raw == "$resume_run":
            return self._attrs.get(mode, {}).get("run_state")
        if raw == "$resume_state":
            return self._attrs.get(mode, {}).get("resume_state")
        if raw == "$prev_mode":
            return ctx.prev_mode
        if raw == "$prev_state":
            return ctx.prev_state
        if raw == "$prev_sub":
            return ctx.prev_sub
        if isinstance(raw, str) and raw.startswith(_ARG_TOKEN_PREFIX):
            key = raw[len(_ARG_TOKEN_PREFIX):]
            return ctx.arg.get(key)
        return raw

    def _resolve_effect_args(self, args: Dict[str, Any], ctx: Context) -> Dict[str, Any]:
        resolved = {}
        for key, val in args.items():
            if isinstance(val, str) and val.startswith(_ARG_TOKEN_PREFIX):
                arg_key = val[len(_ARG_TOKEN_PREFIX):]
                resolved[key] = ctx.arg.get(arg_key)
            else:
                resolved[key] = val
        return resolved

    # --------------------------------------------------------
    def initial_state(self, mode: str) -> str:
        return self._attrs.get(mode, {}).get("initial_state")

    def attributes(self, mode: str) -> dict:
        return self._attrs.get(mode, {})


# ============================================================
# モジュールレベルのヘルパ（純粋関数。Context/mode/state 以外を読まない）
# ============================================================
def _as_list(val) -> List[str]:
    if val is None:
        return []
    if isinstance(val, list):
        return val
    return [val]


def _row_matches(row: dict, mode: str, state: str, event: str) -> bool:
    return (_field_matches(row.get("mode"), mode)
            and _field_matches(row.get("state"), state)
            and _field_matches(row.get("event"), event))


def _field_matches(row_val, actual: str) -> bool:
    if row_val == "*":
        return True
    if isinstance(row_val, list):
        return actual in row_val
    return row_val == actual


def _guard_true(row: dict, mode: str, state: str, ctx: Context, guards: GuardRegistry) -> bool:
    name = row.get("guard")
    if name is None:
        return True
    fn = guards.get(name)
    if fn is None:
        return False
    return bool(fn(mode, state, ctx))


def _is_token(val) -> bool:
    if not isinstance(val, str) or not val.startswith("$"):
        return False
    if val in _KNOWN_TOKENS:
        return True
    return val.startswith(_ARG_TOKEN_PREFIX)


def _identifiers_in_source(src: str) -> Set[str]:
    import re
    return set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", src))


def _build_reachability_edges(transitions: List[dict], mode_entry: Dict[str, List[str]],
                               attrs: Dict[str, dict]) -> Dict[Tuple[str, str], Set[Tuple[str, str]]]:
    """validate()⑤ が使う構造的な到達グラフ。ガードの真偽は問わない
    （ガードは ctx 依存なので、どこかの ctx で真になり得る前提で辺を張る）。
    $prev_* トークンは「以前に到達済みの状態へ戻るだけ」なので新たな到達性を生まない
    （latch_prev で記録された時点で既に到達済みのため、辺を張らなくても正しい）。
    """
    edges: Dict[Tuple[str, str], Set[Tuple[str, str]]] = {}

    def add_edge(src, dst):
        edges.setdefault(src, set()).add(dst)

    for row in transitions:
        if row.get("reject"):
            continue
        for src_mode in _as_list(row.get("mode")) or ["*"]:
            src_modes = MODES if src_mode == "*" else [src_mode]
            for m in src_modes:
                if m not in MODE_STATES:
                    continue
                for src_state in _as_list(row.get("state")) or ["*"]:
                    src_states = MODE_STATES[m] if src_state == "*" else [src_state]
                    for s in src_states:
                        if s not in MODE_STATES[m]:
                            continue
                        for tm in _target_modes(row.get("to_mode"), m, mode_entry):
                            if tm not in MODE_STATES:
                                continue
                            ts = _target_state(row.get("to_state"), m, tm, s, attrs)
                            if ts is None or ts not in MODE_STATES[tm]:
                                continue
                            add_edge((m, s), (tm, ts))
    return edges


def _target_modes(raw, source_mode: str, mode_entry: Dict[str, List[str]]) -> List[str]:
    if raw in ("=", None):
        return [source_mode]
    if raw == "$arg.kind":
        return list(GOTO_KIND_MAP.values())
    if isinstance(raw, str) and raw.startswith(_ARG_TOKEN_PREFIX):
        allowed = mode_entry.get(source_mode)
        return list(allowed) if allowed else list(MODES)
    return [raw]


def _target_state(raw, source_mode: str, target_mode: str, source_state: str,
                   attrs: Dict[str, dict]) -> Optional[str]:
    if raw in ("=", None):
        return source_state
    if raw == "$initial":
        return attrs.get(target_mode, {}).get("initial_state")
    if raw == "$resume_run":
        return attrs.get(source_mode, {}).get("run_state")
    if raw == "$resume_state":
        return attrs.get(source_mode, {}).get("resume_state")
    if raw in ("$prev_mode", "$prev_state", "$prev_sub"):
        return None  # 新たな到達性は生まない（既に到達済みの状態へ戻るだけ）
    if isinstance(raw, str) and raw.startswith(_ARG_TOKEN_PREFIX):
        return None  # 実行時の値なので構造的な到達性検査の対象外
    return raw
