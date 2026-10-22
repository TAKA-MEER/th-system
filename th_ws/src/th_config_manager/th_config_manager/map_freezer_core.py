"""map_freezer_core.py — 地図更新 OFF の間 /map を凍結する判定と状態機械（ROS 非依存）。

Spec-onsite.md §4.0.2。slam_toolbox（map_and_localization）は localization に
していても直近のスキャンを一時的にグラフへ足して地図を配信し続ける。表示にも
経路計画にも「開いた地図そのもの」を使うため、slam_toolbox の出力（/map_live）と
購読者に見せる /map の間に map_freezer ノードを挟む。

このモジュールは「いつ通し、いつ写しを出し続けるか」だけを持つ（ノード側は
トピックの入出力だけ）。判定は純関数 `decide`、写しを取る契機は `MapFreezer`。

写しを取る契機は slam_control が `/map_session/freeze_anchor`（JSON 文字列）で
知らせる。slam_toolbox の出力は /map_live で届くだけで「どの地図か」を持たない
ため、読み直し・保存の前後関係を slam_control が番号つきで出す。

  event       意味（slam_control が出す時点）
  ----------  -------------------------------------------------------------
  opening     読み直しで deserialize_map を呼ぶ直前（respawn 済み・空のグラフ）
  opened      deserialize_map が成功した直後（読み込んだ地図で localization 中）
  saving      保存（serialize）を始める直前
  saved       保存と凍結が成功した直後
  save_failed 保存に失敗した
  unloaded    読み込んだ地図が無くなった（地図作成開始・リセット・破棄・再起動検知・
              読み直し失敗）

写しを取るルール:
  * opening → opened の間に届いた最後の /map_live が、開いた直後の地図の写し。
    opening より前に届いた地図は掴まない（開く前の PREP の地図を掴まない）。
    間に1枚も届かなければ opened のあとに届く最初の地図を写しにする。
  * saving の時点の最新の /map_live が、saved で写しになる（保存した地図）。
  * saved が来るまで写しは変えない（地図更新 ON の書き足しは保存するまで写しに入らない）。
"""

import json

# 通す（凍結しない）モード。PREP は地図を作っている最中、教示は記録中で、
# どちらも書き足しが正しい（Spec-onsite §4.0.2「対象外」）。
PASS_MODES = frozenset({'PREP', 'TEACH_FOLLOW', 'TEACH_MANUAL'})
# 直前のモードを覚えているモード（ESTOP / CARRY からの復帰先は prev_mode）。
LATCH_MODES = frozenset({'ESTOP', 'CARRY'})

ANCHOR_EVENTS = frozenset(
    {'opening', 'opened', 'saving', 'saved', 'save_failed', 'unloaded'})

# decide() の戻り値
FREEZE = 'FREEZE'
PASS = 'PASS'
PASS_NO_COPY = 'PASS_NO_COPY'   # 凍結が要る場面だが写しが無いので通す（警告を出す）


def effective_mode(mode: str, prev_mode: str) -> str:
    """ESTOP / CARRY の間は復帰先のモードで判定する（非常停止で凍結が切れない）。"""
    if mode in LATCH_MODES and prev_mode:
        return prev_mode
    return mode


def decide(loaded: bool, has_copy: bool, mode: str, prev_mode: str,
           map_update: bool) -> str:
    """/map に何を出すかを決める。

    凍結する条件は、地図を開いて localization にしていて（loaded）、地図更新が
    OFF で、PREP・教示の記録中でないとき。写しが取れていなければ通す（PASS_NO_COPY。
    何も出さないより生の地図の方が、地図が無くて走れなくなるよりは現状どおり）。
    """
    if not loaded or map_update:
        return PASS
    if effective_mode(mode, prev_mode) in PASS_MODES:
        return PASS
    return FREEZE if has_copy else PASS_NO_COPY


def make_anchor(boot: str, seq: int, event: str) -> str:
    return json.dumps({'boot': boot, 'seq': int(seq), 'event': event},
                      separators=(',', ':'))


def parse_anchor(text):
    """(boot, seq, event) か、読めなければ None。未知の event も None。"""
    try:
        d = json.loads(text)
        boot, seq, event = str(d['boot']), int(d['seq']), str(d['event'])
    except (ValueError, TypeError, KeyError):
        return None
    if event not in ANCHOR_EVENTS:
        return None
    return boot, seq, event


class MapFreezer:
    """写しの保持と、いま /map に出すべきものの選択。地図は不透明なオブジェクト。"""

    def __init__(self):
        self.loaded = False
        self.copy = None
        self.last_live = None
        self.live_counter = 0
        self.copy_counter = 0
        self.mode = ''
        self.prev_mode = ''
        self.map_update = False
        self._boot = None
        self._seq = 0
        self._collecting = False      # opening と opened の間
        self._staged = None
        self._want_capture = False    # opened のあと最初の地図を写しにする
        self._save_candidate = None
        self._last_token = None       # 最後に出したもの（変わったときだけ出す）

    # ── 入力 ──
    def on_state(self, mode: str, prev_mode: str, map_update: bool):
        self.mode, self.prev_mode, self.map_update = mode, prev_mode, bool(map_update)

    def on_live(self, msg):
        self.live_counter += 1
        self.last_live = msg
        if self._collecting:
            self._staged = msg
        if self._want_capture:
            self._set_copy(msg)
            self._want_capture = False

    def on_anchor(self, boot: str, seq: int, event: str) -> bool:
        """適用したら True。同じ起動で古い番号（再配送）は捨てる。"""
        if boot == self._boot and seq <= self._seq:
            return False
        self._boot, self._seq = boot, seq
        if event == 'opening':
            self.loaded = False
            self.copy = None
            self.last_live = None   # 開く前の地図は掴まない・出さない
            self._collecting = True
            self._staged = None
            self._want_capture = False
            self._save_candidate = None
        elif event == 'opened':
            self._collecting = False
            self.loaded = True
            if self._staged is not None:
                self._set_copy(self._staged)
            else:
                self._want_capture = True
            self._staged = None
        elif event == 'saving':
            self._save_candidate = self.last_live
        elif event == 'saved':
            self.loaded = True
            snap = self._save_candidate if self._save_candidate is not None \
                else self.last_live
            self._save_candidate = None
            if snap is not None:
                self._set_copy(snap)
                self._want_capture = False
            else:
                self.copy = None
                self._want_capture = True
        elif event == 'save_failed':
            self._save_candidate = None
        elif event == 'unloaded':
            self.loaded = False
            self.copy = None
            self._collecting = False
            self._staged = None
            self._want_capture = False
            self._save_candidate = None
        return True

    def _set_copy(self, msg):
        self.copy = msg
        self.copy_counter += 1

    # ── 出力 ──
    def decision(self) -> str:
        return decide(self.loaded, self.copy is not None, self.mode,
                      self.prev_mode, self.map_update)

    def poll(self):
        """いま新しく /map へ出すべき地図（変わっていなければ None）。"""
        d = self.decision()
        if d == FREEZE:
            token = ('copy', self.copy_counter)
            msg = self.copy
        else:
            token = ('live', self.live_counter)
            msg = self.last_live
        if msg is None or token == self._last_token:
            return None
        self._last_token = token
        return msg

    def current_frozen(self):
        """凍結中なら写し（定期の再配信用）。凍結中でなければ None。"""
        return self.copy if self.decision() == FREEZE else None
