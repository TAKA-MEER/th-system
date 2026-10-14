"""speaker_announce_core.py — 機体スピーカーの報知判定の純粋コア（Spec-safety.md §11）。

rclpy を import しない（`th_testing` から直接 pytest できる。follow_planner_core と
同じ二層）。`decide_cue()` が「いま鳴らすもの」を返す。音量・入切・試し鳴らし・
周期配信・途絶検出はノード側（speaker_announcer.py）が持つ。
"""

# 鳴らすものの集合（ラズパイ側と JSON の文字列で合意する）。
CUE_NONE = "none"
CUE_DRIVE = "drive"
CUE_BLOCKED = "blocked"
CUE_WARNING = "warning"
CUES = frozenset({CUE_NONE, CUE_DRIVE, CUE_BLOCKED, CUE_WARNING})

# Spec-safety.md §11 の表の転記。「自律で走っている間」に鳴る (mode, state)。
# 写し間違いを防ぐため、FSM 側の定義との対応をコメントに残す:
# PANEL_NAV/HOME_NAV/SUMMON の NAV は attributes.yaml の run_state、
# ALIGN は state_core.is_driving の _DRIVING_EXTRA_STATES、
# PREP/RETURN は同じく extra、REPLAY/RUN は run_state、
# CALIB の S2（実行・実測）/S4（検証）はウィザードの走行ステップ
# （S1 案内・S3 確認は走らない。LIST は一覧）。
DRIVE_STATES = frozenset({
    ("PANEL_NAV", "NAV"),
    ("PANEL_NAV", "ALIGN"),
    ("HOME_NAV", "NAV"),
    ("SUMMON", "NAV"),
    ("SUMMON", "ALIGN"),
    ("PREP", "RETURN"),
    ("REPLAY", "RUN"),
    ("CALIB", "S2"),
    ("CALIB", "S4"),
    # §2.1 の自律系の走行（2026-10-14 の spec 改定で追加）。
    ("FOLLOW", "RUN"),
    ("TEACH_FOLLOW", "REC"),
    ("LINE", "RUN"),
    ("LEASH", "RUN"),
})

# run_state（attributes.yaml）を持つが §11 の表に無く鳴らさないもの。
# 手動走行・教示（手動）は自律系ではないので鳴らさない。
# test_speaker_announce_core が attributes.yaml と突き合わせて縛る。
NON_DRIVE_RUN_STATES = frozenset({
    # 手動走行（自律ではない。接近警告だけ鳴る）。
    ("MANUAL", "RUN"),
    ("TEACH_MANUAL", "REC"),
    # 地図作成中の連れ回し（走行ではない。is_driving も PREP は RETURN だけ）。
    ("PREP", "MAPPING"),
})

# 塞がれ待ちの状態名。PANEL_NAV/SUMMON/HOME_NAV が持つ（state_core.MODE_STATES）。
# モードを列挙せず状態名で見るので、BLOCKED を持つモードが増えても拾われる。
BLOCKED_STATE = "BLOCKED"


def decide_cue(mode, state, approach_warning: bool) -> str:
    """「いま鳴らすもの」を返す。優先は warning > blocked > drive（§11）。

    mode/state が未受信（None）や未知の組合せのときは "none"
   （分からないものは鳴らさない。安全側＝沈黙）。
    """
    if approach_warning:
        return CUE_WARNING
    if state == BLOCKED_STATE:
        return CUE_BLOCKED
    if mode is not None and state is not None and (mode, state) in DRIVE_STATES:
        return CUE_DRIVE
    return CUE_NONE
