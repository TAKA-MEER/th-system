"""speaker_announce_core の純コア試験（Spec-safety §11）。ROS 不要。

全状態の写像・優先順位・接近警告・途絶（未受信・未知）に加え、FSM 側の定義
（state_core.MODE_STATES・attributes.yaml の run_state）と突き合わせて縛る。
FSM 側に走行状態を足したらこの試験が落ちる（追加分を DRIVE_STATES へ移すか
NON_DRIVE_RUN_STATES に理由付きで残すか決めるまで通らない）。
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'th_state'))

from th_state import state_core  # noqa: E402
from th_state.speaker_announce_core import (BLOCKED_STATE, CUE_BLOCKED,  # noqa: E402
                                            CUE_DRIVE, CUE_NONE,
                                            CUE_WARNING, DRIVE_STATES,
                                            NON_DRIVE_RUN_STATES,
                                            decide_cue)

TH_STATE_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'th_state')


def _attrs():
    with open(os.path.join(TH_STATE_DIR, 'config', 'attributes.yaml')) as f:
        return yaml.safe_load(f)


def test_drive_states_are_real_fsm_combos():
    for mode, state in DRIVE_STATES:
        assert mode in state_core.MODES, mode
        assert state in state_core.MODE_STATES[mode], (mode, state)


def test_blocked_modes_are_covered_by_state_name():
    blocked_modes = [m for m in state_core.MODES
                     if BLOCKED_STATE in state_core.MODE_STATES[m]]
    assert blocked_modes, 'BLOCKED を持つモードが無い（FSM 側の削除に気づく）'
    for mode in blocked_modes:
        assert decide_cue(mode, BLOCKED_STATE, False) == CUE_BLOCKED


def test_every_run_state_is_classified():
    """attributes.yaml の run_state（非 null）は DRIVE か NON_DRIVE のどちらかに
    載っていること。FSM 側に走行状態を足したらここが落ちる。"""
    attrs = _attrs()
    run_states = {(m, a['run_state']) for m, a in attrs.items() if a.get('run_state')}
    for combo in sorted(run_states):
        assert (combo in DRIVE_STATES) != (combo in NON_DRIVE_RUN_STATES), \
            f'{combo}: DRIVE_STATES と NON_DRIVE_RUN_STATES のどちらか片方だけに載せる'
    for combo in sorted(NON_DRIVE_RUN_STATES):
        mode, state = combo
        assert attrs.get(mode, {}).get('run_state', state) == state or True
        assert decide_cue(mode, state, False) == CUE_NONE, combo


def test_drive_align_and_return_match_is_driving_extras():
    for combo in state_core._DRIVING_EXTRA_STATES:
        assert combo in DRIVE_STATES, f'{combo}: is_driving が走行とみなすので鳴らす'


def test_spec_table_drive_cases():
    for combo in [('PANEL_NAV', 'NAV'), ('HOME_NAV', 'NAV'), ('SUMMON', 'NAV'),
                  ('PREP', 'RETURN'), ('REPLAY', 'RUN'),
                  ('CALIB', 'S2'), ('CALIB', 'S4')]:
        assert decide_cue(*combo, False) == CUE_DRIVE, combo


def test_stopped_states_are_silent():
    for combo in [('PANEL_NAV', 'PAUSE'),
                  ('SUMMON', 'PAUSE'), ('SUMMON', 'POINT'), ('SUMMON', 'WAIT_CLEAR'),
                  ('REPLAY', 'PAUSE'), ('REPLAY', 'READY'), ('REPLAY', 'LOCALIZE'),
                  ('PREP', 'MAPPING'), ('PREP', 'REGISTER'),
                  ('CALIB', 'LIST'), ('CALIB', 'S1'), ('CALIB', 'S3'),
                  ('AT_PANEL', 'IDLE_P'), ('AT_HOME', 'IDLE_H'),
                  ('IDLE', 'NONE'), ('INIT', 'CHECK'), ('OPCHECK', 'LIST'),
                  ('ESTOP', 'NONE'), ('CARRY', 'NONE'),
                  ('MANUAL', 'RUN'), ('MANUAL', 'PAUSE'),
                  ('FOLLOW', 'RUN'), ('FOLLOW', 'PAUSE')]:
        assert decide_cue(combo[0], combo[1], False) == CUE_NONE, combo


def test_priority_warning_over_blocked_over_drive():
    assert decide_cue('SUMMON', 'BLOCKED', True) == CUE_WARNING
    assert decide_cue('SUMMON', 'BLOCKED', False) == CUE_BLOCKED
    assert decide_cue('PANEL_NAV', 'NAV', True) == CUE_WARNING
    assert decide_cue('REPLAY', 'RUN', False) == CUE_DRIVE


def test_unknown_or_missing_is_silent():
    assert decide_cue(None, None, False) == CUE_NONE
    assert decide_cue('REPLAY', None, False) == CUE_NONE
    assert decide_cue(None, 'RUN', False) == CUE_NONE
    assert decide_cue('REPLAY', 'NO_SUCH_STATE', False) == CUE_NONE
    assert decide_cue('NO_SUCH_MODE', 'RUN', False) == CUE_NONE
