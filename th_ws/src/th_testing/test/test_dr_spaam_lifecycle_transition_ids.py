"""test_dr_spaam_lifecycle_transition_ids.py — core直書きIDとlifecycle_msgs実値の照合

dr_spaam_lifecycle_controller_core.py は rclpy 非依存を保つため transition ID を
直書きしている。既存の core/node 試験は同じ定数を import して自分自身と比べる
だけなので、値がずれていても全部緑になる（2026-10-10 実機で発覚:
TRANSITION_ACTIVATE=2 は実は CLEANUP で、起動要求が dr_spaam_ros を
unconfigured へ落とした）。この試験だけが lifecycle_msgs の実値との一致を
直接縛る。変異チェック: core の 3 定数を旧値（CONFIGURE=0/ACTIVATE=2/
DEACTIVATE=3）に戻すと赤になること。

lifecycle_msgs の import が要るため ROS 環境専用（ホストの素の pytest では
動かない。CLAUDE.md の ROS 試験一覧に載せる）。
"""
from lifecycle_msgs.msg import State, Transition

from dr_spaam_lifecycle_controller_core import (
    STATE_ACTIVE,
    STATE_INACTIVE,
    STATE_UNCONFIGURED,
    TRANSITION_ACTIVATE,
    TRANSITION_CONFIGURE,
    TRANSITION_DEACTIVATE,
    TRANSITION_NONE,
)


class TestTransitionIdsMatchLifecycleMsgs:
    def test_configure(self):
        assert TRANSITION_CONFIGURE == Transition.TRANSITION_CONFIGURE == 1

    def test_activate(self):
        assert TRANSITION_ACTIVATE == Transition.TRANSITION_ACTIVATE == 3

    def test_deactivate(self):
        assert TRANSITION_DEACTIVATE == Transition.TRANSITION_DEACTIVATE == 4

    def test_none_does_not_collide_with_create(self):
        """TRANSITION_NONE(-1) は実 transition_id（CREATE=0 を含む）と衝突しない。"""
        assert TRANSITION_NONE == -1
        assert TRANSITION_NONE != Transition.TRANSITION_CREATE

    def test_states(self):
        assert STATE_UNCONFIGURED == State.PRIMARY_STATE_UNCONFIGURED == 1
        assert STATE_INACTIVE == State.PRIMARY_STATE_INACTIVE == 2
        assert STATE_ACTIVE == State.PRIMARY_STATE_ACTIVE == 3
