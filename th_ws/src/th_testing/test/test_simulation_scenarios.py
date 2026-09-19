"""
test_simulation_scenarios.py
==============================
設計書 10.3 節 — シミュレーションテスト（as-built 追従ロジックの回帰テスト。既定でスキップ）

【2026-08-25 調査で判明した事実 — 将来このファイルを復活させる前に必読】

1. このテストは Gazebo を起動しない。
   `generate_test_description()` が起動するのは mode_manager / safety_monitor /
   person_predictor.py / follow_planner.py の 4 ノードのみで、Gazebo も Nav2 も
   costmap も TF ブロードキャスタも存在しない。旧来の「Gazebo + Nav2 のフル環境が
   必要」という注記は誤りだった。

2. test_scenario_A1_retreat_on_approach が赤くなる真因はハーネスの未配線。
   follow_planner は `/local_costmap/costmap`（TRANSIENT_LOCAL）を受信して初めて
   base_link→costmap フレームの TF を引き、自己位置 (robot_pose_odom) を得る
   （follow_planner.py の `_update_costmap_tf()`）。本ハーネスは costmap を一切
   publish しないため robot_pose_odom は常に None になり、
   `follow_planner_core.FollowPlannerCore._handle_prepare/_handle_evading` は
   常に `kind="stop", reason="no_pose"` を返す。ログにも
   「自己位置(TF/costmap)未確立 — その場停止」が繰り返し出る。
   退避ロジック自体（純粋コアは test_follow_planner_logic.py で緑）は壊れておらず、
   自己位置不明時に速度指令を出さないのは意図した安全側の設計。実機では Nav2 の
   local_costmap が常時 costmap を配信するため、この経路は実際には動作する。

3. test_scenario_A2 / A3 は「見かけ緑」であり実質は何も検証していない。
   A2 は `len(retreat_history) > 0` だけを見るが、follow_planner は FOLLOWING 中
   毎周期 `_stop_retreat()`（ゼロの Twist）を publish するため、上記2の
   「常に no_pose で停止」状態でも無条件に真になる。A3 も「非ゼロ指令が来ない」
   ことを見るだけなので、同じ理由で退避ロジックが一切動いていなくても通る。
   つまり blocked（壁際停止）挙動も release/ヒステリシス挙動も、このテストでは
   実質未検証だった。

4. 近接退避・捜索旋回そのものが新設計では廃止されている。
   `docs/plan/detailed/DetailedDesign-names.md` §6.1（`/cmd_vel_retreat` 廃止の
   注記、`Spec-transit.md` §1.2「対象に近づいたとき: 停止する」と食い違うため）、
   および `docs/plan/detailed/DetailedDesign-reuse.md`（`follow_planner_core.py`
   の `FollowPlannerCore` 本体・`PREPARE`/`EVADING`・`find_nearest_open_direction`/
   `compute_evade_goal`、`scripts/follow_planner.py`、`scripts/person_predictor.py`
   をいずれも「廃止」と明記。新設計で近接時に効くのは後退ではなく停止で、担当は
   `obstacle_limiter`（WP-SAFE-03、未実装））。段階3 `WP-TRANSIT-01` で
   follow_planner / person_predictor ごとこのファイルも削除される見込み。

以上により、本ファイルは新規の costmap/TF 配線もアサーション強化も行わず、
既定 (TH_SKIP_SIM=1) でスキップする。`TH_SKIP_SIM=0` を明示すれば実行はできるが、
上記2・3の理由で「後退方向確認」「blocked/release 挙動」の検証としては無力な点に
注意すること。

  Scenario A: 試験員がロボットに近づいたとき後退するか
              (後退可能な場合・壁際で後退不可の場合)
  Scenario B: 試験員以外の人物モデルが接近したとき
              Nav2 標準回避で衝突を避けるか（識別なし・未実装）
  Scenario C: 試験員ロスト後に予測外挿・捜索旋回が起きるか
"""

import pytest
import rclpy
import time
import math
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from th_system_msgs.msg import RobotMode, PersonStatus
from th_system_msgs.srv import SetMode


# ── シミュレーション環境フラグ ────────────────────────────────
# 廃止予定の as-built 挙動（近接退避 PREPARE/EVADING・捜索旋回）の回帰テストのため
# 既定でスキップする。詳細は本ファイル冒頭 docstring と上記の設計書節を参照。
import os
SKIP_SIM = os.environ.get('TH_SKIP_SIM', '1') == '1'

_SKIP_REASON = (
    '廃止予定の as-built 挙動（近接退避 PREPARE/EVADING・捜索旋回）の回帰テスト。'
    '本ハーネスは /local_costmap/costmap と TF(odom→base_link) を供給しないため '
    'follow_planner は常に no_pose で停止を返す（ハーネス未配線。実機では Nav2 が '
    'costmap を出すため動作する）。加えて近接退避・捜索旋回は新設計で廃止 '
    '(DetailedDesign-names.md §6.1 の /cmd_vel_retreat 廃止注記、'
    'DetailedDesign-reuse.md の follow_planner.py / person_predictor.py 廃止表)。'
    '段階3 WP-TRANSIT-01 で本ファイルごと削除予定。'
    'TH_SKIP_SIM=0 で明示実行可能だが A2/A3 のアサーションは常時ゼロ指令でも'
    '通過するため実質未検証。')

sim_mark = pytest.mark.skipif(SKIP_SIM, reason=_SKIP_REASON)


@pytest.mark.launch_test
def generate_test_description():
    """
    テストノードが /person/status を直接発行する軽量版テスト環境。
    person_tracker_stub を使わないことでテスト間のメッセージ競合を防ぐ。
    Gazebo を使う場合は gazebo.launch.py を追加すること。
    """
    nodes = [
        # mode_manager
        launch_ros.actions.Node(
            package='th_mode_manager', executable='mode_manager',
            name='mode_manager', output='screen'),
        # safety_monitor (LiDAR/ESP32 は stub なのでタイムアウトを長く)
        launch_ros.actions.Node(
            package='th_safety', executable='safety_monitor',
            name='safety_monitor',
            parameters=[{
                'lidar_timeout_ms':  10000,
                'esp32_timeout_ms':  10000,
                'person_timeout_ms': 10000,
                'check_period_ms':   100,
            }],
            output='screen'),
        # person_predictor
        launch_ros.actions.Node(
            package='th_perception', executable='person_predictor.py',
            name='person_predictor',
            parameters=[{
                'max_predict_sec':  2.0,
                'search_timeout_sec': 5.0,
            }],
            output='screen'),
        # follow_planner
        # 距離帯を旧設計のシナリオ距離（0.3〜1.5m）に合わせて縮小している。
        # d_evade/d_prepare のデフォルト(2.0m/3.0m)のままだと本テストが使う
        # 位置(x=0.3〜1.5)が常に EVADING 圏内に入ってしまうため。
        launch_ros.actions.Node(
            package='th_planning', executable='follow_planner.py',
            name='follow_planner',
            parameters=[{
                'd_evade':                  0.8,
                'd_prepare':                 1.2,
                'distance_hysteresis_m':     0.1,
                'evade_scan_directions':    16,
                'evade_scan_max_dist':       2.0,
                'evade_route_length_m':      1.0,
                'retreat_check_clearance':   0.5,
                'retreat_speed':             0.15,
                'goal_deadzone_m':           0.0,
                'update_rate_hz':           10.0,
            }],
            output='screen'),
    ]

    return launch.LaunchDescription(
        nodes + [launch_testing.actions.ReadyToTest()]
    ), {}


@sim_mark
@unittest.skipIf(SKIP_SIM, _SKIP_REASON)
class TestSimulationScenarios(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_sim_scenarios')

        self._mode_history:    list[int]   = []
        self._retreat_history: list[Twist] = []
        self._pred_pos_history: list[PointStamped] = []
        self._search_mode_history: list[bool] = []

        self.node.create_subscription(
            RobotMode, '/robot/mode',
            lambda m: self._mode_history.append(m.mode), 10)
        self.node.create_subscription(
            Twist, '/cmd_vel_retreat',
            lambda m: self._retreat_history.append(m), 10)
        self.node.create_subscription(
            PointStamped, '/person/predicted_position',
            lambda m: self._pred_pos_history.append(m), 10)
        self.node.create_subscription(
            Bool, '/person/search_mode',
            lambda m: self._search_mode_history.append(m.data), 10)

        self.pub_person = self.node.create_publisher(
            PersonStatus, '/person/status', 10)
        self.cli_mode = self.node.create_client(SetMode, '/mode_manager/set_mode')

        assert self.cli_mode.wait_for_service(timeout_sec=5.0)
        time.sleep(3.5)
        self._mode_history.clear()
        self._retreat_history.clear()

    def tearDown(self):
        self.node.destroy_node()

    # ── ヘルパー ──────────────────────────────────────────────
    def _spin(self, sec: float):
        deadline = time.time() + sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _set_mode(self, mode: int):
        req = SetMode.Request()
        req.requested_mode = mode
        req.requester      = 'test'
        future = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        return future.result()

    def _wait_mode(self, mode: int, timeout: float = 3.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            if self._mode_history and self._mode_history[-1] == mode:
                return True
        return False

    def _publish_person(self, x: float, y: float,
                        is_lost: bool = False,
                        lost_reason: str = ''):
        msg = PersonStatus()
        msg.header.stamp    = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.position.x      = x
        msg.position.y      = y
        msg.confidence      = 0.0 if is_lost else 0.9
        msg.is_lost         = is_lost
        msg.lost_reason     = lost_reason
        self.pub_person.publish(msg)

    # ════════════════════════════════════════════════════════
    # Scenario A-1: 試験員接近 → /cmd_vel_retreat 発行
    #
    # 注意（followLogic.md v2 移行に伴う既知の制約）:
    # 新ロジックの退避方向は find_nearest_open_direction による
    # 「地図上の最空きスペース方向」であり、試験員の位置とは無関係に
    # 決定される。costmap を配信しないこの stub 環境では試験員自身が
    # 障害物としてマップに現れないため、試験員の方向がそのまま
    # 「最空きスペース」として選ばれ得る（＝試験員へ向かって前進する
    # 可能性がある）。したがって本シナリオの「後退方向確認」は
    # 実際の costmap（Gazebo + LiDAR）がある環境でのみ意味を持つ。
    # Docker + Gazebo 環境で本テストを実行し、退避方向が実際に
    # 試験員を避けているか確認すること。
    # ════════════════════════════════════════════════════════

    def test_scenario_A1_retreat_on_approach(self):
        """
        試験員がロボットに trigger 距離より近く接近したとき
        /cmd_vel_retreat に後退指令が発行される。
        (4.3.7 節)
        """
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)
        self._retreat_history.clear()

        # trigger_distance(0.8m) より近い位置に試験員を配置
        for _ in range(15):
            self._publish_person(x=0.5, y=0.0)
            self._spin(0.1)

        retreat_cmds = [t for t in self._retreat_history
                        if abs(t.linear.x) > 0.01 or abs(t.angular.z) > 0.01]
        assert len(retreat_cmds) > 0, (
            '試験員接近時に /cmd_vel_retreat に後退指令が来なかった')

        # 後退方向確認（試験員が前方 → linear.x が負）
        backwards = [t for t in retreat_cmds if t.linear.x < 0]
        assert len(backwards) > 0, (
            '試験員が前方にいるのに後退（linear.x < 0）しなかった')

    # ════════════════════════════════════════════════════════
    # Scenario A-2: 壁際で退避不可 → その場停止
    # ════════════════════════════════════════════════════════

    def test_scenario_A2_stop_when_retreat_blocked(self):
        """
        costmap が退避方向を blockしている場合、
        /cmd_vel_retreat にゼロが発行される（その場停止）。
        costmap なし環境では follow_planner_core の unit test で検証済み。
        このテストは統合環境でのゼロ発行を確認する。
        """
        # costmap が来ない環境（stub 環境）では costmap_is_free が常に True
        # そのため退避コマンドが出ることを確認（壁際シナリオは core UT で保証）
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)
        self._retreat_history.clear()

        for _ in range(10):
            self._publish_person(x=0.3, y=0.0)   # 極近接
            self._spin(0.1)

        # costmap stub (常に free) では退避コマンドが来る
        assert len(self._retreat_history) > 0, (
            '/cmd_vel_retreat が全く来なかった')

    # ════════════════════════════════════════════════════════
    # Scenario A-3: 退避解除 → 通常追従に戻る
    # ════════════════════════════════════════════════════════

    def test_scenario_A3_retreat_releases_on_distance(self):
        """
        距離が release_distance(1.2m) を超えたとき退避が解除され
        /cmd_vel_retreat の発行が止まる（twist_mux タイムアウトで自動解除）。
        """
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)

        # まず接近させて退避開始
        for _ in range(10):
            self._publish_person(x=0.5, y=0.0)
            self._spin(0.1)

        # 遠ざかる (release_distance = 1.2m 超)
        # まず 1.5m を数回送って follow_planner の退避を止めてからクリア
        for _ in range(5):
            self._publish_person(x=1.5, y=0.0)
            self._spin(0.1)
        self._retreat_history.clear()

        # 退避解除後は retreat にゼロのみが来るはず
        for _ in range(10):
            self._publish_person(x=1.5, y=0.0)
            self._spin(0.1)

        non_zero = [t for t in self._retreat_history
                    if abs(t.linear.x) > 0.01]
        assert len(non_zero) == 0, (
            '距離が release を超えても退避指令が来続けた（ハンチング）')

    # ════════════════════════════════════════════════════════
    # Scenario C: 試験員ロスト → 予測外挿 → 捜索旋回
    # ════════════════════════════════════════════════════════

    def test_scenario_C_prediction_and_search(self):
        """
        試験員がロストしたとき:
        1. 予測位置が max_predict_sec 間は外挿される
        2. max_predict_sec 後に /person/search_mode が True になる
        (4.2 節)
        """
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)

        # まず検出状態にする
        for _ in range(10):
            self._publish_person(x=1.5, y=0.0)
            self._spin(0.1)

        self._pred_pos_history.clear()
        self._search_mode_history.clear()

        # ロスト状態に切替
        for _ in range(5):
            self._publish_person(x=1.5, y=0.0,
                                 is_lost=True, lost_reason='DETECTION_LOST')
            self._spin(0.1)

        # 予測フェーズ (max_predict_sec=2.0s): search_mode が False
        # 0.5s(lost phase) + 1.2s = 1.7s < 2.0s で境界条件を回避
        self._spin(1.2)
        search_during_predict = [s for s in self._search_mode_history if s]
        assert len(search_during_predict) == 0, (
            '予測フェーズ中に捜索モードが起動した（予期しない動作）')

        # max_predict_sec 経過後: search_mode が True になる
        self._search_mode_history.clear()
        self._spin(1.5)   # さらに 1.5s 待つ (合計 3.2s > max_predict_sec 2.0s)

        search_active = [s for s in self._search_mode_history if s]
        assert len(search_active) > 0, (
            'max_predict_sec 後に /person/search_mode が True にならなかった')

    # ════════════════════════════════════════════════════════
    # Scenario D: MANUAL 中は退避が作動しない
    # ════════════════════════════════════════════════════════

    def test_scenario_D_no_retreat_in_manual(self):
        """
        MANUAL モード中は follow_planner の近接退避が無効化される。
        操作者がロボット近くにいても退避しない。
        (4.3.7 §8 節)
        """
        self._wait_mode(RobotMode.IDLE)
        self._set_mode(RobotMode.FOLLOWING)
        self._wait_mode(RobotMode.FOLLOWING)
        self._set_mode(RobotMode.MANUAL)
        self._wait_mode(RobotMode.MANUAL)

        self._retreat_history.clear()

        # trigger 距離以内に試験員（操作者）
        for _ in range(15):
            self._publish_person(x=0.4, y=0.0)
            self._spin(0.1)

        non_zero_retreat = [t for t in self._retreat_history
                            if abs(t.linear.x) > 0.01]
        assert len(non_zero_retreat) == 0, (
            'MANUAL 中に /cmd_vel_retreat に非ゼロが来た（4.3.7 §8 違反）')


if __name__ == '__main__':
    pytest.main([__file__, '-v', '-s'])
