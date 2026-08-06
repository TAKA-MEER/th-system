#!/usr/bin/env python3
"""
slam_control.py — ROS2 ノード本体
====================================
WebUI からの地図作成 開始/停止 要求を仲介する。

担当:
  - /robot/mode 購読 → IDLE/MANUAL 以外での切替を拒否（サーバー側の安全
    ガード。UI 側の表示制御だけに頼らない。config_manager.py と同一ロジック）
  - /slam_control/toggle_mapping → slam_toolbox の
    /slam_toolbox/pause_new_measurements (型 slam_toolbox/srv/Pause、
    呼ぶたびに一時停止状態をトグルする) を呼び出して転送する。
    レスポンスの `status` フィールドは実機確認の結果、トグル後の状態
    ではなく呼び出し成功フラグ（常に true）だったため、現在のマッピング
    状態は slam_control 自身がローカルに管理する（このノードが唯一の
    呼び出し元である前提。呼び出しが成功した時点でこちら側のトグルを
    確定させる）。MultiThreadedExecutor + ReentrantCallbackGroup 下では
    連続押下等で複数の /slam_control/toggle_mapping 呼び出しが並行実行
    されうるため、self._mapping_active の読み取り→反転→書き込みと
    slam_toolbox への呼び出しをまとめて Lock で直列化する（実機検証で、
    ロックなしでは短時間の2連続押下により slam_toolbox 側は
    pause→resume と2回トグルされ実質変化なしに戻る一方、こちら側の
    ローカル状態は競合により不整合になる事象を確認したため）
  - /slam_control/mapping_active (std_msgs/Bool, transient_local) に
    現在のマッピング状態を publish する。起動直後は明示的に一度 pause を
    呼び、地図作成を停止状態から始める（VISION.md §7 参照）。
    mode_manager の /robot/mode 同様、状態変化時だけでなく一定周期でも
    再送信する（transient_local だけに頼ると、rosbridge 経由で遅れて
    購読した WebUI が変化直後の一度きりの publish を取りこぼし、
    表示が古いまま固まって見えることがあるため）

スレッドモデル: config_manager.py と同じ理由（サービスコールバックの中から
別サービスを呼ぶ構成は単純な call_async 発火だけだとハングしうる）で
MultiThreadedExecutor + ReentrantCallbackGroup + spin_until_future_complete
を使う。
"""

STARTUP_PAUSE_TIMEOUT_SEC = 60.0   # サービスの存在待ち (wait_for_service)。
                                    # 実機/シム検証で async_slam_toolbox_node
                                    # の初期化 (ソルバプラグイン読み込み等) が
                                    # Gazebo 起動等との輻輳で 25 秒以上かかる
                                    # ケースを確認したため、以前の 5 秒では
                                    # サービス出現前にスキップしてしまい、
                                    # 一時停止が一度も実行されないまま以降の
                                    # トグルが全てずれる不具合があった
STARTUP_CALL_TIMEOUT_SEC  = 20.0   # 起動直後の初回呼び出しの応答待ち。
                                    # Gazebo spawn 等と輻輳し、サービスは
                                    # 既に存在していても応答が数秒遅れる
                                    # ことがあるため、通常運用時より長めに取る
SERVICE_TIMEOUT_SEC       = 2.0    # 通常運用時(ボタン押下時)の応答待ち
STATUS_PUBLISH_PERIOD_SEC = 0.5

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from slam_toolbox.srv import Pause

from th_system_msgs.msg import RobotMode

from th_config_manager.service_call import call_and_wait


class SlamControl(Node):
    def __init__(self):
        super().__init__('slam_control')
        self._mode = RobotMode.IDLE
        self._executor = None   # main() で MultiThreadedExecutor を渡す
        self._mapping_active = False   # 起動直後は停止状態から始める前提の既定値
        # toggle_mapping の連続押下等による並行実行から
        # self._mapping_active の読み取り→反転→書き込みを保護する
        self._toggle_lock = threading.Lock()

        cbg = ReentrantCallbackGroup()
        self.create_subscription(RobotMode, '/robot/mode', self._cb_mode, 10,
                                  callback_group=cbg)

        self._pause_client = self.create_client(
            Pause, '/slam_toolbox/pause_new_measurements', callback_group=cbg)

        status_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_active = self.create_publisher(
            Bool, '/slam_control/mapping_active', status_qos)

        self.create_service(
            Trigger, '/slam_control/toggle_mapping', self._cb_toggle,
            callback_group=cbg)

        # mode_manager の /robot/mode と同様、定期的に現在状態を再送信する
        # (rosbridge 経由の遅延購読が変化直後の一度きりの publish を
        #  取りこぼし、WebUI の表示が古いまま固まるのを防ぐため)
        self.create_timer(STATUS_PUBLISH_PERIOD_SEC, self._publish_timer_cb,
                           callback_group=cbg)

        self.get_logger().info('slam_control 起動')

    def _cb_mode(self, msg: RobotMode):
        self._mode = msg.mode

    def _mode_allows_change(self) -> bool:
        return self._mode in (RobotMode.IDLE, RobotMode.MANUAL)

    def _cb_toggle(self, request, response):
        if not self._mode_allows_change():
            response.success = False
            response.message = 'IDLE/MANUAL モード中のみ操作できます'
            return response

        # 連続押下等による並行実行を直列化する。ロックなしだと、2つの
        # 呼び出しが重なった場合に slam_toolbox 側は pause→resume と
        # 2回トグルされて実質変化なしに戻る一方、こちら側のローカル状態
        # (読み取り→反転→書き込み) が競合し実体とズレることを確認済み。
        with self._toggle_lock:
            if not self._pause_client.wait_for_service(timeout_sec=1.0):
                response.success = False
                response.message = 'slam_toolbox に接続できません'
                return response

            _, err = call_and_wait(
                self, self._pause_client, Pause.Request(), SERVICE_TIMEOUT_SEC)
            if err:
                response.success = False
                response.message = f'pause_new_measurements 呼び出し失敗: {err}'
                return response

            # pause_new_measurements は呼ぶたびに実際の一時停止状態をトグル
            # する (レスポンスの status では現在状態を判別できないため、
            # 呼び出しが成功した時点でこちら側のローカル状態も追従してトグ
            # ルする)
            self._set_active(not self._mapping_active)

        response.success = True
        response.message = 'OK'
        return response

    def _set_active(self, active: bool, warn_on_stop: bool = True):
        self._mapping_active = active
        self._pub_active.publish(Bool(data=self._mapping_active))

        # 起動時の初期停止(_force_initial_pause)では警告しない。設計どおりの
        # 初期状態であり、この時点ではまだ地図も自己位置も無いため。
        if not warn_on_stop:
            return

        # 停止は「地図の更新を止める」だけでなく「自己位置推定も止める」。
        # slam_toolbox の pause_new_measurements はスキャン処理そのものを止める
        # 実装で (shouldProcessScan が isPaused(NEW_MEASUREMENTS) で早期 return し、
        # map_to_odom_ は処理経路内でしか更新されない)、停止後は map→odom が
        # 最後の値のまま凍結してデッドレコニングになる。
        # 2026-08-07 実機で、停止後の中速走行で地図と自己位置が大きくズレ、
        # tf2_echo map odom が完全に凍結することを確認した。
        # 恒久対処が入るまでの暫定運用は「走行中は停止しない」(VISION.md §8)。
        # 黙って破綻させないため、停止するたびに警告する。
        if not active:
            self.get_logger().warn(
                '地図作成を停止しました。**自己位置補正(map→odom)も同時に停止します**。'
                'このまま走行するとデッドレコニングのみになり、地図と自己位置が'
                'ズレ続けます。走行する場合は地図作成を再開してください '
                '(VISION.md §8「地図作成停止後の自己位置推定」)')

    def _publish_timer_cb(self):
        self._pub_active.publish(Bool(data=self._mapping_active))

    def _force_initial_pause(self):
        """起動直後に一度 pause を呼び、地図作成を停止状態から始める。

        slam_toolbox は起動直後デフォルトでマッピング有効(非一時停止)の
        ため、ここで一度トグルすれば必ず一時停止状態になる
        (self._mapping_active の初期値 False と対応させる)。
        """
        if not self._pause_client.wait_for_service(
                timeout_sec=STARTUP_PAUSE_TIMEOUT_SEC):
            self.get_logger().warn(
                'slam_toolbox 未起動のため初期停止状態への遷移をスキップ')
            return
        _, err = call_and_wait(
            self, self._pause_client, Pause.Request(), STARTUP_CALL_TIMEOUT_SEC)
        if err:
            self.get_logger().warn(
                f'初期 pause 呼び出し失敗: {err} '
                '(地図作成は停止されていない可能性があります。'
                'WebUI から手動で開始/停止し直してください)')
            return
        self._set_active(False, warn_on_stop=False)
        self.get_logger().info('初期状態: 地図作成停止')


def main(args=None):
    rclpy.init(args=args)
    node = SlamControl()
    executor = MultiThreadedExecutor()
    node._executor = executor
    executor.add_node(node)
    node._force_initial_pause()
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
