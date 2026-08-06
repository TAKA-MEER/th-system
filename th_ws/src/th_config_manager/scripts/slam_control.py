#!/usr/bin/env python3
"""
slam_control.py — ROS2 ノード本体
====================================
WebUI からの地図操作要求を仲介する。

担当:
  - /robot/mode 購読 → IDLE/MANUAL 以外での操作を拒否（サーバー側の安全
    ガード。UI 側の表示制御だけに頼らない。config_manager.py と同一ロジック）
  - 地図操作を slam_toolbox のサービス呼び出しへ転送する（下表）
  - /slam_control/mapping_active (std_msgs/Bool, transient_local) に
    現在のマッピング状態を publish する

提供する操作:

  | slam_control のサービス        | 型      | 転送先                                |
  |--------------------------------|---------|---------------------------------------|
  | /slam_control/toggle_mapping   | Trigger | set_localization_mode（トグル）       |
  | /slam_control/set_mapping      | SetBool | set_localization_mode（明示指定）     |
  | /slam_control/save_map         | Trigger | save_map + serialize_map              |
  | /slam_control/discard_map      | Trigger | deserialize_map（起動時の空グラフ）   |

「地図作成停止」の意味（VISION.md §8）
--------------------------------------
停止は「地図の更新を止める」であって「自己位置推定を止める」ではない。
停止後も待機・呼び寄せ・配電盤移動を行う以上、map→odom は走行中ずっと
更新され続けなければならない。

このため slam_toolbox は `map_and_localization_slam_toolbox_node` で起動し、
`/slam_toolbox/set_localization_mode` (std_srvs/SetBool, true=localization)
でモードを切り替える。localization モードはポーズグラフにノードを追加せず
スキャンマッチングだけ行うため、地図は凍結したまま自己位置推定が続く。

2026-08-07 まではここで `pause_new_measurements` を呼んでいたが、これは
スキャン処理そのものを止める実装で（`shouldProcessScan()` が
`isPaused(NEW_MEASUREMENTS)` を見て早期 return し、`map_to_odom_` は処理
経路内でしか更新されない）、停止後は map→odom が凍結して純粋な
デッドレコニングになっていた。実機で停止後の中速走行により地図と自己位置が
大きくズレ、`tf2_echo map odom` が完全に凍結することを確認している。

地図の破棄について
------------------
slam_toolbox にポーズグラフを空へ戻すサービスは無い。このため起動直後
（＝まだスキャンを 1 枚も取り込んでいない localization モードの状態）で
`serialize_map` して「空のポーズグラフ」をファイルに退避しておき、破棄要求時に
それを `deserialize_map` で読み戻す。ノードを再起動せずに初期状態へ戻せる。

スレッドモデル: config_manager.py と同じ理由（サービスコールバックの中から
別サービスを呼ぶ構成は単純な call_async 発火だけだとハングしうる）で
MultiThreadedExecutor + ReentrantCallbackGroup + spin_until_future_complete
を使う。
"""

STARTUP_SERVICE_TIMEOUT_SEC = 60.0  # サービスの存在待ち (wait_for_service)。
                                     # 実機/シム検証で slam_toolbox の初期化
                                     # (ソルバプラグイン読み込み等) が Gazebo 起動等
                                     # との輻輳で 25 秒以上かかるケースを確認した
                                     # ため、以前の 5 秒ではサービス出現前に
                                     # スキップしてしまい、初期状態への遷移が
                                     # 一度も実行されないまま以降の操作が全て
                                     # ずれる不具合があった
STARTUP_CALL_TIMEOUT_SEC    = 20.0  # 起動直後の初回呼び出しの応答待ち。
                                     # Gazebo spawn 等と輻輳し、サービスは既に
                                     # 存在していても応答が数秒遅れることがある
SERVICE_TIMEOUT_SEC         = 5.0   # 通常運用時(ボタン押下時)の応答待ち。
                                     # serialize/deserialize はディスク I/O を
                                     # 伴うため toggle より長めに取る
STATUS_PUBLISH_PERIOD_SEC   = 0.5

# 起動直後の空ポーズグラフの退避先。slam_toolbox が
# <filename>.posegraph と <filename>.data を作る。
EMPTY_POSEGRAPH_PATH = '/tmp/th_slam_empty'

import os
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from slam_toolbox.srv import (
    DeserializePoseGraph, SaveMap, SerializePoseGraph)
from std_msgs.msg import String

from th_system_msgs.msg import RobotMode

from th_config_manager.service_call import call_and_wait


class SlamControl(Node):
    def __init__(self):
        super().__init__('slam_control')
        self._mode = RobotMode.IDLE
        self._executor = None   # main() で MultiThreadedExecutor を渡す
        self._mapping_active = False   # 起動直後は停止状態から始める
        self._empty_graph_ready = False
        # 操作の並行実行から状態の読み取り→更新と slam_toolbox 呼び出しまでを
        # まとめて保護する。ReentrantCallbackGroup 下では連続押下等で複数の
        # 要求が並行実行されうるため（実機検証で、ロックなしでは短時間の2連続
        # 押下により slam_toolbox 側とこちらのローカル状態が不整合になる事象を
        # 確認したため）
        self._lock = threading.Lock()

        cbg = ReentrantCallbackGroup()
        self.create_subscription(RobotMode, '/robot/mode', self._cb_mode, 10,
                                  callback_group=cbg)

        self._cli_mode = self.create_client(
            SetBool, '/slam_toolbox/set_localization_mode', callback_group=cbg)
        self._cli_serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=cbg)
        self._cli_deserialize = self.create_client(
            DeserializePoseGraph, '/slam_toolbox/deserialize_map', callback_group=cbg)
        self._cli_save = self.create_client(
            SaveMap, '/slam_toolbox/save_map', callback_group=cbg)

        status_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_active = self.create_publisher(
            Bool, '/slam_control/mapping_active', status_qos)
        # 直近の操作結果。WebUI が結果表示に使う（サービス応答を取りこぼしても
        # 状態が分かるようにするため）
        self._pub_last_result = self.create_publisher(
            String, '/slam_control/last_result', status_qos)

        self.create_service(
            Trigger, '/slam_control/toggle_mapping', self._cb_toggle,
            callback_group=cbg)
        self.create_service(
            SetBool, '/slam_control/set_mapping', self._cb_set_mapping,
            callback_group=cbg)
        self.create_service(
            Trigger, '/slam_control/save_map', self._cb_save_map,
            callback_group=cbg)
        self.create_service(
            Trigger, '/slam_control/discard_map', self._cb_discard_map,
            callback_group=cbg)

        # mode_manager の /robot/mode と同様、定期的に現在状態を再送信する
        # (rosbridge 経由の遅延購読が変化直後の一度きりの publish を
        #  取りこぼし、WebUI の表示が古いまま固まるのを防ぐため)
        self.create_timer(STATUS_PUBLISH_PERIOD_SEC, self._publish_timer_cb,
                           callback_group=cbg)

        self.get_logger().info('slam_control 起動')

    # ── 共通 ────────────────────────────────────────────────
    def _cb_mode(self, msg: RobotMode):
        self._mode = msg.mode

    def _mode_allows_change(self) -> bool:
        return self._mode in (RobotMode.IDLE, RobotMode.MANUAL)

    def _reject_if_mode_disallows(self, response):
        if self._mode_allows_change():
            return None
        response.success = False
        response.message = 'IDLE/MANUAL モード中のみ操作できます'
        return response

    def _set_active(self, active: bool):
        self._mapping_active = active
        self._pub_active.publish(Bool(data=self._mapping_active))

    def _publish_timer_cb(self):
        self._pub_active.publish(Bool(data=self._mapping_active))

    def _report(self, text: str):
        self._pub_last_result.publish(String(data=text))

    def _apply_mapping(self, active: bool) -> "str | None":
        """slam_toolbox のモードを切り替える。エラー文字列 or None を返す。

        set_localization_mode は「localization を有効にするか」なので、
        マッピングを有効にしたいときは False を渡す。
        """
        if not self._cli_mode.wait_for_service(timeout_sec=1.0):
            return 'slam_toolbox に接続できません'
        _, err = call_and_wait(
            self, self._cli_mode, SetBool.Request(data=not active),
            SERVICE_TIMEOUT_SEC)
        if err:
            return f'set_localization_mode 呼び出し失敗: {err}'
        self._set_active(active)
        return None

    # ── 地図作成 開始/停止 ──────────────────────────────────
    def _cb_toggle(self, request, response):
        rejected = self._reject_if_mode_disallows(response)
        if rejected:
            return rejected
        with self._lock:
            err = self._apply_mapping(not self._mapping_active)
        return self._finish(response, err,
                            f'地図作成を{"開始" if self._mapping_active else "停止"}しました')

    def _cb_set_mapping(self, request, response):
        rejected = self._reject_if_mode_disallows(response)
        if rejected:
            return rejected
        with self._lock:
            err = self._apply_mapping(request.data)
        return self._finish(response, err,
                            f'地図作成を{"開始" if self._mapping_active else "停止"}しました')

    # ── 地図の保存 ──────────────────────────────────────────
    def _cb_save_map(self, request, response):
        rejected = self._reject_if_mode_disallows(response)
        if rejected:
            return rejected

        with self._lock:
            # 占有格子 (pgm + yaml)。Nav2 の map_server や外部ツールで読める形式。
            if not self._cli_save.wait_for_service(timeout_sec=1.0):
                return self._finish(response, 'slam_toolbox に接続できません', '')
            os.makedirs(os.path.dirname(self._map_basename()), exist_ok=True)
            req = SaveMap.Request()
            req.name = String(data=self._map_basename())
            result, err = call_and_wait(
                self, self._cli_save, req, SERVICE_TIMEOUT_SEC)
            if err:
                return self._finish(response, f'save_map 呼び出し失敗: {err}', '')
            # 呼び出しが成功しても結果コードで失敗を返してくる
            # (地図が1枚も無い場合など)。応答を見ないと「保存した」と嘘をつく。
            if result.result != SaveMap.Response.RESULT_SUCCESS:
                reason = ('地図がまだ生成されていません'
                          if result.result == SaveMap.Response.RESULT_NO_MAP_RECEIEVD
                          else f'result={result.result}')
                return self._finish(response, f'地図の保存に失敗: {reason}', '')

            # ポーズグラフ。占有格子と違い、読み戻して SLAM を再開できる。
            err = self._serialize(self._map_basename())
            if err:
                return self._finish(response, err, '')

        return self._finish(
            response, None, f'地図を保存しました ({self._map_basename()})')

    def _map_basename(self) -> str:
        return os.path.join(
            os.path.expanduser('~'), 'th_maps', 'th_map')

    def _serialize(self, path: str) -> "str | None":
        if not self._cli_serialize.wait_for_service(timeout_sec=1.0):
            return 'slam_toolbox に接続できません'
        os.makedirs(os.path.dirname(path), exist_ok=True)
        req = SerializePoseGraph.Request()
        req.filename = path
        result, err = call_and_wait(
            self, self._cli_serialize, req, SERVICE_TIMEOUT_SEC)
        if err:
            return f'serialize_map 呼び出し失敗: {err}'
        if result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            return f'ポーズグラフの書き出しに失敗しました (result={result.result})'
        return None

    # ── 地図の破棄 ──────────────────────────────────────────
    def _cb_discard_map(self, request, response):
        rejected = self._reject_if_mode_disallows(response)
        if rejected:
            return rejected

        if not self._empty_graph_ready:
            return self._finish(
                response,
                '起動時の空ポーズグラフの退避に失敗しているため破棄できません。'
                'slam_control のログを確認してください', '')

        with self._lock:
            if not self._cli_deserialize.wait_for_service(timeout_sec=1.0):
                return self._finish(response, 'slam_toolbox に接続できません', '')
            req = DeserializePoseGraph.Request()
            req.filename = EMPTY_POSEGRAPH_PATH
            # 空グラフなので開始姿勢の指定は意味を持たない。最初のノードから
            # 始める指定にしておく。
            req.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
            _, err = call_and_wait(
                self, self._cli_deserialize, req, SERVICE_TIMEOUT_SEC)
            if err:
                return self._finish(
                    response, f'deserialize_map 呼び出し失敗: {err}', '')
            # 破棄直後は「地図作成停止」状態に揃える（起動直後と同じ状態）。
            err = self._apply_mapping(False)
            if err:
                return self._finish(response, err, '')

        return self._finish(response, None, '地図を破棄しました（停止状態）')

    def _finish(self, response, err: "str | None", ok_message: str):
        if err:
            response.success = False
            response.message = err
            self.get_logger().warn(err)
            self._report(f'NG: {err}')
        else:
            response.success = True
            response.message = ok_message
            self.get_logger().info(ok_message)
            self._report(f'OK: {ok_message}')
        return response

    # ── 起動時の初期化 ──────────────────────────────────────
    def _startup(self):
        """localization モード（=地図作成停止）から始め、空グラフを退避する。

        slam_toolbox は起動直後 mapping モードなので、まず localization へ倒す。
        その時点ではまだポーズグラフにノードが 1 つも入っていないため、ここで
        serialize しておけば「空の地図」のスナップショットになる。破棄操作は
        これを読み戻すことで実現する（slam_toolbox に reset サービスが無いため）。
        """
        if not self._cli_mode.wait_for_service(
                timeout_sec=STARTUP_SERVICE_TIMEOUT_SEC):
            self.get_logger().warn(
                'slam_toolbox 未起動のため初期化をスキップします '
                '(地図作成が停止されていない可能性があります。'
                'WebUI から明示的に開始/停止し直してください)')
            return

        _, err = call_and_wait(
            self, self._cli_mode, SetBool.Request(data=True),
            STARTUP_CALL_TIMEOUT_SEC)
        if err:
            self.get_logger().warn(f'初期モード設定に失敗: {err}')
            return
        self._set_active(False)
        self.get_logger().info('初期状態: 地図作成停止（自己位置推定は継続）')

        err = self._serialize(EMPTY_POSEGRAPH_PATH)
        if err:
            self.get_logger().warn(
                f'空ポーズグラフの退避に失敗: {err} — 地図の破棄操作は使えません')
            return
        self._empty_graph_ready = True
        self.get_logger().info(
            f'空ポーズグラフを退避しました ({EMPTY_POSEGRAPH_PATH})')


def main(args=None):
    rclpy.init(args=args)
    node = SlamControl()
    executor = MultiThreadedExecutor()
    node._executor = executor
    executor.add_node(node)
    node._startup()
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
