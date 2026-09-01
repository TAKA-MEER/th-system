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
  | /slam_control/discard_map      | Trigger | slam_toolbox を終了 → respawn で再起動 |

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
slam_toolbox にポーズグラフを空へ戻すサービスは無い。このためプロセスごと
終了させ、`bringup.launch.py` の `respawn=True` で真っさらに立ち上げ直す。

当初は起動直後の空ポーズグラフを `serialize_map` で退避し、破棄要求時に
`deserialize_map` で読み戻す方式にしていたが、2026-08-07 の実機で
**slam_toolbox が SIGSEGV で落ちた**。空に近いグラフの読み込みは想定されて
いないと判断して廃止した（それ以前は localization モード中に呼んでいたため
何もせず「OK」を返していた。VISION.md §8 の落とし穴 1・2 を参照）。

クラッシュ耐性
--------------
`map_and_localization_slam_toolbox_node` は slam_toolbox の `experimental/`
配下の実装で、実機で SIGSEGV を確認している。落ちたままだと map→odom が
消えて自己位置が失われるうえ、他ノードは全て生きているため気づきにくい。
`respawn` で自動復帰させ、こちらはサービスの消失→再出現を検知して
モードを再適用する（_check_slam_restart）。破棄もこの経路に相乗りしている。

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

import os
import signal
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String

from th_system_msgs.msg import RobotMode

from th_config_manager.service_call import call_and_wait


# 破棄で終了させる対象の実行ファイル名。bringup.launch.py が起動するものと
# 一致させること。pkill -f のようなパターン照合ではなく /proc を直接見るのは、
# パターンが自分自身のコマンドラインにマッチして自滅する事故を避けるため
# (CLAUDE.md「環境の癖」参照)。
SLAM_EXECUTABLE = 'map_and_localization_slam_toolbox_node'


def _find_slam_toolbox_pids():
    """slam_toolbox の PID を /proc から探す（自分自身は必ず除外する）。"""
    me = os.getpid()
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmdline = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        except OSError:
            continue   # 読む前に終了した等
        if SLAM_EXECUTABLE in cmdline:
            found.append(pid)
    return found


class SlamControl(Node):
    def __init__(self):
        super().__init__('slam_control')
        self._mode = RobotMode.IDLE
        self._executor = None   # main() で MultiThreadedExecutor を渡す
        # WS-8B: enable_route_slam のとき true。起動時に localization へ倒さず
        # slam_toolbox を既定の mapping モードのまま走らせる（教示・再生で
        # map→odom を連続補正するため）。
        self.declare_parameter('startup_mapping', False)
        self._startup_mapping = bool(
            self.get_parameter('startup_mapping').value)
        self._mapping_active = self._startup_mapping   # 起動直後の状態
        # slam_toolbox のサービスが見えているか。None = まだ一度も判定していない。
        # 消失→再出現を respawn による再起動とみなす (_check_slam_restart)
        self._slam_ready = None
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
        self._check_slam_restart()

    def _check_slam_restart(self):
        """slam_toolbox の再起動を検知して、こちらの状態を再適用する。

        slam_toolbox は SIGSEGV で落ちることが実機で確認されており
        (2026-08-07)、bringup では respawn=True で自動再起動する。再起動した
        ノードは mapping モードで立ち上がるため、こちらが「停止中」と表示して
        いても実際には地図が更新され続けることになる。サービスの消失→再出現を
        再起動とみなし、望みのモードを入れ直す。

        空ポーズグラフも取り直す。再起動でグラフは空に戻っており、この瞬間が
        最も「空」に近いスナップショットを取れるタイミングであるため。
        """
        ready = self._cli_mode.service_is_ready()
        was_ready = self._slam_ready
        self._slam_ready = ready
        if was_ready is None or ready == was_ready:
            return

        if not ready:
            self.get_logger().error(
                'slam_toolbox が応答しなくなりました (クラッシュの可能性)。'
                ' 自己位置補正が止まります。respawn による再起動を待ちます')
            self._report('NG: slam_toolbox が停止しました (再起動待ち)')
            return

        self.get_logger().warn('slam_toolbox の再起動を検知しました。状態を再適用します')
        if not self._lock.acquire(blocking=False):
            self._slam_ready = was_ready   # 次周期でやり直す
            return
        try:
            desired = self._mapping_active
            err = self._apply_mapping(desired)
            if err:
                self.get_logger().error(f'モードの再適用に失敗: {err}')
                self._report('NG: 再起動後のモード再適用に失敗しました')
            else:
                self._report(
                    f'OK: slam_toolbox 再起動後に'
                    f'{"地図作成中" if desired else "地図作成停止"}を再適用しました')
        finally:
            self._lock.release()

    def _report(self, text: str):
        self._pub_last_result.publish(String(data=text))

    def _set_localization(self, localization: bool) -> "str | None":
        """slam_toolbox のモードを切り替える。エラー文字列 or None を返す。"""
        if not self._cli_mode.wait_for_service(timeout_sec=1.0):
            return 'slam_toolbox に接続できません'
        _, err = call_and_wait(
            self, self._cli_mode, SetBool.Request(data=localization),
            SERVICE_TIMEOUT_SEC)
        if err:
            return f'set_localization_mode 呼び出し失敗: {err}'
        return None

    def _apply_mapping(self, active: bool) -> "str | None":
        """マッピングの有効/無効を切り替える。エラー文字列 or None を返す。"""
        err = self._set_localization(not active)
        if err:
            return err
        self._set_active(active)
        return None

    def _in_mapping_mode(self, fn) -> "str | None":
        """mapping モードでしか受け付けられない操作を実行する。

        LocalizationSlamToolbox は serializePoseGraphCallback と
        deserializePoseGraphCallback を override しており、localization モード中は
        何もせずエラーを返す（serialize は無条件、deserialize は match_type が
        LOCALIZE_AT_POSE 以外なら拒否）。

        しかも呼び出し側からは成功と区別がつかない。DeserializePoseGraph.Response
        にはフィールドが無く、SerializePoseGraph.Response.result も未設定なら
        0 (=RESULT_SUCCESS) になるためである。実際 2026-08-07 の実機で、
        地図の破棄が「OK」を返しながら何もしていない事象が発生した。
        必ずこのヘルパー経由で mapping モードへ入れてから呼ぶこと。

        元のモードは処理後に復元する。一時的に mapping モードへ入る間にスキャンが
        グラフへ入りうるが、破棄では直後に mapper ごと差し替わるため影響は無く、
        保存では現在地のスキャンが1枚多く入るだけで実害は無い。
        """
        was_mapping = self._mapping_active
        if not was_mapping:
            err = self._set_localization(False)
            if err:
                return err
        try:
            return fn()
        finally:
            if not was_mapping:
                restore_err = self._set_localization(True)
                if restore_err:
                    self.get_logger().error(
                        f'モードを停止状態へ戻せませんでした: {restore_err}')

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
        """ポーズグラフを path.posegraph / path.data へ書き出す。

        成否は応答ではなくファイルの実在で判定する。SerializePoseGraph.Response
        の result は slam_toolbox が早期 return したとき未設定のままとなり、
        0 (=RESULT_SUCCESS) に見えてしまうため信用できない（_in_mapping_mode の
        説明を参照）。slam_control は slam_toolbox と同一コンテナで動くので、
        書けたかどうかは直接確認できる。
        """
        if not self._cli_serialize.wait_for_service(timeout_sec=1.0):
            return 'slam_toolbox に接続できません'
        os.makedirs(os.path.dirname(path), exist_ok=True)
        graph_path = path + '.posegraph'
        before = os.path.getmtime(graph_path) if os.path.exists(graph_path) else None

        def _call():
            req = SerializePoseGraph.Request()
            req.filename = path
            _, err = call_and_wait(
                self, self._cli_serialize, req, SERVICE_TIMEOUT_SEC)
            return f'serialize_map 呼び出し失敗: {err}' if err else None

        err = self._in_mapping_mode(_call)
        if err:
            return err

        if not os.path.exists(graph_path):
            return (f'ポーズグラフが書き出されませんでした ({graph_path} が無い)。'
                    ' slam_toolbox のログを確認してください')
        if before is not None and os.path.getmtime(graph_path) == before:
            return (f'ポーズグラフが更新されませんでした ({graph_path} の更新時刻が'
                    ' 変わっていない)。slam_toolbox のログを確認してください')
        return None

    # ── 地図の破棄 ──────────────────────────────────────────
    def _cb_discard_map(self, request, response):
        """slam_toolbox を作り直して地図を捨てる。

        slam_toolbox にポーズグラフを空へ戻すサービスは無い。当初は起動直後の
        空グラフを `deserialize_map` で読み戻す方式にしていたが、2026-08-07 の
        実機で **slam_toolbox が SIGSEGV で落ちた**。空に近いグラフの読み込みは
        想定されていないと判断し廃止した。

        代わりにプロセスごと終了させ、`bringup.launch.py` の `respawn=True` で
        真っさらに立ち上げ直す。再起動の検知とモードの再適用は
        _check_slam_restart() が行うため、ここでは終了させるだけでよい。
        """
        rejected = self._reject_if_mode_disallows(response)
        if rejected:
            return rejected

        with self._lock:
            pids = _find_slam_toolbox_pids()
            if not pids:
                return self._finish(
                    response, 'slam_toolbox のプロセスが見つかりません', '')
            # 破棄後は「地図作成停止」状態から始める（起動直後と同じ）。
            # 再起動した slam_toolbox は mapping モードで立ち上がるので、
            # _check_slam_restart がこの値を見て停止状態を入れ直す。
            self._set_active(False)
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                    self.get_logger().warn(
                        f'地図を破棄するため slam_toolbox (pid {pid}) を終了させました。'
                        ' respawn による再起動を待ちます')
                except OSError as e:
                    return self._finish(
                        response, f'slam_toolbox (pid {pid}) を終了できません: {e}', '')

        return self._finish(
            response, None, '地図を破棄しました（slam_toolbox を再起動中）')

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
        """起動時の初期モードを確定させる。

        既定 (startup_mapping=false): localization モード（=地図作成停止）へ倒す。
        slam_toolbox は起動直後 mapping モードで立ち上がるため、ここで倒さないと
        「停止中」と表示したまま地図が更新され続ける。VISION.md §8 の
        「起動直後は地図作成を停止した状態にする」を満たすための処理。

        WS-8B (startup_mapping=true): 倒さず mapping モードのまま。教示・再生が
        map→odom の連続補正を必要とするため。ローカル状態は mapping 有効で始める。
        """
        if not self._cli_mode.wait_for_service(
                timeout_sec=STARTUP_SERVICE_TIMEOUT_SEC):
            self.get_logger().warn(
                'slam_toolbox 未起動のため初期化をスキップします '
                '(地図作成が停止されていない可能性があります。'
                'WebUI から明示的に開始/停止し直してください)')
            return

        if self._startup_mapping:
            self._set_active(True)
            self.get_logger().info(
                '初期状態: 地図作成 継続（WS-8B / startup_mapping=true）')
            self._slam_ready = True
            return

        err = self._apply_mapping(False)
        if err:
            self.get_logger().warn(f'初期モード設定に失敗: {err}')
            return
        self.get_logger().info('初期状態: 地図作成停止（自己位置推定は継続）')
        # ここまで来た＝サービスは見えている。以降の消失→再出現を再起動として
        # 扱えるよう基準を確定させる (_check_slam_restart)
        self._slam_ready = True


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
