#!/usr/bin/env python3
"""
slam_control.py — ROS2 ノード本体
====================================
WebUI / route_recorder / replay_runner からの地図操作要求を仲介する。

担当:
  - /robot/mode 購読 → IDLE/MANUAL 以外での操作を拒否（サーバー側の安全
    ガード。UI 側の表示制御だけに頼らない。config_manager.py と同一ロジック。
    ただし /map_session/open だけは除く。教示・再生の最中に呼ばれるため）
  - 地図操作を slam_toolbox のサービス呼び出しへ転送する（下表）
  - /slam_control/mapping_active (std_msgs/Bool, transient_local) に
    現在のマッピング状態を publish する

提供する操作:

  | slam_control のサービス        | 型             | 転送先                                   |
  |--------------------------------|----------------|------------------------------------------|
  | /slam_control/toggle_mapping   | Trigger        | set_localization_mode（トグル）          |
  | /slam_control/set_mapping      | SetBool        | set_localization_mode（明示指定）        |
  | /slam_control/save_map         | Trigger        | save_map + serialize_map                 |
  | /slam_control/discard_map      | Trigger        | slam_toolbox を終了 → respawn で再起動   |
  | /map_session/open              | OpenMapSession | save: serialize→freeze / reload: freeze→deserialize |

「地図作成停止」の意味（VISION.md §8）
--------------------------------------
停止は「地図の更新を止める」であって「自己位置推定を止める」ではない。
停止後も待機・呼び寄せ・配電盤移動を行う以上、map→odom は走行中ずっと
更新され続けなければならない。

**WS-9N（2026-09-03）: map_and_localization_slam_toolbox_node による切替。**
`bringup.launch.py` で SLAM ノードを `map_and_localization_slam_toolbox_node` へ
差し替え済み。`/slam_toolbox/set_localization_mode`（`std_srvs/srv/SetBool`）により、
同一プロセスのまま地図作成（mapping, data=False）と地図凍結＋自己位置推定継続
（localization, data=True）を切り替える（実機確認済み、success=True を返す）。

地図の保存・再読込（WS-9N）————————
`/map_session/open` は route_recorder（教示の「保存」）と replay_runner（再生の
経路選択）が呼ぶ。`slot:"ROUTE"` / `session_id:<経路名>` / `mode:"save"|"reload"`。

  - save:   mapping モードのまま `serialize_map` で `<map_dir>/<session_id>` へ
    （.posegraph + .data）。localization モードでは serialize が書き出さないため
    （2026-09-03 実機確認）、保存は必ず mapping モードで行う。
    書き出し成功後に `set_localization_mode(True)` を呼んで地図を凍結する。
  - reload: 先に `set_localization_mode(True)` で地図を凍結し、次に `deserialize_map` を
    `match_type: 3`(LOCALIZE_AT_POSE) + 初期姿勢（経路の始点）で呼ぶ。
    ファイルが無ければ success=false。6.9MB のグラフ I/O が重いので専用の
    長めのタイムアウト（DESERIALIZE_TIMEOUT_SEC）を使う。
    読み込み後も localization モードのまま維持し、再生中に地図作成へ戻さない。

   2026-09-03 実機確認:
   - set_localization_mode(true): success=True
   - deserialize(match_type=3, 始点 pose): 成功、map->base_link と始点の差 1.9cm / 5.5°
   - localization モードで 40 秒後の serialize: 書き出されない（＝地図は読み取り専用）
   - mapping モードで 40 秒後の serialize: .posegraph が 27KB 増える（＝再生中に地図作成を走らせてはいけない）
   - 105 秒間の連続稼働で SIGSEGV なし

地図の破棄について
------------------
slam_toolbox にポーズグラフを空へ戻すサービスは無い。このためプロセスごと
終了させ、`bringup.launch.py` の `respawn=True` で真っさらに立ち上げ直す。
`map_and_localization_slam_toolbox_node` を kill する（SLAM_EXECUTABLE / _find_slam_toolbox_pids）。

クラッシュ耐性
--------------
slam_toolbox は実機で SIGSEGV を確認している。落ちたままだと map→odom が
消えて自己位置が失われるうえ、他ノードは全て生きているため気づきにくい。
`respawn` で自動復帰させ、こちらはサービスの消失→再出現を検知して
モードを再適用する（_check_slam_restart）。破棄もこの経路に相乗りしている。

スレッドモデル: config_manager.py と同じ理由（サービスコールバックの中から
別サービスを呼ぶ構成は単純な call_async 発火だけだとハングしうる）で
MultiThreadedExecutor + ReentrantCallbackGroup + spin_until_future_complete
を使う。route_recorder / replay_runner から /map_session/open を同期的に呼ぶ
側も同じ構成（MultiThreadedExecutor + ReentrantCallbackGroup + call_and_wait）
に揃えてある。
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
                                     # serialize はディスク I/O を伴うため
                                     # toggle より長めに取る
DESERIALIZE_TIMEOUT_SEC     = 30.0  # deserialize_map は 6.9MB のポーズグラフを
                                     # 読むのでディスク I/O が重い
                                     # (2026-09-03 実測 .posegraph 6.9MB + .data
                                     # 29KB)。SERVICE_TIMEOUT_SEC(5s) では足りない
                                     # 可能性があるため長めに取る
RESPAWN_WAIT_SEC           = 45.0   # WS-9S: reload のたび slam_toolbox を SIGTERM →
                                     # launch の respawn=True で作り直す。旧プロセス
                                     # 消失 → 新プロセスのサービス復帰までの待ち上限。
                                     # respawn_delay=2.0 + ノード初期化が Gazebo 等と
                                     # 輻輳すると 25s 超（STARTUP_SERVICE_TIMEOUT_SEC の
                                     # コメント参照）になる実績があるため長めに取る
STATUS_PUBLISH_PERIOD_SEC   = 0.5

import os
import signal
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from slam_toolbox.srv import (DeserializePoseGraph, SaveMap,
                              SerializePoseGraph)

from th_system_msgs.msg import RobotMode
from th_system_msgs.srv import OpenMapSession

from th_config_manager.service_call import call_and_wait
from th_config_manager.slam_control_logic import (
    deserialize_match_type, map_session_filename, open_session_error,
    slam_restart_complete,
)


# 破棄で終了させる対象の実行ファイル名。bringup.launch.py が起動するものと
# 一致させること。pkill -f のようなパターン照合ではなく /proc を直接見るのは、
# パターンが自分自身のコマンドラインにマッチして自滅する事故を避けるため
# (CLAUDE.md「環境の癖」参照)。現在の教示・再生で使うのは
# map_and_localization_slam_toolbox_node (WS-9N) なので、これに合わせること。
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
        # WS-9L: /map_session/open の保存先ディレクトリ。経路 JSON と同じ場所
        # (routes_dir) でよい。serialize_map は <name>.posegraph / <name>.data を
        # 作るが、/route/catalog は .json しか拾わないため一覧は汚れない。
        self.declare_parameter('map_dir', '/root/th_data/routes')
        self._map_dir = self.get_parameter('map_dir').value
        # slam_toolbox のサービスが見えているか。None = まだ一度も判定していない。
        # 消失→再出現を respawn による再起動とみなす (_check_slam_restart)
        self._slam_ready = None
        # WS-9S: 再生の地図読み直しは意図的に slam_toolbox を respawn する。その間
        # _check_slam_restart にモード再適用を走らせない（reload 側が最後まで面倒を見る）。
        self._reload_in_progress = False
        # 操作の並行実行から状態の読み取り→更新と slam_toolbox 呼び出しまでを
        # まとめて保護する。ReentrantCallbackGroup 下では連続押下等で複数の
        # 要求が並行実行されうるため（実機検証で、ロックなしでは短時間の2連続
        # 押下により slam_toolbox 側とこちらのローカル状態が不整合になる事象を
        # 確認したため）
        self._lock = threading.Lock()

        cbg = ReentrantCallbackGroup()
        self.create_subscription(RobotMode, '/robot/mode', self._cb_mode, 10,
                                  callback_group=cbg)

        # WS-9N: map_and_localization_slam_toolbox_node に差し替えたため、
        # set_localization_mode サービスが存在する（2026-09-03 実機確認済み、success=True）。
        # 地図作成中 = localization オフ (False)、地図凍結 = localization オン (True)。
        self._cli_localization = self.create_client(
            SetBool, '/slam_toolbox/set_localization_mode', callback_group=cbg)
        self._cli_serialize = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=cbg)
        self._cli_save = self.create_client(
            SaveMap, '/slam_toolbox/save_map', callback_group=cbg)
        # WS-9L: 地図の再読込。教示の地図を読み直して自己位置を経路の始点に合わせる。
        self._cli_deserialize = self.create_client(
            DeserializePoseGraph, '/slam_toolbox/deserialize_map',
            callback_group=cbg)

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
        # WS-9L: 地図の保存・再読込。教示の「保存」と再生の経路選択が呼ぶ。
        # 教示・再生の最中に呼ばれるため _mode_allows_change のガードは**かけない**
        # （slot / mode の検証だけ行う。純ロジックは slam_control_logic.py）。
        self.create_service(
            OpenMapSession, '/map_session/open', self._cb_map_session_open,
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
        if self._reload_in_progress:
            return   # WS-9S: reload が respawn を意図的に起こしている最中は触らない
        ready = self._cli_localization.service_is_ready()
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

    def _set_localization(self, enabled: bool) -> "str | None":
        """slam_toolbox の localization モードを切り替える。エラー文字列 or None を返す。

        /slam_toolbox/set_localization_mode (std_srvs/srv/SetBool):
        - enabled=True:  localization モード（地図凍結・自己位置推定継続）
        - enabled=False: mapping モード（地図作成）

        実機確認 (2026-09-03): サービスが存在し、success=True を返す。
        応答の success を必ず確認し、False の場合はエラー文字列を返す。
        """
        if not self._cli_localization.wait_for_service(timeout_sec=1.0):
            return 'slam_toolbox に接続できません'
        req = SetBool.Request(data=enabled)
        result, err = call_and_wait(
            self, self._cli_localization, req, SERVICE_TIMEOUT_SEC)
        if err:
            return f'set_localization_mode 呼び出し失敗: {err}'
        if result is not None and not result.success:
            msg = result.message if result.message else 'success=False'
            return f'set_localization_mode 失敗: {msg}'
        return None

    def _apply_mapping(self, active: bool) -> "str | None":
        """マッピングの有効/無効を切り替える。エラー文字列 or None を返す。

        地図作成中 (active=True) = localization オフ (enabled=False)。
        地図作成停止 (active=False) = localization オン (enabled=True、地図凍結・自己位置推定継続)。
        """
        err = self._set_localization(not active)
        if err:
            return err
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
        """ポーズグラフを path.posegraph / path.data へ書き出す。

        成否は応答ではなくファイルの実在で判定する。SerializePoseGraph.Response
        の result は slam_toolbox が早期 return したとき未設定のままとなり、
        0 (=RESULT_SUCCESS) に見えてしまうため信用できない。slam_control は
        slam_toolbox と同一コンテナで動くので、書けたかどうかは直接確認できる。
        """
        if not self._cli_serialize.wait_for_service(timeout_sec=1.0):
            return 'slam_toolbox に接続できません'
        os.makedirs(os.path.dirname(path), exist_ok=True)
        graph_path = path + '.posegraph'
        before = os.path.getmtime(graph_path) if os.path.exists(graph_path) else None

        req = SerializePoseGraph.Request()
        req.filename = path
        _, err = call_and_wait(self, self._cli_serialize, req, SERVICE_TIMEOUT_SEC)
        if err:
            return f'serialize_map 呼び出し失敗: {err}'

        if not os.path.exists(graph_path):
            return (f'ポーズグラフが書き出されませんでした ({graph_path} が無い)。'
                    ' slam_toolbox のログを確認してください')
        if before is not None and os.path.getmtime(graph_path) == before:
            return (f'ポーズグラフが更新されませんでした ({graph_path} の更新時刻が'
                    ' 変わっていない)。slam_toolbox のログを確認してください')
        return None

    # ── 地図の保存・再読込 (WS-9L /map_session/open) ─────────
    def _map_session_base(self, session_id: str) -> str:
        """session_id から保存/読込先ベース名（拡張子なし）を作る。

        route_record_core._safe_id（`/` `\\` → `_`）と同じ規則で無害化する
        （map_session_filename）。経路 JSON は最終的に _safe_id で保存されるため、
        ここも同じにしないと名前が食い違って読み直せない。
        """
        return os.path.join(self._map_dir, map_session_filename(session_id))

    def _cb_map_session_open(self, request, response):
        err = open_session_error(request.slot, request.mode, request.session_id)
        if err:
            return self._finish(response, err, '')

        base = self._map_session_base(request.session_id)
        with self._lock:
            if request.mode == 'save':
                return self._handle_map_save(response, base)
            return self._handle_map_reload(response, base, request)

    def _handle_map_save(self, response, base: str):
        """mapping モードのまま serialize_map で保存し、書き終えてから地図を凍結する。

        localization モードでは serialize が書き出さないので保存は必ず
        mapping モードで行う（2026-09-03 実機確認）。
        保存成功後に _set_localization(True) で凍結し、self._set_active(False) で
        状態を合わせる。凍結に失敗したらエラーを返す。
        """
        err = self._serialize(base)
        if err:
            return self._finish(response, err, '')
        err = self._set_localization(True)
        if err:
            return self._finish(response, f'地図の凍結に失敗: {err}', '')
        self._set_active(False)
        return self._finish(
            response, None, f'地図を保存しました（地図を凍結しました）: {base}.posegraph')

    def _kill_slam_toolbox(self):
        """稼働中の slam_toolbox プロセスを SIGTERM する。落とした PID の一覧を返す。

        launch の `respawn=True` が作り直す。`_cb_discard_map` と共用。
        """
        pids = _find_slam_toolbox_pids()
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
                self.get_logger().warn(
                    f'slam_toolbox (pid {pid}) を終了させました。'
                    ' respawn による再起動を待ちます')
            except OSError as e:
                self.get_logger().warn(f'slam_toolbox (pid {pid}) を終了できません: {e}')
        return pids

    def _wait_for_slam_restart(self, old_pids, timeout_s: float) -> "str | None":
        """WS-9S: SIGTERM 後、旧プロセス消失 → 新プロセスのサービス復帰まで待つ。

        戻り値: エラー文字列（タイムアウト等） or None（復帰完了）。
        """
        deadline = time.monotonic() + timeout_s
        # 1. slam_toolbox の PID が総入れ替わりするまで /proc をポーリングする
        #    （ROS グラフのタイミングに依存しない。純判定は slam_restart_complete）。
        while time.monotonic() < deadline:
            if slam_restart_complete(old_pids, _find_slam_toolbox_pids()):
                break
            time.sleep(0.5)
        else:
            return 'slam_toolbox が再起動しませんでした（respawn 待ちタイムアウト）'
        # 2. 新ノードのサービスが出そろうのを待つ。
        remaining = max(1.0, deadline - time.monotonic())
        if not self._cli_deserialize.wait_for_service(timeout_sec=remaining):
            return 'slam_toolbox の deserialize_map サービスが復帰しません'
        if not self._cli_localization.wait_for_service(timeout_sec=5.0):
            return 'slam_toolbox の set_localization_mode サービスが復帰しません'
        return None

    def _handle_map_reload(self, response, base: str, request=None):
        """slam_toolbox を作り直してから deserialize_map で地図を読み直す（WS-9S）。

        順序: (ファイル存在確認) → slam_toolbox を SIGTERM して respawn を待つ
              → _set_localization(True) → deserialize_map を **1 回だけ**

        WS-9S: 経路選択のたび長寿命ノードへ deserialize を反復するとポーズグラフが
        上乗せされ、`/map` が選択のたび形を変え `map→odom` が数 m 飛ぶ（2026-09-04
        実機で確定）。まっさらなノードに deserialize を 1 回、が本来の使い方。

        LocalizationSlamToolbox 側が match_type を扱うため、localization モードに
        入ってから読み直す（2026-09-03 実機確認。順序は AST テストで固定）。

        - req.filename = base（slam_toolbox が .posegraph を付けるため拡張子なし）
        - req.match_type = deserialize_match_type(request.has_initial_pose)
        - has_initial_pose なら req.initial_pose.x/.y/.theta に経路始点を設定
        - 再生中は localization モードのまま維持し、地図作成（mapping）には戻さない
          （戻すと地図が汚れ自己位置が発散する。ここが欠陥の中心）。
        - _set_active(False) で「地図作成停止」を publish する。
        """
        graph_path = base + '.posegraph'
        data_path = base + '.data'
        # kill する前にファイルの有無を確認して fail-fast する（無いのに slam を
        # 落とすと自己位置補正が無駄に止まる）。
        if not os.path.exists(graph_path):
            return self._finish(
                response, f'地図ファイルが無いため読み直せません ({graph_path})', '')
        if not os.path.exists(data_path):
            return self._finish(
                response, f'地図データファイルが無いため読み直せません ({data_path})', '')

        # kill の前に立てる。kill 直後に _check_slam_restart が「クラッシュした」と
        # 誤認するのを防ぐ（この関数は self._lock を持って呼ばれている）。
        self._reload_in_progress = True
        try:
            old_pids = self._kill_slam_toolbox()
            if not old_pids:
                self.get_logger().warn(
                    'slam_toolbox のプロセスが見つかりません。respawn 待ちに入ります'
                    '（サービス復帰を待って読み直しを試みます）')

            err = self._wait_for_slam_restart(old_pids, RESPAWN_WAIT_SEC)
            if err:
                return self._finish(response, err, '')

            # WS-9N: localization モードへ切り替えて地図を凍結する
            # (/slam_toolbox/set_localization_mode)
            err = self._set_localization(True)
            if err:
                return self._finish(response, f'localization モード切替失敗: {err}', '')

            req = DeserializePoseGraph.Request()
            req.filename = base   # WS-9M: slam_toolbox が `.posegraph` を付けるため拡張子なし
            has_init = bool(getattr(request, 'has_initial_pose', False)) if request else False
            req.match_type = deserialize_match_type(has_init)
            if has_init:
                req.initial_pose.x = float(request.initial_x)
                req.initial_pose.y = float(request.initial_y)
                req.initial_pose.theta = float(request.initial_yaw)

            _result, err = call_and_wait(
                self, self._cli_deserialize, req, DESERIALIZE_TIMEOUT_SEC)
            if err:
                return self._finish(response, f'deserialize_map 呼び出し失敗: {err}', '')

            # 再生中は localization モードのまま維持（地図作成に戻さない）
            self._set_active(False)
            # サービス復帰は確認済み。次周期の _check_slam_restart に
            # 「再起動を検知」させて余計なモード再適用を走らせない。
            self._slam_ready = True
        finally:
            self._reload_in_progress = False

        return self._finish(
            response, None,
            '地図を読み直しました（地図を凍結して自己位置推定に切り替えました）')

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
            if not _find_slam_toolbox_pids():
                return self._finish(
                    response, 'slam_toolbox のプロセスが見つかりません', '')
            # 破棄後は「地図作成停止」状態から始める（起動直後と同じ）。
            # 再起動した slam_toolbox は mapping モードで立ち上がるので、
            # _check_slam_restart がこの値を見て停止状態を入れ直す。
            self._set_active(False)
            # _handle_map_reload と違い、ここは respawn を待たない。再起動の検知と
            # モード再適用は _check_slam_restart() に任せる（従来どおり即返す）。
            self._kill_slam_toolbox()

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

        既定 (startup_mapping=false): 地図作成停止（localization モード）へ倒す。
        slam_toolbox は起動直後 mapping モード（=地図作成中）で立ち上がるため、
        ここで倒さないと「停止中」と表示したまま地図が更新され続ける。VISION.md
        §8 の「起動直後は地図作成を停止した状態にする」を満たすための処理。

        WS-8B (startup_mapping=true): 倒さず mapping のまま。教示・再生が
        map→odom の連続補正を必要とするため。ローカル状態は mapping 有効で始める。
        """
        if self._startup_mapping:
            self._set_active(True)
            self.get_logger().info(
                '初期状態: 地図作成 継続（WS-8B / startup_mapping=true。'
                '地図は再読込時に localization モードで凍結する）')
            self._slam_ready = self._cli_localization.service_is_ready()
            return

        if not self._cli_localization.wait_for_service(
                timeout_sec=STARTUP_SERVICE_TIMEOUT_SEC):
            self.get_logger().warn(
                'slam_toolbox 未起動のため初期化をスキップします '
                '(地図作成が停止されていない可能性があります。'
                'WebUI から明示的に開始/停止し直してください)')
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
    # WS-9U: call_and_wait はポーリング中に worker スレッドを塞ぐので下限 4 本。
    executor = MultiThreadedExecutor(num_threads=4)
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
