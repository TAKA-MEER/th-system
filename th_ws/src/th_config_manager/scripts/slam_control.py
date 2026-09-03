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
  | /slam_control/toggle_mapping   | Trigger        | pause_new_measurements（トグル）         |
  | /slam_control/set_mapping      | SetBool        | pause_new_measurements（明示指定）       |
  | /slam_control/save_map         | Trigger        | save_map + serialize_map                 |
  | /slam_control/discard_map      | Trigger        | slam_toolbox を終了 → respawn で再起動   |
  | /map_session/open              | OpenMapSession | save: pause→serialize / reload: deserialize→resume |

「地図作成停止」の意味（VISION.md §8）
--------------------------------------
停止は「地図の更新を止める」であって「自己位置推定を止める」ではない。
停止後も待機・呼び寄せ・配電盤移動を行う以上、map→odom は走行中ずっと
更新され続けなければならない。

**WS-9L（2026-09-03）: 切替手段を `pause_new_measurements` へ戻す。**
いま起動しているのは `async_slam_toolbox_node`（仰出堆 `bringup.launch.py`）
で、`map_and_localization_slam_toolbox_node` 専用の `set_localization_mode`
サービスを持たない（実機でサービス一覧を取って確認済み。以下「地図作成停止」
の実装は `pause_new_measurements` ベースに書き直した）。

`pause_new_measurements` は**引数を取らないトグル**で、応答の `status` は
常に True（＝呼べた）を返すだけで現在の一時停止状態は返さない（2026-09-03
実機で 3 回続けて呼んで 3 回とも status=True）。状態は `slam_control` 側が
`_paused_tracked` で追跡する（_set_pause を参照。純ロジックは
slam_control_logic.py の pause_toggle_needed）。

なお 2026-08-07 時点では `pause_new_measurements` は「スキャン処理そのものを
止める」ため map→odom が凍結して純粋なデッドレコニングになると確認し、この
経路を離れた。WS-9L では教示の地図を**凍結して保存 → 機体を始点へ戻す →
再読込**という構成に変え、再生は読み直した地図（自己位置＝地図の最初のノード）
の上で行う。地図作成の一時停止は「地図を凍結する」操作として使うので、
この制約とは矛盾しない。

地図の保存・再読込（WS-9L）————————
`/map_session/open` は route_recorder（教示の「保存」）と replay_runner（再生の
経路選択）が呼ぶ。`slot:"ROUTE"` / `session_id:<経路名>` / `mode:"save"|"reload"`。

  - save:   一時停止して `serialize_map` で `<map_dir>/<session_id>` へ
    （.posegraph 6.9MB + .data 29KB、実測）。成功しても一時停止のまま
    （教示後の地図を凍結した状態として保つ。再生の reload で再開する）。
  - reload: `deserialize_map` を `match_type: 1`(START_AT_FIRST_NODE) で呼ぶ。
    読み込み後 map→base_link ≒ 原点（= 経路の始点）になり、成功したら地図作成
    を再開（一時停止解除）する。ファイルが無ければ success=false。6.9MB の
   グラフ I/O が重いので専用の長めのタイムアウト（DESERIALIZE_TIMEOUT_SEC）を
    使う。

   2026-08-07 の SIGSEGV は「ほぼ空のポーズグラフを読み込んだ」ケース固有で、
   中身のあるグラフの読み込みは slam_toolbox の正規の使い方（2026-09-03 実測で
   SIGSEGV しないことを確認済み）。

地図の破棄について
------------------
slam_toolbox にポーズグラフを空へ戻すサービスは無い。このためプロセスごと
終了させ、`bringup.launch.py` の `respawn=True` で真っさらに立ち上げ直す。
`async_slam_toolbox_node` を kill する（SLAM_EXECUTABLE / _find_slam_toolbox_pids）。

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
DEFROST_SERVICE_TIMEOUT_SEC = 5.0   # deserialize 後の再開 (pause 解除) 用。普通の
                                     # トグルと同じなので SERVICE_TIMEOUT_SEC で良い
DESERIALIZE_TIMEOUT_SEC     = 30.0  # deserialize_map は 6.9MB のポーズグラフを
                                     # 読むのでディスク I/O が重い
                                     # (2026-09-03 実測 .posegraph 6.9MB + .data
                                     # 29KB)。SERVICE_TIMEOUT_SEC(5s) では足りない
                                     # 可能性があるため長めに取る
STATUS_PUBLISH_PERIOD_SEC   = 0.5

import os
import signal
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from slam_toolbox.srv import (DeserializePoseGraph, Pause, SaveMap,
                              SerializePoseGraph)

from th_system_msgs.msg import RobotMode
from th_system_msgs.srv import OpenMapSession

from th_config_manager.service_call import call_and_wait
from th_config_manager.slam_control_logic import (
    map_session_filename, open_session_error, pause_toggle_needed,
)


# 破棄で終了させる対象の実行ファイル名。bringup.launch.py が起動するものと
# 一致させること。pkill -f のようなパターン照合ではなく /proc を直接見るのは、
# パターンが自分自身のコマンドラインにマッチして自滅する事故を避けるため
# (CLAUDE.md「環境の癖」参照)。現在の教示・再生で使うのは
# async_slam_toolbox_node (WS-8B) なので、これに合わせること。
SLAM_EXECUTABLE = 'async_slam_toolbox_node'


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
        # WS-9M: pause_new_measurements の応答 status は常に True で現在状態を
        # 教えないため、トグルの呼び出し側が状態を追跡する。起動直後は
        # mapping モード（一時停止していない）。
        self._paused_tracked = False
        # WS-9L: /map_session/open の保存先ディレクトリ。経路 JSON と同じ場所
        # (routes_dir) でよい。serialize_map は <name>.posegraph / <name>.data を
        # 作るが、/route/catalog は .json しか拾わないため一覧は汚れない。
        self.declare_parameter('map_dir', '/root/th_data/routes')
        self._map_dir = self.get_parameter('map_dir').value
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

        # set_localization_mode は map_and_localization_slam_toolbox_node 専用で、
        # いま起動している async_slam_toolbox_node には存在しない (WS-9L)。
        # 存在しないサービスを wait し続けると起動が遅くなるので、クライアントは
        # 作らない。地図作成の開始/停止は pause_new_measurements で行う。
        self._cli_pause = self.create_client(
            Pause, '/slam_toolbox/pause_new_measurements', callback_group=cbg)
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
        ready = self._cli_pause.service_is_ready()
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
        # WS-9M: 再起動で slam_toolbox は mapping モード（一時停止していない）で
        # 立ち上がる。追跡状態をリセットしないと、実際に動いてるのに「停止中」と
        # 思い込んで逆に倒してしまう。
        self._paused_tracked = False
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

    def _set_pause(self, paused: bool) -> "str | None":
        """slam_toolbox の一時停止状態を切り替える。エラー文字列 or None を返す。

        `/slam_toolbox/pause_new_measurements` は**引数を取らないトグル**で、応答の
        `status` は常に True（＝呼べた）を返すだけで現在の状態を教えてくれない
        （2026-09-03 実機確認。3 回続けて呼んで 3 回とも status=True）。状態は
        `_paused_tracked` で追跡し、望みと違うときだけ **1 回だけ**トグルする。
        追跡状態はトグル成功時に反転させる。
        """
        if not pause_toggle_needed(self._paused_tracked, paused):
            return None
        if not self._cli_pause.wait_for_service(timeout_sec=1.0):
            return 'slam_toolbox に接続できません'
        _result, err = call_and_wait(
            self, self._cli_pause, Pause.Request(), SERVICE_TIMEOUT_SEC)
        if err:
            return f'pause_new_measurements 呼び出し失敗: {err}'
        self._paused_tracked = not self._paused_tracked
        return None

    def _apply_mapping(self, active: bool) -> "str | None":
        """マッピングの有効/無効を切り替える。エラー文字列 or None を返す。

        地図作成の一時停止 = 地図の凍結。active=True（地図作成中）は一時停止を
        解除、active=False（地図作成停止）は一時停止に倒す。
        """
        err = self._set_pause(not active)
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
            return self._handle_map_reload(response, base)

    def _handle_map_save(self, response, base: str):
        """地図を凍結（一時停止）して serialize_map で保存する。

        成功しても一時停止のままにする（教示後の地図を凍結した状態として保つ。
        再生の reload で再開される）。route_recorder はこの後 failure でも経路保存は
        成功扱いのままにすること（失敗時はエラー文字列を返す）。
        """
        err = self._set_pause(True)
        if err:
            return self._finish(response, err, '')
        err = self._serialize(base)
        if err:
            return self._finish(response, err, '')
        return self._finish(
            response, None, f'地図を保存しました（一時停止中に凍結）: {base}.posegraph')

    def _handle_map_reload(self, response, base: str):
        """deserialize_map で地図を読み直し、成功したら地図作成を再開する。

        match_type 1 = START_AT_FIRST_NODE（地図の最初のノード＝経路の始点に自己位置を
        置く）。読み込み後 map→base_link ≒ 原点（実測 (-0.021, 0.000)）。
        6.9MB のグラフ I/O が重いので DESERIALIZE_TIMEOUT_SEC(30s) を使う。

        WS-9M: slam_toolbox は渡された filename に `.posegraph` を自分で付ける。
        拡張子つきの名前を渡すと `<base>.posegraph.posegraph` を探して必ず失敗する
        （実機ログ:
          serialization::Read: Failed to open requested file: .../crash_check.posegraph.
          DeserializePoseGraph: Failed to read file: .../crash_check.posegraph.）。
        そこでサービスに渡す名前は拡張子なしの `base`、存在確認はディスク上の
        実ファイル名 `base + '.posegraph'` / `base + '.data'` と、あえて別にする。
        """
        if not self._cli_deserialize.wait_for_service(timeout_sec=1.0):
            return self._finish(response, 'slam_toolbox に接続できません', '')
        # WS-9M: deserialize を呼ぶ前に `.posegraph` と `.data` の両方の存在を
        # 確認する。DeserializePoseGraph の応答は空でフィールドが 1 つも無く、
        # 読み込みが失敗しても call_and_wait は成功として返る（＝サービスの
        # 戻り値では成否が分からない）。片方でも欠けていれば deserialize は必ず
        # 失敗するので、呼ばずに success=false を返して replay_runner が
        # localize_done → READY まで進まないようにする。
        # 確認するのはディスク上の実ファイル名（拡張子つき。サービスに渡す名前は
        # 上の docstring のとおり拡張子なし）。
        graph_path = base + '.posegraph'
        data_path = base + '.data'
        if not os.path.exists(graph_path):
            return self._finish(
                response, f'地図ファイルが無いため読み直せません ({graph_path})', '')
        if not os.path.exists(data_path):
            return self._finish(
                response, f'地図データファイルが無いため読み直せません ({data_path})', '')
        req = DeserializePoseGraph.Request()
        req.filename = base   # WS-9M: slam_toolbox が `.posegraph` を付けるため拡張子なし
        req.match_type = 1   # START_AT_FIRST_NODE
        _result, err = call_and_wait(
            self, self._cli_deserialize, req, DESERIALIZE_TIMEOUT_SEC)
        if err:
            return self._finish(response, f'deserialize_map 呼び出し失敗: {err}', '')

        # 成功したら地図作成を再開（一時停止解除）。resume 側は普通のトグルなので
        # DEFROST_SERVICE_TIMEOUT_SEC で足りる。
        err = self._set_pause(False)
        if err:
            return self._finish(response, err, '')
        return self._finish(
            response, None, '地図を読み直しました（自己位置を経路の始点に合わせました）')

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

        既定 (startup_mapping=false): 地図作成停止（一時停止）へ倒す。
        slam_toolbox は起動直後 mapping モード（=地図作成中）で立ち上がるため、
        ここで倒さないと「停止中」と表示したまま地図が更新され続ける。VISION.md
        §8 の「起動直後は地図作成を停止した状態にする」を満たすための処理。

        WS-8B (startup_mapping=true): 倒さず mapping のまま。教示・再生が
        map→odom の連続補正を必要とするため。ローカル状態は mapping 有効で始める。
        この経路は async_slam_toolbox_node（pause_new_measurements は持つ）で使う。
        """
        if self._startup_mapping:
            self._set_active(True)
            self.get_logger().info(
                '初期状態: 地図作成 継続（WS-8B / startup_mapping=true。'
                '地図は再読込時に一時停止で凍結する）')
            self._slam_ready = self._cli_pause.service_is_ready()
            return

        if not self._cli_pause.wait_for_service(
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
