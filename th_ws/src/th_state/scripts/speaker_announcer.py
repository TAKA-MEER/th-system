#!/usr/bin/env python3
"""speaker_announcer.py — 機体スピーカーの報知判定ノード（Spec-safety.md §11）。

`/system/state`（モード・状態）と `/safety/limiter_status` の `approach_warning`
から「いま鳴らすもの」を決め、ラズパイ側の再生へ `std_msgs/String` の JSON で
周期配信する（ラズパイには th_system_msgs が無い前提。rpi_speaker_play.py 参照）。
判定は speaker_announce_core が持つ（純粋・ROS 非依存）。

トピック:
  購読 `/system/state`（発行側と同じ transient_local・reliable）
  購読 `/safety/limiter_status`（発行側と同じ best_effort。reliable にすると
    QoS 不一致で届かない）
  発行 `/speaker/cue`（transient_local・reliable・depth 1。2 Hz）
    {"cue": "none"|"drive"|"blocked"|"warning", "enabled": bool,
     "volume": 0〜100, "test": bool（試し鳴らし要求。1 回だけ true）,
     "rpi_ok": bool（再生側の応答があるか）}
  購読 `/speaker/status`（transient_local・reliable・depth 1。再生側の状態）
    {"cue": str, "volume": int, "error": str}

パラメータ（registry 由来の生成 yaml から渡す。R2）:
  speaker_enabled（bool）・speaker_volume（int 0〜100）
WebUI（S-50）から実行中に set_parameters で変えられる。変えた値は
`/root/th_data/speaker/settings.json` に残し、再起動後に読み戻す
（生成 yaml の値よりファイルが勝つ）。

報知は安全装置ではない。再生側の応答が無くても走行は止めない
（ログと cue の rpi_ok で分かるようにするだけ）。
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

from th_system_msgs.msg import LimiterStatus, SystemState

from th_state.speaker_announce_core import CUE_NONE, decide_cue

# 周期配信の間隔（brief の例 2 Hz）。再生側は 1 秒の途絶で止める。
PUBLISH_PERIOD_S = 0.5
# 再生側の状態がこの秒数届かなければ「スピーカー応答なし」。
STATUS_TIMEOUT_S = 5.0
# 「応答なし」のログはこの秒数ごとに 1 回（うるさくしない）。
NO_REPLY_LOG_PERIOD_S = 30.0
SETTINGS_NAME = 'settings.json'


class SpeakerAnnouncer(Node):

    def __init__(self, **kwargs):
        super().__init__('speaker_announcer', **kwargs)
        # R2: 調整値は既定値なしで宣言し、registry 由来の生成 yaml から必ず渡す
        self.declare_parameter('speaker_enabled', Parameter.Type.BOOL)
        self.declare_parameter('speaker_volume', Parameter.Type.INTEGER)
        # 試し鳴らしの要求（S-50 のボタンが true を書く。送ったらノードが戻す）
        self.declare_parameter('test_chime', False)
        # 置き場は調整値ではないので既定値を持つ（fault_recorder と同じ形）
        self.declare_parameter('speaker_settings_dir', '/root/th_data/speaker')

        self._settings_path = os.path.join(
            str(self.get_parameter('speaker_settings_dir').value), SETTINGS_NAME)
        self._apply_saved_settings()

        self._mode = None
        self._state = None
        self._approach_warning = False
        self._last_status_wall = 0.0
        self._test_pending = False
        self._last_no_reply_log = 0.0
        self._reply_logged = False
        self._write_warned = False

        self.add_on_set_parameters_callback(self._on_set_params)

        state_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                               history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(SystemState, '/system/state', self._on_state, state_qos)
        limiter_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(LimiterStatus, '/safety/limiter_status',
                                 self._on_limiter, limiter_qos)
        status_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(String, '/speaker/status', self._on_status, status_qos)
        cue_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(String, '/speaker/cue', cue_qos)
        self.create_timer(PUBLISH_PERIOD_S, self._on_timer)
        self.get_logger().info(
            f'speaker_announcer 起動: enabled={self._enabled()} '
            f'volume={self._volume()} settings={self._settings_path}')

    # -- パラメータ ------------------------------------------------------
    def _enabled(self) -> bool:
        return bool(self.get_parameter('speaker_enabled').value)

    def _volume(self) -> int:
        return int(self.get_parameter('speaker_volume').value)

    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        ok = True
        for p in params:
            if p.name == 'speaker_enabled':
                if p.type_ != Parameter.Type.BOOL:
                    ok = False
            elif p.name == 'speaker_volume':
                if (p.type_ != Parameter.Type.INTEGER or isinstance(p.value, bool)
                        or not 0 <= p.value <= 100):
                    ok = False
            elif p.name == 'test_chime':
                if p.type_ != Parameter.Type.BOOL:
                    ok = False
        result = SetParametersResult(successful=ok)
        if ok:
            names = {p.name for p in params}
            if 'test_chime' in names:
                self._test_pending = bool(
                    next(p.value for p in params if p.name == 'test_chime'))
            if {'speaker_enabled', 'speaker_volume'} & names:
                self._save_settings()
        return result

    # -- 永続化 ----------------------------------------------------------
    def _apply_saved_settings(self) -> None:
        try:
            with open(self._settings_path, encoding='utf-8') as f:
                saved = json.load(f)
        except (OSError, ValueError):
            return
        req = []
        if isinstance(saved.get('enabled'), bool):
            req.append(Parameter('speaker_enabled', Parameter.Type.BOOL,
                                 saved['enabled']))
        vol = saved.get('volume')
        if isinstance(vol, int) and not isinstance(vol, bool) and 0 <= vol <= 100:
            req.append(Parameter('speaker_volume', Parameter.Type.INTEGER, vol))
        if req:
            self.set_parameters(req)

    def _save_settings(self) -> None:
        try:
            os.makedirs(os.path.dirname(self._settings_path), exist_ok=True)
            with open(self._settings_path, 'w', encoding='utf-8') as f:
                json.dump({'enabled': self._enabled(), 'volume': self._volume()}, f)
        except OSError as e:
            if not self._write_warned:
                self._write_warned = True
                self.get_logger().warn(f'スピーカー設定を保存できない: {e}')

    # -- 購読 ------------------------------------------------------------
    def _on_state(self, msg: SystemState) -> None:
        self._mode = msg.mode
        self._state = msg.state

    def _on_limiter(self, msg: LimiterStatus) -> None:
        self._approach_warning = bool(msg.approach_warning)

    def _on_status(self, msg: String) -> None:
        self._last_status_wall = time.time()
        if not self._reply_logged:
            self._reply_logged = True
            self.get_logger().info('スピーカー応答あり')

    # -- 周期配信 --------------------------------------------------------
    def _on_timer(self) -> None:
        now = time.time()
        rpi_ok = (self._last_status_wall > 0.0
                  and now - self._last_status_wall < STATUS_TIMEOUT_S)
        if not rpi_ok and now - self._last_no_reply_log >= NO_REPLY_LOG_PERIOD_S:
            self._last_no_reply_log = now
            # 報知は安全装置ではない。走行は止めない（Spec-safety §11）。
            self.get_logger().warn('スピーカー応答なし（走行は止めない）')
        cue = decide_cue(self._mode, self._state, self._approach_warning)
        if not self._enabled():
            cue = CUE_NONE
        test = self._test_pending
        self._test_pending = False
        if test:
            self.set_parameters([Parameter('test_chime', Parameter.Type.BOOL, False)])
        self._pub.publish(String(data=json.dumps({
            'cue': cue, 'enabled': self._enabled(), 'volume': self._volume(),
            'test': test, 'rpi_ok': rpi_ok,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerAnnouncer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
