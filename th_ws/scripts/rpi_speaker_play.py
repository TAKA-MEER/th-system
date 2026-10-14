#!/usr/bin/env python3
"""rpi_speaker_play.py — ラズパイ側の再生ノード（Spec-safety.md §11）。

ラズパイ（mirs2602@192.168.5.1）に置いて systemd（rpi_speaker.service）で動かす。
PC の th_ws に依存しない。import するのは標準ライブラリ＋rclpy＋std_msgs だけ
（th_system_msgs はラズパイに無い）。判定の純粋部は rpi_speaker_logic.py。

  購読 /speaker/cue（std_msgs/String の JSON。transient_local・2 Hz）
  発行 /speaker/status（std_msgs/String の JSON。transient_local・1 Hz）

音の出し方（--backend で選ぶ。既定 pulse）:
  pulse … paplay で鳴らし、音量は pactl set-sink-volume（--sink。省略時は既定）
  alsa  … aplay で鳴らし、音量は amixer -c <card> set <control> N%
切り替えは即時、none・無効・配信途絶（1 秒）で停止。ループ再生は 1 回の再生が
終わるたびに止める条件を見直す。試し鳴らし（test=true の立ち上がり）は
drive.wav を 1 回だけ鳴らす。

使い方（ラズパイ）:
  /usr/bin/python3 /home/mirs2602/rpi_speaker_play.py --wav-dir /home/mirs2602/speaker_wavs
  /usr/bin/python3 /home/mirs2602/rpi_speaker_play.py --backend alsa --wav-dir ...
導入手順は docs/network.md「ラズパイ: スピーカー再生の導入」参照。
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rpi_speaker_logic import (CUE_DRIVE, CUE_WAVS, make_status,  # noqa: E402
                               parse_cue, resolve_playback)

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,  # noqa: E402
                       QoSReliabilityPolicy)
from std_msgs.msg import String  # noqa: E402


def _latch1():
    return QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.KEEP_LAST)


class Player:
    """wav のループ再生。再生と停止だけ持つ（どれを鳴らすかは呼び出し側）。"""

    def __init__(self, backend: str, sink: str, card: str, control: str, wav_dir: str):
        self._backend = backend
        self._sink = sink
        self._card = card
        self._control = control
        self._wav_dir = wav_dir
        self._lock = threading.Lock()
        self._want = None  # 鳴らしたい cue（None＝停止）
        self._playing = None
        self._volume = -1
        self._error = ''
        self._proc = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    @property
    def error(self):
        with self._lock:
            return self._error

    @property
    def playing(self):
        with self._lock:
            return self._playing

    def start(self):
        self._check_tools()
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._terminate()
        self._thread.join(timeout=5.0)

    def set_want(self, cue_or_none, volume: int):
        with self._lock:
            self._want = cue_or_none
            if self._volume != volume:
                self._volume = volume
                self._apply_volume_locked()

    def play_once(self, cue: str, volume: int):
        """試し鳴らし。ループを邪魔せず 1 回だけ鳴らす。"""
        path = os.path.join(self._wav_dir, CUE_WAVS[cue])
        try:
            subprocess.run(self._play_cmd(path), timeout=10.0,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.SubprocessError):
            pass

    # -- 内部 ----------------------------------------------------------
    def _check_tools(self):
        play = 'paplay' if self._backend == 'pulse' else 'aplay'
        if shutil.which(play) is None:
            with self._lock:
                self._error = f'{play} が無い'
        if self._backend == 'pulse':
            if shutil.which('pactl') is None:
                with self._lock:
                    self._error = self._error or 'pactl が無い（音量は変えられない）'
        elif shutil.which('amixer') is None:
            with self._lock:
                self._error = self._error or 'amixer が無い（音量は変えられない）'

    def _play_cmd(self, path):
        if self._backend == 'pulse':
            return ['paplay', path]
        return ['aplay', '-q', path]

    def _apply_volume_locked(self):
        volume = self._volume
        try:
            if self._backend == 'pulse':
                if shutil.which('pactl') is None:
                    return
                target = self._sink or '@DEFAULT_SINK@'
                subprocess.run(['pactl', 'set-sink-volume', target, f'{volume}%'],
                               timeout=5.0, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
            else:
                if shutil.which('amixer') is None:
                    return
                subprocess.run(['amixer', '-c', self._card, 'set', self._control,
                                f'{volume}%'],
                               timeout=5.0, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        except (OSError, subprocess.SubprocessError):
            pass

    def _terminate(self):
        with self._lock:
            proc = self._proc
            self._proc = None
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _loop(self):
        while not self._stop.is_set():
            with self._lock:
                want = self._want
            if want is None:
                self._terminate()
                with self._lock:
                    self._playing = None
                self._stop.wait(0.1)
                continue
            path = os.path.join(self._wav_dir, CUE_WAVS[want])
            if not os.path.isfile(path):
                with self._lock:
                    self._error = f'{path} が無い'
                    self._playing = None
                self._stop.wait(0.5)
                continue
            with self._lock:
                self._playing = want
                self._error = ''
            try:
                proc = subprocess.Popen(
                    self._play_cmd(path), stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
            except OSError as e:
                with self._lock:
                    self._error = str(e)
                    self._playing = None
                self._stop.wait(0.5)
                continue
            with self._lock:
                self._proc = proc
            while proc.poll() is None and not self._stop.is_set():
                with self._lock:
                    changed = self._want != want
                if changed:
                    break
                time.sleep(0.05)
            if proc.poll() is None:
                # 切り替え・停止のときだけ止める。鳴り終わりはループして繰り返す。
                with self._lock:
                    changed = self._want != want
                if changed or self._want is None:
                    self._terminate()


class SpeakerPlayer(Node):
    def __init__(self, player: Player):
        super().__init__('speaker_player')
        self._player = player
        self._cue_msg = None
        self._last_recv = None
        self._last_test = False
        self._volume = 60
        self.create_subscription(String, '/speaker/cue', self._on_cue, _latch1())
        self._pub = self.create_publisher(String, '/speaker/status', _latch1())
        self.create_timer(1.0, self._on_timer)

    def _on_cue(self, msg: String):
        parsed = parse_cue(msg.data)
        now = time.time()
        if parsed is not None:
            self._cue_msg = parsed
            self._last_recv = now
            self._volume = parsed['volume']
            if parsed['test'] and not self._last_test:
                self._player.play_once(CUE_DRIVE, parsed['volume'])
            self._last_test = parsed['test']

    def _on_timer(self):
        now = time.time()
        age = None if self._last_recv is None else now - self._last_recv
        want = resolve_playback(self._cue_msg, age)
        self._player.set_want(want, self._volume)
        self._pub.publish(String(data=make_status(
            self._player.playing, self._volume, self._player.error)))


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', choices=('pulse', 'alsa'), default='pulse')
    ap.add_argument('--sink', default='',
                    help='PulseAudio のシンク名（省略時は既定。実機は '
                         'alsa_output.platform-bcm2835_audio.stereo-fallback）')
    ap.add_argument('--card', default='0', help='ALSA のカード番号')
    ap.add_argument('--control', default='Headphone',
                    help='ALSA のコントロール名（実機の amixer -c 0 scontents の名前）')
    ap.add_argument('--wav-dir', required=True)
    ns, ros_args = ap.parse_known_args(args)
    rclpy.init(args=ros_args)
    player = Player(ns.backend, ns.sink or None, ns.card, ns.control, ns.wav_dir)
    node = SpeakerPlayer(player)
    player.start()
    node.get_logger().info(
        f'speaker_player 起動: backend={ns.backend} wav_dir={ns.wav_dir}')

    def _sig(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT では spin が ExternalShutdownException を投げることがある
        # （実機で確認。rclpy 側は既に shutdown 済みなので下の ok() が弾く）。
        pass
    finally:
        player.stop()
        node.destroy_node()
        # SIGINT では rclpy 側のハンドラが先に shutdown している。
        # 二重に呼ぶと RCLError（rcl_shutdown already called）になるので、
        # context が生きているときだけ shutdown する。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
