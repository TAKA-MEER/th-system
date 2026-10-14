"""test_speaker_announcer_node.py — speaker_announcer を本物で起動して縛る
（Spec-safety §11）。

/system/state と /safety/limiter_status の代役を、実機の発行側と同じ QoS で
流し、/speaker/cue（std_msgs/String の JSON）の cue が変わることを見る。
パラメータを変えると配信の volume が変わり、ファイルに残ることも縛る。
rclpy 要＝ホスト除外（CLAUDE.md の除外一覧）。
"""

import json
import os
import tempfile
import time
import unittest

import pytest
import rclpy

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String
from th_system_msgs.msg import LimiterStatus, SystemState

_DIR = tempfile.mkdtemp(prefix='speaker_test_')


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='th_state', executable='speaker_announcer.py', name='speaker_announcer',
        parameters=[{'speaker_enabled': True, 'speaker_volume': 60,
                     'speaker_settings_dir': _DIR}],
        output='screen')
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'node': node}


_LATCH1 = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)
_STATE_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        history=QoSHistoryPolicy.KEEP_LAST)
_BE1 = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                  history=QoSHistoryPolicy.KEEP_LAST)


class TestSpeakerAnnouncer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_speaker_announcer')
        cls.cues = []
        cls.node.create_subscription(String, '/speaker/cue', cls.cues.append, _LATCH1)
        cls.p_state = cls.node.create_publisher(SystemState, '/system/state', _STATE_QOS)
        cls.p_limiter = cls.node.create_publisher(LimiterStatus, '/safety/limiter_status', _BE1)
        cls.p_status = cls.node.create_publisher(String, '/speaker/status', _LATCH1)
        cls.cli_set = cls.node.create_client(SetParameters, '/speaker_announcer/set_parameters')
        cls.cli_get = cls.node.create_client(GetParameters, '/speaker_announcer/get_parameters')
        assert cls.cli_set.wait_for_service(timeout_sec=30.0), 'speaker_announcer 未起動'
        assert cls.cli_get.wait_for_service(timeout_sec=30.0), 'speaker_announcer 未起動'
        end = time.time() + 20
        while time.time() < end and not cls.cues:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        assert cls.cues, '/speaker/cue が届かない'
        time.sleep(1.0)  # 購読側 discovery
        cls._send_state('IDLE', 'NONE', False)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _send_state(cls, mode, state, warning):
        msg = SystemState()
        msg.mode = mode
        msg.state = state
        cls.p_state.publish(msg)
        lim = LimiterStatus()
        lim.approach_warning = warning
        cls.p_limiter.publish(lim)

    @classmethod
    def _last_cue(cls):
        end = time.time() + 10
        while time.time() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if cls.cues:
                try:
                    return json.loads(cls.cues[-1].data)
                except ValueError:
                    pass
        raise AssertionError('/speaker/cue の JSON が読めない')

    def _wait_cue(self, want, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.cues and json.loads(self.cues[-1].data).get('cue') == want:
                return json.loads(self.cues[-1].data)
        raise AssertionError(f'cue={want} にならない（最後: {self.cues[-1].data if self.cues else None}）')

    def _set_param(self, name, ptype, value):
        req = SetParameters.Request()
        p = Parameter(name=name, value=ParameterValue(type=ptype))
        if ptype == Parameter.Type.BOOL:
            p.value.bool_value = value
        else:
            p.value.integer_value = value
        req.parameters = [p]
        fut = self.cli_set.call_async(req)
        end = time.time() + 10
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        assert fut.done() and fut.result().results[0].successful, f'{name} の set が失敗'

    def test_10_initial_is_none_and_no_reply(self):
        self._send_state('IDLE', 'NONE', False)
        cue = self._wait_cue('none')
        assert cue['enabled'] is True and cue['volume'] == 60
        assert cue['rpi_ok'] is False  # 再生側がまだ無い

    def test_20_drive_blocked_warning_priority(self):
        self._send_state('REPLAY', 'RUN', False)
        assert self._wait_cue('drive')['cue'] == 'drive'
        self._send_state('SUMMON', 'BLOCKED', False)
        assert self._wait_cue('blocked')['cue'] == 'blocked'
        self._send_state('SUMMON', 'BLOCKED', True)
        assert self._wait_cue('warning')['cue'] == 'warning'
        self._send_state('REPLAY', 'PAUSE', False)
        assert self._wait_cue('none')['cue'] == 'none'

    def test_30_status_reply_marks_rpi_ok(self):
        self.p_status.publish(String(data=json.dumps({'cue': 'drive', 'volume': 60, 'error': ''})))
        end = time.time() + 10
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.cues and json.loads(self.cues[-1].data).get('rpi_ok') is True:
                return
        raise AssertionError('再生側の状態を出しても rpi_ok にならない')

    def test_40_volume_change_applies_and_persists(self):
        self._send_state('REPLAY', 'RUN', False)
        self._set_param('speaker_volume', Parameter.Type.INTEGER, 80)
        end = time.time() + 10
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.cues and json.loads(self.cues[-1].data).get('volume') == 80:
                break
        else:
            raise AssertionError('volume を変えても配信に反映されない')
        with open(os.path.join(_DIR, 'settings.json'), encoding='utf-8') as f:
            saved = json.load(f)
        assert saved == {'enabled': True, 'volume': 80}, saved
        self._set_param('speaker_volume', Parameter.Type.INTEGER, 60)

    def test_50_test_chime_sends_once_and_resets(self):
        self._set_param('test_chime', Parameter.Type.BOOL, True)
        end = time.time() + 10
        seen = False
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.cues and json.loads(self.cues[-1].data).get('test') is True:
                seen = True
                break
        assert seen, '試し鳴らし要求が /speaker/cue に載らない'
        req = GetParameters.Request(names=['test_chime'])
        fut = self.cli_get.call_async(req)
        end = time.time() + 10
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        assert fut.done() and fut.result().values[0].bool_value is False, \
            'test_chime が false に戻らない'

    def test_60_invalid_volume_is_rejected(self):
        self._set_param('speaker_volume', Parameter.Type.INTEGER, 80)
        req = SetParameters.Request()
        p = Parameter(name='speaker_volume',
                      value=ParameterValue(type=Parameter.Type.INTEGER, integer_value=200))
        req.parameters = [p]
        fut = self.cli_set.call_async(req)
        end = time.time() + 10
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        assert fut.done() and fut.result().results[0].successful is False, \
            '範囲外の volume が受理されてはいけない'
        self._set_param('speaker_volume', Parameter.Type.INTEGER, 60)
