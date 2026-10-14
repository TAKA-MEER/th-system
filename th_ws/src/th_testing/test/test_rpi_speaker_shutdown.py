"""rpi_speaker_play.py の終了処理の試験（ROS 不要の素の pytest）。

実機で出た不具合: SIGINT で止めると rclpy 側のハンドラが先に shutdown し、
finally の shutdown が二重になって RCLError（rcl_shutdown already called）の
Traceback が出る。context が生きているときだけ shutdown することを縛る。

rclpy はホストに無いので、最小のスタブを sys.modules に入れて main() の
finally 経路だけを通す（paplay は呼ばない。スレッドは daemon で lon に止める）。
"""
import sys
import types
from pathlib import Path

SCRIPTS = str(Path(__file__).resolve().parents[3] / "scripts")
sys.path.insert(0, SCRIPTS)


def _install_stubs(spin_behavior):
    calls = {'shutdown': 0}

    rclpy = types.ModuleType('rclpy')
    rclpy._ok = True

    def _init(args=None):
        rclpy._ok = True

    def _spin(node):
        if spin_behavior == 'sigint_after_auto_shutdown':
            # 実機の SIGINT: rclpy 側が先に shutdown してから KeyboardInterrupt
            rclpy._ok = False
            raise KeyboardInterrupt
        if spin_behavior == 'external_shutdown':
            # 実機の SIGINT: spin が ExternalShutdownException を投げることもある
            rclpy._ok = False
            raise exec_mod.ExternalShutdownException('external shutdown')
        return None

    def _shutdown():
        if not rclpy._ok:
            raise RuntimeError('rcl_shutdown already called')
        calls['shutdown'] += 1
        rclpy._ok = False

    rclpy.init = _init
    rclpy.spin = _spin
    rclpy.shutdown = _shutdown
    rclpy.ok = lambda: rclpy._ok

    node_mod = types.ModuleType('rclpy.node')

    class _Logger:
        def info(self, *a, **k):
            pass

    class Node:
        def __init__(self, *a, **k):
            pass

        def create_subscription(self, *a, **k):
            return object()

        def create_publisher(self, *a, **k):
            class _Pub:
                def publish(self, msg):
                    pass
            return _Pub()

        def create_timer(self, *a, **k):
            return object()

        def get_logger(self):
            return _Logger()

        def destroy_node(self):
            pass

    node_mod.Node = Node

    qos_mod = types.ModuleType('rclpy.qos')

    class _QoSProfile:
        def __init__(self, *a, **k):
            pass

    qos_mod.QoSProfile = _QoSProfile
    qos_mod.QoSDurabilityPolicy = type('D', (), {'TRANSIENT_LOCAL': 1})
    qos_mod.QoSHistoryPolicy = type('H', (), {'KEEP_LAST': 1})
    qos_mod.QoSReliabilityPolicy = type('R', (), {'RELIABLE': 1})

    std_mod = types.ModuleType('std_msgs')
    msg_mod = types.ModuleType('std_msgs.msg')
    msg_mod.String = type('String', (), {'__init__': lambda self, **k: None})

    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_mod
    sys.modules['rclpy.qos'] = qos_mod
    sys.modules['std_msgs'] = std_mod
    sys.modules['std_msgs.msg'] = msg_mod
    exec_mod = types.ModuleType('rclpy.executors')

    class ExternalShutdownException(Exception):
        pass

    exec_mod.ExternalShutdownException = ExternalShutdownException
    sys.modules['rclpy.executors'] = exec_mod
    for m in list(sys.modules):
        if m == 'rpi_speaker_play' or m.startswith('rpi_speaker_play.'):
            del sys.modules[m]
    return calls


def _run_main(spin_behavior):
    calls = _install_stubs(spin_behavior)
    import rpi_speaker_play
    rpi_speaker_play.main(['--wav-dir', '/tmp/no_such_dir'])
    return calls


def test_sigint_does_not_shutdown_twice():
    # 二重 shutdown ならスタブが RuntimeError（実機の RCLError 相当）を投げる。
    # 投げずに抜ければ直っている。
    calls = _run_main('sigint_after_auto_shutdown')
    assert calls['shutdown'] == 0


def test_normal_exit_shuts_down_once():
    calls = _run_main('return')
    assert calls['shutdown'] == 1


def test_external_shutdown_exception_exits_cleanly():
    # 実機の SIGINT（spin が ExternalShutdownException）。Traceback を出さず、
    # 二重 shutdown もしない。
    calls = _run_main('external_shutdown')
    assert calls['shutdown'] == 0


def test_alsa_control_default_matches_hardware():
    # 実機の amixer -c 0 scontents の名前は Headphone（s 無し）。
    import inspect
    _install_stubs('return')
    for m in list(sys.modules):
        if m == 'rpi_speaker_play' or m.startswith('rpi_speaker_play.'):
            del sys.modules[m]
    import rpi_speaker_play
    src = inspect.getsource(rpi_speaker_play.main)
    assert "default='Headphone'" in src and "default='Headphones'" not in src
