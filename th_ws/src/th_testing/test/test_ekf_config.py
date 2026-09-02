"""test_ekf_config.py — ekf_params*.yaml の整合性検査。

2026-09-02 実機調査で見つかった不具合の回帰試験:

  `ekf_params.yaml` は `imu0_config` / `imu0_differential` / `imu0_queue_size` …
  と IMU 用の設定を一通り持っていたのに、**センサを登録する `imu0:` の 1 行だけが
  欠落していた**。robot_localization は `imu0: <topic>` が無いとそのセンサを
  そもそも登録しないため、`imu_enabled:=true` で起動しても**ジャイロは一切
  融合されず、無言で odom0 だけの EKF になる**。エラーも警告も出ない。

  デッドレコニングのヨードリフトを抑える唯一の手段がジャイロなので、
  校舎 1 周（100m 以上）の教示・再生ではこの欠落が効く。

一般化して「`<sensor>N_*` の設定があるのに `<sensor>N:` が無い」を検出する。
ROS2 を必要としない純粋な YAML 検査。
"""
import re
from pathlib import Path

import pytest
import yaml

_CONFIG_DIR = Path(__file__).resolve().parents[2] / 'th_bringup' / 'config'
_EKF_FILES = sorted(_CONFIG_DIR.glob('ekf_params*.yaml'))

# robot_localization がセンサとして受け付ける接頭辞。
_SENSOR_PREFIXES = ('odom', 'imu', 'pose', 'twist')
_SUFFIXED = re.compile(r'^(?P<base>(?:odom|imu|pose|twist)\d+)_[a-z_]+$')


def _ekf_params(path: Path) -> dict:
    doc = yaml.safe_load(path.read_text(encoding='utf-8'))
    node = doc['ekf_filter_node']
    return node['ros__parameters']


def test_ekf_config_files_exist():
    assert _EKF_FILES, f'ekf_params*.yaml が見つからない: {_CONFIG_DIR}'


@pytest.mark.parametrize('path', _EKF_FILES, ids=lambda p: p.name)
def test_every_configured_sensor_has_its_topic_line(path):
    """`imu0_config` 等があるのに `imu0:` が無い、を検出する。

    これが今回踏んだ不具合そのもの。設定だけあってセンサ登録が無いと
    robot_localization は黙ってそのセンサを無視する。
    """
    params = _ekf_params(path)
    declared = {k for k in params if k in
                {f'{p}{i}' for p in _SENSOR_PREFIXES for i in range(8)}}
    referenced = set()
    for key in params:
        m = _SUFFIXED.match(key)
        if m:
            referenced.add(m.group('base'))
    orphans = sorted(referenced - declared)
    assert not orphans, (
        f'{path.name}: {orphans} の設定行はあるのに、センサを登録する '
        f'"{orphans[0]}: <topic>" が無い。robot_localization はこのセンサを'
        f'無言で無視するため、設定が効かない')


@pytest.mark.parametrize('path', _EKF_FILES, ids=lambda p: p.name)
def test_sensor_topics_are_absolute_names(path):
    """トピックは絶対名で書く（名前空間で意図せず解決されないように）。"""
    params = _ekf_params(path)
    for key, val in params.items():
        if key in {f'{p}{i}' for p in _SENSOR_PREFIXES for i in range(8)}:
            assert isinstance(val, str) and val.startswith('/'), \
                f'{path.name}: {key} が絶対トピック名でない: {val!r}'


def test_imu_variant_actually_fuses_the_gyro():
    """IMU あり版は /esp32/imu_data の vyaw を融合する構成であること。

    CLAUDE.md の不変ルール:「EKF が融合する IMU 入力はジャイロの vyaw のみ」。
    BNO055 は NDOF で絶対方位を返すため、屋内の磁気擾乱でヨーが飛ぶ。
    world_frame: odom の EKF に絶対方位を入れてはいけない。
    """
    path = _CONFIG_DIR / 'ekf_params.yaml'
    params = _ekf_params(path)
    assert params.get('imu0') == '/esp32/imu_data', \
        'imu0 が /esp32/imu_data を指していない（esp32_bridge の publish 先）'
    cfg = params['imu0_config']
    assert len(cfg) == 15 or len(cfg) == 9, f'imu0_config の長さが異常: {len(cfg)}'
    # [roll, pitch, yaw, vroll, vpitch, vyaw, ax, ay, az]
    assert cfg[5] is True, 'vyaw（ジャイロ角速度）が融合対象になっていない'
    assert cfg[2] is False, 'yaw（絶対方位）を融合してはいけない（屋内の磁気擾乱）'
    assert params.get('world_frame') == 'odom'


def test_no_imu_variant_has_no_imu_sensor():
    """IMU なし版には IMU センサが登録されていないこと（取り違え防止）。"""
    path = _CONFIG_DIR / 'ekf_params_no_imu.yaml'
    if not path.exists():
        pytest.skip('ekf_params_no_imu.yaml が無い')
    params = _ekf_params(path)
    assert 'imu0' not in params, 'IMU なし版に imu0 が入っている'
