"""test_onsite_nodes_launched.py — th_onsite の実行ファイルが bringup で実際に起動されることを固定する。

実機で踏んだ事故（2026-09-08）:
    S-21 の「待機場所を宣言」が必ず失敗する。原因は `home_declarer.py` が
    `CMakeLists.txt` の `install(PROGRAMS ...)` には載っていて
    `ros2 pkg executables th_onsite` にも出るのに、
    **`bringup.launch.py` にノードが書かれていなかった**こと。
    稼働中の実機で `ros2 service list` に `/onsite/declare_home` が無いことで確定した。

`test_installed_scripts_executable.py` は「install に載っているか・起動できる形か」を守るが、
**「launch から実際に起動されるか」は誰も見ていなかった**。この穴を塞ぐ。

ROS2 環境は不要（ファイルを読むだけ）。ホストの素の pytest で走る。
"""
import os
import re

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, '..', '..'))
_ONSITE_CMAKE = os.path.join(_SRC, 'th_onsite', 'CMakeLists.txt')
_BRINGUP_LAUNCH = os.path.join(_SRC, 'th_bringup', 'launch', 'bringup.launch.py')


def _cmake_installed_programs():
    """th_onsite の install(PROGRAMS ...) に載っている scripts/*.py のファイル名。"""
    text = open(_ONSITE_CMAKE, encoding='utf-8').read()
    block = re.search(r'install\(PROGRAMS(.*?)\)', text, re.S)
    assert block, 'th_onsite/CMakeLists.txt に install(PROGRAMS ...) が無い'
    return sorted(os.path.basename(p) for p in re.findall(r'scripts/(\S+\.py)', block.group(1)))


def _launch_text():
    return open(_BRINGUP_LAUNCH, encoding='utf-8').read()


def test_every_onsite_executable_is_launched():
    """install に載せた th_onsite の実行ファイルは全部 bringup.launch.py に出てくる。

    「ビルドされるのに誰も起動しない」成果物を作らないための不変条件。
    """
    text = _launch_text()
    missing = [name for name in _cmake_installed_programs()
               if f"executable='{name}'" not in text]
    assert not missing, (
        f'th_onsite の実行ファイルが bringup.launch.py に無い: {missing}. '
        'install(PROGRAMS) に載せただけでは実機で起動しない（2026-09-08 の home_declarer）')


def test_onsite_nodes_are_gated_by_onsite_enabled():
    """th_onsite のノードは全部 onsite_enabled 条件つきで起動する。

    段階 1/2（Nav2 も人物トラッカーも無い）で立てても何もできないので、
    条件を付け忘れると起動ログにノイズが増えるだけになる。
    """
    text = _launch_text()
    assert "onsite_enabled" in text, 'bringup.launch.py に onsite_enabled が無い'
    for name in _cmake_installed_programs():
        idx = text.find(f"executable='{name}'")
        assert idx >= 0, f'{name} が bringup.launch.py に無い'
        # そのノードの Node(...) ブロック（次の "))" まで）に condition があること。
        end = text.find('))', idx)
        block = text[idx:end]
        assert 'condition=IfCondition(onsite_enabled)' in block, (
            f'{name} の Node に condition=IfCondition(onsite_enabled) が無い')


def test_onsite_package_is_declared_in_bringup_deps():
    """th_bringup が th_onsite に依存宣言していること（起動時に見つからない事故を防ぐ）。"""
    pkg_xml = os.path.join(_SRC, 'th_bringup', 'package.xml')
    text = open(pkg_xml, encoding='utf-8').read()
    assert 'th_onsite' in text, (
        'th_bringup/package.xml が th_onsite を宣言していない。'
        'bringup が th_onsite のノードを起動するなら依存に載せる')
