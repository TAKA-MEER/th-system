"""test_gazebo_measure_launch.py — gazebo.launch.py が自律測定の走行の受け手を持つことを固定する。

8-g P6（DetailedDesign-measure.md §6.7）: 測定機能が本物の Gazebo 一式に繋いで
自律測定を回したとき 2 か所で止まった。1 つ目は `gazebo.launch.py` が
`venue_navigator` を起動しておらず、`state_manager` が `meas.goal` を
`venue_navigator` 宛ての effect で配る受け手が居なかったこと。

「install に載っているか」だけでは「launch から実際に起動されるか」は
分からない（test_onsite_nodes_launched.py と同じ穴）。ROS2 環境は不要で、
launch ファイルの記述だけを読む（ホストの素の pytest で走る）。
"""
import os
import re

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, '..', '..'))
_GAZEBO_LAUNCH = os.path.join(_SRC, 'th_bringup', 'launch', 'gazebo.launch.py')


def _text():
    return open(_GAZEBO_LAUNCH, encoding='utf-8').read()


def _node_block(text, name):
    """name='<name>' を持つ Node(...) ブロックを返す（次の Node( か 1200 文字まで）。"""
    idx = text.find(f"name='{name}'")
    assert idx >= 0, f"gazebo.launch.py に name='{name}' の Node 定義が無い"
    nxt = text.find('\n        Node(', idx)
    if nxt < 0:
        nxt = idx + 1200
    return text[idx:nxt]


def test_gazebo_launches_venue_navigator_with_generated_yaml():
    """venue_navigator が sim で起動し、bringup と同じ生成 yaml を読むこと。"""
    text = _text()
    assert "executable='venue_navigator.py'" in text, (
        "gazebo.launch.py に venue_navigator の Node 定義が無い")
    block = _node_block(text, 'venue_navigator')
    assert 'package=\'th_onsite\'' in text, "venue_navigator の package が th_onsite でない"
    assert 'venue_navigator.yaml' in block, (
        "venue_navigator が生成 yaml（GENERATED_DIR/venue_navigator.yaml）を読んでいない")
    assert 'condition=IfCondition(sim)' in block, (
        "venue_navigator の Node に condition=IfCondition(sim) が無い")


def test_gazebo_launches_slam_control_for_map_session_status():
    """slam_control が sim に居ること（/map_session/status の publisher 兼
    start_measure_map の effect の受け手。自律測定の走行が頼る）。
    既に在ることの固定であり、この試験が赤になったら走行の前提が崩れる。"""
    text = _text()
    assert "executable='slam_control.py'" in text, (
        "gazebo.launch.py に slam_control の Node 定義が無い"
        "（/map_session/status が出ず meas.goal の map 照合が通らない）")
    assert "name='slam_control'" in text


def test_gazebo_nav2_sim_provides_follow_and_compute_path():
    """Nav2 の navigation_launch が sim で起動すること
    （venue_navigator の FollowPath / ComputePathToPose の受け手）。"""
    text = _text()
    assert 'navigation_launch.py' in text, "gazebo.launch.py に navigation_launch が無い"
    # sim 用の include は IfCondition(sim) であること。
    assert re.search(r"condition=IfCondition\(sim\)", text), \
        "nav2_sim の include に condition=IfCondition(sim) が無い"
