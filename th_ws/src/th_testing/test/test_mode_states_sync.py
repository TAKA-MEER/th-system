"""test_mode_states_sync.py — モード→状態の表が 2 箇所でずれていないことを固定する。

実機で踏んだ事故（2026-09-08）:
    AT_HOME モードを足した直後、そのモードに入った瞬間に
    `[FAULT] STATE_INCONSISTENT (severity=CRITICAL)` → ESTOP。
    解除するとまた AT_HOME に戻るので、また落ちる（ログで 4 回連続を確認）。

原因は `safety_monitor_core.cpp` の `default_mode_states()` に AT_HOME が
無かったこと。この表は `th_state/th_state/state_core.py` の `MODE_STATES` と
同じものを **C++ 側に手で書き写している**（ソースのコメントいわく「同期が
ずれても検出漏れになるだけで誤検知の方向にはならないよう、両者は独立に
DetailedDesign-names.md §3 の表から書き起こしてある」）。

だが実際には**誤検知の方向に倒れた**。C++ 側に無いモードは「知らない mode」
として STATE_INCONSISTENT になるからで、モードを足すたびに同じ事故が起きる。
2 つの表が一致していることをここで固定する。

ROS2 環境は不要（Python を import し、C++ をテキストとして読むだけ）。
"""
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, '..', '..'))
_CPP = os.path.join(_SRC, 'th_safety', 'src', 'safety_monitor_core.cpp')

sys.path.insert(0, os.path.join(_SRC, 'th_state'))
from th_state.state_core import MODE_STATES  # noqa: E402


def _cpp_mode_states():
    """safety_monitor_core.cpp の default_mode_states() を読み取る。

    `{"MODE", {"S1", "S2"}},` の形の行だけを拾う（コメント行は括弧の形が
    違うので自然に外れる）。
    """
    text = open(_CPP, encoding='utf-8').read()
    body = re.search(r'kModeStates\s*=\s*\{(.*?)\n\s*\};', text, re.S)
    assert body, 'safety_monitor_core.cpp の kModeStates が見つからない'
    out = {}
    for mode, states in re.findall(r'\{"(\w+)",\s*\{([^}]*)\}\}', body.group(1)):
        out[mode] = set(re.findall(r'"(\w+)"', states))
    return out


def test_cpp_and_python_mode_states_match():
    """C++ の default_mode_states() と Python の MODE_STATES が完全一致する。

    片方だけにモードを足すと、そのモードに入った瞬間に safety_monitor が
    STATE_INCONSISTENT(CRITICAL) を上げて ESTOP になる（2026-09-08 の AT_HOME）。
    """
    cpp = _cpp_mode_states()
    assert cpp, 'C++ 側の表を 1 行も読めていない（正規表現が実装とずれた）'

    only_py = sorted(set(MODE_STATES) - set(cpp))
    only_cpp = sorted(set(cpp) - set(MODE_STATES))
    assert not only_py, (
        f'Python にあって C++ に無いモード: {only_py}. '
        'safety_monitor_core.cpp の default_mode_states() に足すこと'
        '（無いとそのモードに入った瞬間 STATE_INCONSISTENT → ESTOP）')
    assert not only_cpp, (
        f'C++ にあって Python に無いモード: {only_cpp}. '
        'state_core.py の MODE_STATES と食い違っている')

    mismatched = {m: (sorted(MODE_STATES[m]), sorted(cpp[m]))
                  for m in sorted(MODE_STATES) if MODE_STATES[m] != cpp[m]}
    assert not mismatched, f'状態集合が食い違うモード（python, cpp）: {mismatched}'
