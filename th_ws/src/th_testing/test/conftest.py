"""
conftest.py — pytest パス設定
================================
colcon build 前でも `pytest test/` で直接実行できるよう
th_planning / th_esp32_bridge の Python パッケージパスを sys.path に追加する。
"""
import sys
import os

_repo_root  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# th_planning パッケージのルートを追加
_planning     = os.path.join(_repo_root, 'th_planning')
_planning_core = os.path.join(_repo_root, 'th_planning', 'th_planning')

# th_esp32_bridge パッケージのルートを追加
_esp32_bridge      = os.path.join(_repo_root, 'th_esp32_bridge')
_esp32_bridge_core = os.path.join(_repo_root, 'th_esp32_bridge', 'th_esp32_bridge')

for _p in [_planning, _planning_core, _esp32_bridge, _esp32_bridge_core]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
