"""
conftest.py — pytest パス設定
================================
colcon build 前でも `pytest test/` で直接実行できるよう
th_planning の Python パッケージパスを sys.path に追加する。
"""
import sys
import os

# th_planning パッケージのルートを追加
_repo_root  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
_planning   = os.path.join(_repo_root, 'th_planning')
_core_dir   = os.path.join(_repo_root, 'th_planning', 'th_planning')

for _p in [_planning, _core_dir]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
