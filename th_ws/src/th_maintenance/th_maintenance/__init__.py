"""th_maintenance — 始業点検（OPCHECK）の実行部。

豆知識: `check_core` は ROS2 非依存の純粋関数（`judge_*` / `estimate_*`）、
`scripts/opcheck_runner.py` はその呼び出し側（ノード）。
"""