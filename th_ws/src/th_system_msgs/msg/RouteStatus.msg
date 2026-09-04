std_msgs/Header header

string    state
RouteInfo current
float32   recorded_m
float32   elapsed_sec
uint32    points
int32     target_index   # 再生中の pure-pursuit 目標点の添字。記録中や未走行は -1
bool      saved          # 記録側のみ使用。finalize_route_file が成功したときだけ true。
                         # FSM の状態（SAVED）とは無関係。ファイルが書けたかどうかだけ。
bool      arrived        # 再生側のみ使用。経路の終端まで走り切ったときだけ true。
                         # PAUSE は「終端に達した」以外に「フォルト」「ジョグ介入」
                         # 「停止ボタン」でも起きる。UI がその区別に使う（WS-9P）。
