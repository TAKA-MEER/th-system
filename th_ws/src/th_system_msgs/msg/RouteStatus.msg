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
float32   cross_track_m    # 再生側のみ使用。現在位置が経路から横にどれだけ外れているか [m]
                          # （最寄り区間への垂線距離・符号なし。S-14 表示用。
                          # Spec-webui.md §3.7 / Spec-transit.md §4.4）。
                          # 記録側は出さない（常に 0）。
float32   cross_track_max_m # 再生側のみ使用。この再生での cross_track_m の最大値 [m]。
                          # 停止しても残し、次の再生（load_route）で 0 に戻す。
                          # 記録側は出さない（常に 0）。
