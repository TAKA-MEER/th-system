std_msgs/Header header

string    state
RouteInfo current
float32   recorded_m
float32   elapsed_sec
uint32    points
int32     target_index   # 再生中の pure-pursuit 目標点の添字。記録中や未走行は -1
float32   start_yaw      # 記録側のみ使用。記録開始時の向き [rad]
                         # （S-13「開始時の向き」表示用。Spec-webui.md §3.6）。
                         # 記録を開いていないときは 0。
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
string    localize_quality  # 再生側のみ。全域ローカライズの確度（W-01 P2）。
                          # '' 未評価／'searching' 探索中／'high' 成立／
                          # 'low_margin' 成立したが m < localize_margin_low
                          # （似た場所あり。READY へは進む）／'failed' 不成立
                          # （LOCALIZE に留まる）／'unknown' pgm 無しの旧経路
                          # （始点決め打ち）。load_route で '' に戻す。
                          # READY・RUN・PAUSE 中も最後の値を出し続ける。
float32   localize_score    # 再生側のみ。最後の探索の s（0〜1）。未評価は 0。
float32   localize_margin   # 再生側のみ。最後の探索の m。未評価は 0。
bool      has_map           # 再生側のみ。読み込んだ経路に保存地図があるか（1b-11 SG-B14）。
                            # state_manager が map_update_available の材料にする。
                            # 記録側は出さない（常に false）。
                            # 末尾に追加（既存フィールドの順序は変えない）。
