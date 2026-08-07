std_msgs/Header header

# どちらの追従ノードが発行したか
#   "map"     — follow_planner (地図あり追従・退避シーケンス)
#   "mapless" — follow_planner_mapless (地図なし軌跡追従)
#   "face"    — face_planner (超信地旋回のみ・展示用、並進なし)
string planner

# 追従ロジックの内部状態
#   共通    : "INACTIVE" (当該モードでない)
#   mapless : "TRACKING" / "STOPPED"
#   map     : "TRACKING" / "PREPARE" / "EVADING"
#   face    : "TRACKING" / "ALIGNED" / "STOPPED"
string state

# 状態・停止の理由。state だけでは区別できない事象を表す。
#   "tracking"        — 正常に追従中
#   "person_lost"     — 試験員を見失って停止中
#   "person_close"    — 接近しすぎて停止中 (mapless)
#   "obstacle_ahead"  — 進路上の障害物で停止中 (mapless)
#   "no_scan"         — LiDAR スキャン未受信で停止中 (mapless)
#   "no_pose"         — 自己位置(TF/costmap)未確立で停止中
#   "preparing"       — 退避準備 (向き合わせ) 中 (map)
#   "evading"         — 退避走行中 (map)
#   "retreat_blocked" — 周囲に退避できる空きがなくその場停止 (map)
#   "aligned"         — 正対完了、角速度ほぼ0で待機中 (face)
#   "low_confidence"  — confidence 不足で旋回停止中 (face)
#   "no_odom"         — odom TF未取得のため自己移動補償なしの生値で旋回中 (face)
#
# 注意: "retreat_blocked" と "no_pose" は必ず区別すること。両者を混ぜると
# costmap 未受信のたびに「逃げ場なし」の警告が誤って出る。
string reason

float32 person_distance   # m 試験員との距離。不明時は -1.0
float32 linear_x          # m/s この周期の指令前進速度

# 退避方向 (base_link 相対角)。has_escape_angle が false のとき escape_angle は無効。
bool    has_escape_angle
float32 escape_angle
