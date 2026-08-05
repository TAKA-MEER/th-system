std_msgs/Header header

# 試験員ロスト後の段階。
#   "NONE"       — 捕捉中 (ロストしていない)
#   "PREDICTING" — 直近の速度ベクトルで位置を外挿して追従継続中
#   "SEARCHING"  — 予測上限を超えたのでその場旋回で捜索中
#   "GAVE_UP"    — 捜索タイムアウト。発見できなかった
#
# 既存の /person/search_mode (Bool) はこの phase から導出して従来どおり
# 発行し続ける (SEARCHING または GAVE_UP のとき true)。Bool のままでは
# 捜索中と打ち切り後を区別できないため本トピックを追加した。
string phase

float32 lost_elapsed_sec  # ロストしてからの経過秒。NONE のときは 0.0
