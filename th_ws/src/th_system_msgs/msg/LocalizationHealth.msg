# LocalizationHealth.msg — 自己位置推定の健全性（WP-SAFE-05）。
#
# safety_monitor の `localization` 監視の入力。一定周期（1 Hz）で出し続け、
# 止まったこと自体が異常（トピック途絶）の合図になる（LimiterStatus と同じ流儀）。
#
# reason の文字列定義はこのコメントが正（リポジトリの流儀）:
#   ""               正常
#   "stale"          A: map→odom が localization_stale_ms 更新されない（凍結）
#   "node_down"      C: 自己位置推定のノード（localization_expected_nodes）が
#                    1 つも居ない（起動失敗・クラッシュ）
#   "low_confidence" B 用の予約。B は範囲外（O-e1。信号源未確定）のため出さない。
std_msgs/Header header

bool ok

string reason

# 判断に使った値
float32 transform_age_sec
bool node_present
