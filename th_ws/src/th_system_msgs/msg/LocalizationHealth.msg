# LocalizationHealth.msg — 自己位置推定の健全性（WP-SAFE-05）。
#
# safety_monitor の `localization` 監視の入力。比較周期（jump_window_ms。
# 既定 500 ms）で出し続け、止まったこと自体が異常（トピック途絶）の合図に
# なる（LimiterStatus と同じ流儀）。
#
# reason の文字列定義はこのコメントが正（リポジトリの流儀）:
#   ""               正常
#   "stale"          A: map→odom が localization_stale_ms 更新されない（凍結）
#   "node_down"      C: 自己位置推定のノード（localization_expected_nodes）が
#                    1 つも居ない（起動失敗・クラッシュ）
#   "jump"           B′: map→odom が比較周期に許容（jump_translation_m ／
#                    jump_rotation_rad）を超えて動いた（補正が飛んだ）
#   "low_confidence" ゆっくり間違っていく用（O-e2）の予約。出さない。
#
# 優先順位は node_down ＞ stale ＞ jump（localization_health_core.py の
# docstring に理由を記載）。safety_monitor は reason の中身を見ず、
# ok==false の継続で LOCALIZATION_LOST を出す。
std_msgs/Header header

bool ok

string reason

# 判断に使った値
float32 transform_age_sec
bool node_present
