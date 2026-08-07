std_msgs/Header header

# 呼び寄せ (SUMMONING) の進行イベント。
#   "ACCEPTED"  — Nav2 がゴールを受理し移動を開始した
#   "ARRIVED"   — 試験員の手前に到着した
#   "CANCELLED" — SUMMONING から離脱してゴールを取り消した
#   "REJECTED"  — Nav2 がゴールを拒否した
#   "FAILED"    — Nav2 が失敗で終了した (到着できなかった)
#
# 到着 (ARRIVED) と中止 (CANCELLED) はどちらも SUMMONING → IDLE の遷移を
# 伴うため、モード遷移だけを見ている購読側では区別できない。本トピックは
# その区別を与えるために存在する。
string event

string message   # 補足 (Nav2 の status 等)。無い場合は空文字列
