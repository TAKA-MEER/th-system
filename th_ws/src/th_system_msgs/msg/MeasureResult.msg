# MeasureResult.msg — 測定機能（fk-system）とのやり取り。interface_version "1"。
# 正本は docs/plan/detailed/DetailedDesign-measure.md §3。型・名前・意味を変えたら版を上げるか fk に知らせる（§1.2）。
std_msgs/Header header
string command_id
string command
string result      # ARRIVED / DONE / CANCELLED（FAILED は予約。いまは出さない）
string reason      # CANCELLED の理由キー。ほかは空
