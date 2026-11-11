# MeasureHeartbeat.msg — 測定機能（fk-system）とのやり取り。interface_version "1"。
# 正本は docs/plan/detailed/DetailedDesign-measure.md §3。型・名前・意味を変えたら版を上げるか fk に知らせる（§1.2）。
std_msgs/Header header
string interface_version
string sender            # "fk-system"
uint32 seq
