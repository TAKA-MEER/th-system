std_msgs/Header header

Pin[] pins
# WS-9AL(2026-09-11): これらのピンが登録された時点で有効だった地図の
# instance_id（MapSessionStatus.msg と同じ意味）。slam_control が
# LOCALIZE_AT_POSE の初期姿勢に HOME ピンを信用してよいか判定するのに使う。
string map_instance_id
