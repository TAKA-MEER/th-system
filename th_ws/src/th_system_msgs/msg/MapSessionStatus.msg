std_msgs/Header header

string slot         # "VENUE"/"ROUTE"
string session_id
string mode         # "UNLOADED"/"MAPPING"/"LOCALIZING"
bool   dirty
# WS-9AL(2026-09-11): 地図の「生存世代」を表す乱数トークン。まっさらな地図
# 作成を始めるたび新規発行し、地図を読み直せたら読み込んだ地図のものを
# 継承する。ピン（pins.yaml の同名フィールド／PinList.map_instance_id）と
# 突き合わせて、bringup 再起動をまたいだ取り違えを検知する。
string instance_id
