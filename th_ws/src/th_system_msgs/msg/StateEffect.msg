# 教示・再生ノードへ effect を配送するメッセージ（P2 / demo-teach-replay）
std_msgs/Header header
string name          # effect 名 (start_record / load_route / ...)
string dest          # 宛先ノード名 (_EFFECT_DESTINATIONS の値。不明なら "unknown")
string args_json     # 付随データ (JSON 文字列。引数なしの effect は "{}")
