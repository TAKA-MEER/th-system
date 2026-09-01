std_msgs/Header header

string screen_id     # S-xx 画面 ID
string client_id     # 端末を識別する ID
bool   interacting   # この端末が現在操作中か

# header.stamp は 2Hz の定期発行時刻であって「最後の操作時刻」ではない。
# last_input が無いと、画面を開いているだけの端末が永久に「使用中」になる。
builtin_interfaces/Time last_input
