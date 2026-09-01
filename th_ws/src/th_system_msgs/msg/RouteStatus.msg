std_msgs/Header header

string    state
RouteInfo current
float32   recorded_m
float32   elapsed_sec
uint32    points
int32     target_index   # 再生中の pure-pursuit 目標点の添字。記録中や未走行は -1
