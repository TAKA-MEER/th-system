std_msgs/Header header

# candidates は base_link フレーム (ロボット基準相対座標)
geometry_msgs/Point[] candidates
int32   selected_index
float32 confidence
bool    is_lost
string  lost_reason
