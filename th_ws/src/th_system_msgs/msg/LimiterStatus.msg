std_msgs/Header header

bool   alive
string action                 # "PASS"/"CLAMP"/"STOP"/"ZERO_STALE"/"BLOCKED_UNCALIBRATED"
float32 in_linear
float32 out_linear
float32 nearest_obstacle_m
string source_class           # "MANUAL"/"AUTO"
float32 applied_limit_mps
bool   approach_warning       # 自動ブレーキ OFF の手動系が、ON なら減速が始まる距離に入った（速度は落とさない。1b-15 SG-B10）
