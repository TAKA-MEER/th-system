std_msgs/Header header

bool   alive
string action                 # "PASS"/"CLAMP"/"STOP"/"ZERO_STALE"/"BLOCKED_UNCALIBRATED"
float32 in_linear
float32 out_linear
float32 nearest_obstacle_m
string source_class           # "MANUAL"/"AUTO"
float32 applied_limit_mps
bool   approach_warning       # 自動ブレーキ OFF の手動系が、ON なら減速が始まる距離に入った（速度は落とさない。1b-15 SG-B10）
float32 approach_distance_m   # 接近警告の元の距離。動いているときは進む向き、止まっているときは死角を除く周囲の最近傍（2026-11-12）。画面はこちらを出す
bool   approach_warning_moving  # approach_warning のうち機体が動いている間だけ立つもの。スピーカーの警告音用（止まっているときは画面だけ。Spec-safety §11）
