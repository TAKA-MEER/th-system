std_msgs/Header header
geometry_msgs/Point position  # ロボット基準相対位置 (m)
float32 confidence            # 検出信頼度 0.0-1.0
bool    is_lost               # ロストフラグ
string  lost_reason           # "" / "DETECTION_LOST" / "LOW_CONFIDENCE" / "TARGET_SWITCHED"
