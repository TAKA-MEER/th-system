std_msgs/Header header
geometry_msgs/Point position  # ロボット基準相対位置 (m)
float32 confidence            # 検出信頼度 0.0-1.0
bool    is_lost               # ロストフラグ
string  lost_reason           # "" / "DETECTION_LOST" / "LOW_CONFIDENCE" / "TARGET_SWITCHED" / "disabled"
# ("disabled" は人物検出 OFF（tracker_enabled=false）時に bridge が強制出力する値。
#  brief-tracker-default-off §3.3)
