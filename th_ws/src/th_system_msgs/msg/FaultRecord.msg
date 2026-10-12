string fault_type
float64 started_at      # 機体の時計 (epoch 秒)
float64 duration_s      # 解消済みの継続時間。継続中は -1.0
bool ongoing
string mode             # 発生時の /system/state の mode
string state            # 同 state
string severity         # "RECOVERABLE" / "CRITICAL"
string outcome          # "PAUSE" / "ESTOP" / "NONE"
