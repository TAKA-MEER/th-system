std_msgs/Header header  # WP-SAFE-01 で追加。M3(WP-MSG-01)は severity 追加のみを例外としていたが、
                         # names.md §5.1 がこのパケットでの先頭挿入を明示的に許可している

bool   active
string fault_type    # "NONE" / "LIDAR_LOST" / "ESP32_DISCONNECTED" / "PERSON_TRACKER_LOST" /
                      # "UI_DISCONNECTED" / "LIMITER_DEAD" / "MUX_DEAD" / "DRIVE_RUNAWAY" /
                      # "STATE_INCONSISTENT" / "ESTOP_BYPASS_ACTIVE"
                      # ("LOCALIZATION_LOST" は severity 分類表にのみ存在。検出は WP-SAFE-05)
string description
string severity       # "RECOVERABLE" / "CRITICAL" (WP-MSG-01 で追加。M3 の唯一の例外)
