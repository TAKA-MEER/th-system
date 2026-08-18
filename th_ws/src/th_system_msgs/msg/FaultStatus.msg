bool   active
string fault_type    # "NONE" / "LIDAR_LOST" / "ESP32_DISCONNECTED" / "PERSON_TRACKER_LOST"
string description
string severity       # "RECOVERABLE" / "CRITICAL" (WP-MSG-01 で追加。M3 の唯一の例外)
