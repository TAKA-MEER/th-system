std_msgs/Header header

# いま立っているフォルト種別の集合（Spec-safety §3.5.5 の記録用）。
# safety_monitor が active_faults_ の変化のたびに出す。types は辞書順で安定。
string[] types
string[] severities    # types と同じ順。"RECOVERABLE" / "CRITICAL"
