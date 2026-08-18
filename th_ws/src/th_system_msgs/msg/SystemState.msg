std_msgs/Header header

string mode              # 18 モード。文字列で持つ (DetailedDesign-names.md §2.1)
string state              # サブステート。モードに状態が無ければ "NONE"
string prev_mode          # ESTOP / CARRY からの復帰先ラッチ
string prev_state         # 同上
string zone               # "IN" / "OUT" / "NA"

bool jog_active           # 手動ジョグ介入中 (リース式)
bool estop_ui             # WebUI の非常停止が押されている
bool estop_hw             # 物理非常停止が押されている
bool tracker_enabled      # 人物追跡が動いている
bool auto_brake           # 自動ブレーキが有効
bool working              # 盤前の「作業中」ボタンが ON
bool map_update           # 地図の書き足しが ON

string[] unsaved          # 未保存の種別 ("venue_map" / "route" / "calib" / "map_patch")

builtin_interfaces/Time since   # 現在の mode/state に入った時刻
string last_event               # 直近に処理した evt.*
string last_reject_reason       # 直近に拒否された遷移の理由キー ("" なら無し)
