std_msgs/Header header

string mode              # 18 モード。文字列で持つ (DetailedDesign-names.md §2.1)
string state              # サブステート。モードに状態が無ければ "NONE"
string prev_mode          # ESTOP / CARRY からの復帰先ラッチ
string prev_state         # 同上
string zone               # "IN" / "OUT" / "NA"

bool jog_active           # 手動ジョグ介入中 (リース式)
bool estop_ui             # WebUI の非常停止が押されている
bool estop_hw             # 物理非常停止が押されている
bool estop_from_ui        # 現在の ESTOP が UI ボタン起因か (重大フォルト起因と区別。SM-3.1.1-11)
bool tracker_enabled      # 人物追跡が動いている
bool auto_brake           # 自動ブレーキが有効
bool working              # 盤前の「作業中」ボタンが ON
bool map_update           # 地図の書き足しが ON

string[] unsaved          # 未保存の種別 ("venue_map" / "route" / "calib" / "map_patch")

builtin_interfaces/Time since   # 現在の mode/state に入った時刻
string last_event               # 直近に処理した evt.*
string last_reject_reason       # 直近に拒否された遷移の理由キー ("" なら無し)

string speed_limit        # 速度上限。registry のパラメータ名 or "stop"（数値ではない）
                           # 画面由来（names.md §4.1 derive_limits）とモード由来
                           # （attributes.yaml）を combine_speed_limits() で合成した
                           # 結果。obstacle_limiter はこれに v_reverse と障害物
                           # クランプを重ねる（N-13 で新設・N-15 で合成対象を拡張）
