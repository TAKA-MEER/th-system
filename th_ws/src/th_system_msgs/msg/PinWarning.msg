# pin_registrar → WebUI（transient_local、latched）。
# 登録しようとしたピンが壁に近すぎるときに出す（2026-09-10。WS-9AB 系）。
# active=false のときは他フィールドは無効（警告なし）。
bool    active
string  kind          # "HOME" / "PANEL"
float32 nearest_m     # 最寄りの占有セルまでの距離
float32 min_m         # 推奨クリアランス（pin_min_clearance_m）
float32 retreat_x     # 「壁から離して登録」を選んだときの退避後 x（map 系）
float32 retreat_y     # 同 y
float32 retreat_yaw   # yaw は変えない（元の値）
