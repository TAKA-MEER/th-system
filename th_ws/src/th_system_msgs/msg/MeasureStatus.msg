# MeasureStatus.msg — 測定機能（fk-system）とのやり取り。interface_version "1"。
# 正本は docs/plan/detailed/DetailedDesign-measure.md §3。型・名前・意味を変えたら版を上げるか fk に知らせる（§1.2）。
std_msgs/Header header
string  interface_version
string  mode                 # SystemState と同じ文字列
string  state
bool    working              # 停止命令の間
bool    link_ok              # 本システムから見た測定機能の生存（§3.4）
bool    pose_valid           # 自己位置が使えるか
string  map_instance_id      # 地図の生存世代（MapSessionStatus.instance_id と同じ意味）
float64 x                    # map フレーム [m]
float64 y
float64 yaw                  # [rad]
string  pending_command_id   # 進行中の指示（無ければ空）
string  pending_command
string  pause_reason         # SystemState.pause_reason と同じ
bool    blocked              # 塞がれて止まっている（BLOCKED）
bool    reentry_required     # 自己位置を見失ったあと、自律測定を抜けて入り直すまで true（-142）
float32 v_limit_mps          # いま効いている速度の上限（本システムと測定機能の指定の小さい方）
float32 w_limit_radps
