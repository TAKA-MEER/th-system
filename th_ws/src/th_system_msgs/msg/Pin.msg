# 待機場所ピン・配電盤ピン。配列要素として使われるため Header を持たない (M2)。

string id
string name
string kind                   # "HOME"/"PANEL"
# pose は map フレーム
geometry_msgs/Pose pose
builtin_interfaces/Time registered_at
