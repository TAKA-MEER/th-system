#!/usr/bin/env bash
# 地図を捨てて SLAM を作り直す（教示のループが閉じなかったときの代替手順）。
#
#   docker exec -it th_robot bash /root/th_ws/scripts/reset_map.sh
#
# ロボットを**始点マークの上に置いた状態で**実行すること。地図を捨てると、
# 今いる場所が新しい地図の原点になり、記録した経路の 1 点目（＝原点）と一致する。
# ロボットを手で動かしても「自分がどこに居ると思っているか」は変わらないため、
# 位置を戻すだけでは直らない。詳細: docs/使い方.md §4-3
#
# 手で 2 つのサービスを叩くのと同じことをするが、
#   - 2 つ目（set_mapping true）の叩き忘れを防ぐ
#     （忘れると地図が空のまま止まり、自己位置補正が無言で効かなくなる）
#   - respawn で slam_toolbox が戻るのを待つ
#   - 効いたかどうかを最後に確認する
# ぶんだけ安全。
#
# set -u は書かない（source /opt/ros/humble/setup.bash が
# AMENT_TRACE_SETUP_FILES: unbound variable で即死する）。
set -e

SETUP=/root/th_ws/install/setup.bash
[ -f "$SETUP" ] || { echo "NG: $SETUP が無い。コンテナ内で実行しているか確認する" >&2; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$SETUP"

echo "=== ① 地図を捨てて slam_toolbox を作り直す ==="
ros2 service call /slam_control/discard_map std_srvs/srv/Trigger

echo
echo "=== ② slam_toolbox が戻るのを待つ（respawn。最大 60 秒）==="
for i in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q '^/slam_toolbox/set_localization_mode$'; then
    echo "  戻ってきた（${i} 秒）"
    break
  fi
  sleep 1
done
if ! ros2 service list 2>/dev/null | grep -q '^/slam_toolbox/set_localization_mode$'; then
  echo "NG: slam_toolbox が 60 秒経っても戻ってこない。bringup のログを見ること" >&2
  exit 1
fi
sleep 2   # サービスが出た直後は set_mapping を取りこぼすことがある

echo
echo "=== ③ 地図作成を再開する（これを忘れると再生が壊れる）==="
ros2 service call /slam_control/set_mapping std_srvs/srv/SetBool "{data: true}"

echo
echo "=== ④ 効いたか確認する ==="
ok=1

echo "--- /map が出ているか（10 秒見る。数秒に 1 回出れば正常）"
if timeout 12 ros2 topic hz /map 2>&1 | grep -q 'average rate'; then
  echo "  OK: /map が出ている"
else
  echo "  NG: /map が出ていない"
  ok=0
fi

echo "--- map -> odom の TF が出ているか（SLAM が効いている証拠）"
if timeout 8 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q 'Translation'; then
  echo "  OK: map -> odom が出ている"
else
  echo "  NG: map -> odom が出ていない"
  ok=0
fi

echo
if [ "$ok" -eq 1 ]; then
  echo "=== 完了。教示再生の画面に戻って再生してよい ==="
else
  echo "=== NG が出た。このまま再生しないこと。docs/使い方.md §7 を見る ===" >&2
  exit 1
fi
