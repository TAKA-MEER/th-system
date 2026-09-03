#!/usr/bin/env bash
# 地図を捨てて SLAM を作り直す（地図をやり直したいときの代替手順）。
#
#   docker exec -it th_robot bash /root/th_ws/scripts/reset_map.sh
#
# 通常の教示→再生の流れでは**このスクリプトは使わない**。地図は「保存」を押すと
# 経路と一緒に自動で保存され、再生時に自動で読み直される（docs/使い方.md §4-4）。
# ここを使うのは「教示と再生を別々の起動セッションでやってしまった」「地図が
# 壊れたので作り直したい」といった、地図を捨てて最初から作り直したいときだけ。
#
# 行う操作: 地図を捨てると、今いる場所が新しい地図の原点になる。ロボットを手で
# 動かしても「自分がどこに居ると思っているか」は変わらない（docs/使い方.md §4-3
# 「なぜ再生前に地図を読み直すのか」）。
#
# 手で 2 つのサービスを叩くのと同じことをするが、
#   - 2 つ目（set_mapping true → 地図作成再開）の叩き忘れを防ぐ
#     （忘れると地図が空のまま止まり、自己位置補正が無言で効かなくなる）
#   - respawn で slam_toolbox が戻るのを待つ
#   - set_mapping の戻り値と、最後の効きを確認する
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
# async_slam_toolbox_node には set_localization_mode は無い（WS-9L）。
# 実在する pause_new_measurements の出現で再起動を検知する。
for i in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q '^/slam_toolbox/pause_new_measurements$'; then
    echo "  戻ってきた（${i} 秒）"
    break
  fi
  sleep 1
done
if ! ros2 service list 2>/dev/null | grep -q '^/slam_toolbox/pause_new_measurements$'; then
  echo "NG: slam_toolbox が 60 秒経っても戻ってこない。bringup のログを見ること" >&2
  exit 1
fi
sleep 2   # サービスが出た直後は set_mapping を取りこぼすことがある

echo
echo "=== ③ 地図作成を再開する（これを忘れると再生が壊れる）==="
set +e   # 戻り値を見るため、この呼び出しだけ失敗しても続行する
set_mapping_out="$(ros2 service call /slam_control/set_mapping std_srvs/srv/SetBool "{data: true}" 2>&1)"
set_mapping_rc=$?
set -e
if [ "$set_mapping_rc" -ne 0 ] || ! echo "$set_mapping_out" | grep -q 'success: True'; then
  echo "NG: /slam_control/set_mapping が成功していない（success: True が返っていない）" >&2
  echo "    $set_mapping_out" >&2
  exit 1
fi

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
