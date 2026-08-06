#!/bin/bash
# ============================================================
# verify_spin_tracking.sh — 自機回転補償の動作確認 (実機・LiDAR 不要)
#
# 回転する odom→base_link TF と、odom 上で静止した脚検出を合成で与え、
# PersonTracker が旋回中も対象を捕捉し続けるかを確認する
# (VISION.md §4 / docs/architecture.md「自機回転補償」)。
#
# 使い方 (コンテナ内):
#   bash /root/th_ws/scripts/verify_spin_tracking.sh
#
# 注意:
#   - 他の PersonTracker が動いていないこと (ノード名が衝突する)
#   - 実機セッションの最中には流さない。先に
#     `ps -eo pid,etimes,args` で稼働中のプロセスを確認すること
# ============================================================
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"

setsid ros2 run rclcpp_components component_container_mt \
  --ros-args -r __node:=spin_test_container > /tmp/spin_container.log 2>&1 < /dev/null &
CONTAINER_PID=$!
# 自分のコンテナだけを確実に止める (pkill -f はこのスクリプト自身にマッチしうる)
trap 'kill -9 $CONTAINER_PID 2>/dev/null' EXIT
sleep 5

ros2 component load /spin_test_container multiple_sensor_person_tracking \
  multiple_sensor_person_tracking::PersonTracker \
  -p detection_mode:=leg -p target_frame:=base_link -p scan_frame_name:=laser_link \
  -p odom_frame_name:=odom -p scan_topic_name:=/scan_filtered \
  -p dr_spaam_topic_name:=/dr_spaam/dr_spaam_detections \
  -p leg_tracking_range:=1.10 -p target_range:=3.0 -p display_marker:=false \
  > /tmp/spin_load.log 2>&1
sleep 2

ros2 lifecycle set /person_tracker configure > /dev/null 2>&1
ros2 lifecycle set /person_tracker activate  > /dev/null 2>&1

python3 "$HERE/verify_spin_tracking.py"
