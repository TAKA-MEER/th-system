#!/bin/bash
# W-01 P0: posegraph/.data -> pgm/yaml 書き出し（コンテナ内で実行）
# 使い方: export_maps.sh <id> [<id> ...]   (例: export_maps.sh 長距離試験0904-2)
# 前提: /mnt/routes に入力(:ro)、/w01p0/maps に出力、/w01p0/export.log に記録
LOG=/w01p0/export.log
echo "=== start $(date -u +%FT%TZ) ===" >> "$LOG"

source /opt/ros/humble/setup.bash

setsid ros2 run slam_toolbox localization_slam_toolbox_node \
  --ros-args -p use_sim_time:=false > /w01p0/maps/_node.log 2>&1 < /dev/null &
NODE_PID=$!
echo "node pid=$NODE_PID" >> "$LOG"
sleep 10
ros2 service list 2>/dev/null | grep -i slam >> "$LOG"

for ID in "$@"; do
  echo "--- $ID: deserialize ---" >> "$LOG"
  # localization モードは 1・2 を拒否する（"non-localization deserialization"）。
  # 地図全体の読込には 3 (LOCALIZE_AT_POSE) を使う。初期姿勢は経路始点≒原点。
  timeout 120 ros2 service call /slam_toolbox/deserialize_map \
    slam_toolbox/srv/DeserializePoseGraph \
    "{filename: '/mnt/routes/${ID}', match_type: 3, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}" >> "$LOG" 2>&1
  echo "rc=$? --- $ID: wait /map ---" >> "$LOG"
  sleep 5
  timeout 30 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once \
    --qos-durability transient_local \
    --field header,info >> "$LOG" 2>&1
  echo "--- $ID: map_saver_cli ---" >> "$LOG"
  timeout 60 ros2 run nav2_map_server map_saver_cli -f "/w01p0/maps/${ID}" >> "$LOG" 2>&1
  echo "rc=$? --- $ID: save_map ---" >> "$LOG"
  timeout 60 ros2 service call /slam_toolbox/save_map \
    slam_toolbox/srv/SaveMap "{name: {data: '/w01p0/maps/${ID}_slam'}}" >> "$LOG" 2>&1
  echo "rc=$?" >> "$LOG"
  ls -lh "/w01p0/maps/${ID}"* >> "$LOG" 2>&1
done

kill -TERM "$NODE_PID" 2>/dev/null
sleep 3
chown -R 1000:1000 /w01p0/maps
echo "=== done $(date -u +%FT%TZ) ===" >> "$LOG"
