#!/usr/bin/env bash
# WP-MEAS-04 リンク品質の記録（手順書 B-0 の共通条件）
#   引数 1: 構成名（出力は /out/<構成名>.csv）
#   引数 2: 記録秒数（既定 900）
#   引数 3: 車輪速度 m/s（既定 0.0 = 停止。エンコーダ ISR 負荷を掛けるなら 0.5 等）
# ★ 引数 3 を 0 以外にすると車輪が回る。必ず車体を浮かせた状態で使うこと。
CONF="${1:?構成名}"
DUR="${2:-900}"
SPD="${3:-0.0}"
cd /root/th_ws
echo "[meas04] ビルド中..."
colcon build --symlink-install > /tmp/build.log 2>&1 || { tail -20 /tmp/build.log; exit 1; }
source install/setup.bash

echo "[meas04] ノード起動 (速度 ${SPD} m/s)"
setsid ros2 run th_esp32_bridge esp32_bridge.py --ros-args \
  --params-file src/th_esp32_bridge/config/params.yaml > /tmp/bridge_${CONF}.log 2>&1 &
BRIDGE=$!
setsid ros2 run th_safety safety_monitor --ros-args \
  --params-file src/th_safety/config/safety_monitor.yaml > /tmp/safety_${CONF}.log 2>&1 &
SAFETY=$!
setsid ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: ${SPD}}, angular: {z: 0.0}}" > /dev/null 2>&1 &
CMDVEL=$!
sleep 15

echo "[meas04] 記録開始: ${DUR} 秒 → /out/${CONF}.csv"
timeout "${DUR}" ros2 topic echo /safety/link_quality --csv > "/out/${CONF}.csv" 2>/dev/null
echo "[meas04] 記録終了 ${CONF}: $(wc -l < /out/${CONF}.csv) 行"

# 停止指令を送ってからノードを落とす（車輪を止める）
timeout 3 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
kill -TERM -$CMDVEL 2>/dev/null; sleep 1
timeout 3 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
for p in $SAFETY $BRIDGE; do kill -TERM -$p 2>/dev/null; done
sleep 2
echo "[meas04] 完了 ${CONF}"
