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
# bringup を止めてから実行すること。person_tracker のノード名が衝突し、
# /tf や /scan_filtered も実機側と混ざるため。起動中は自動で中断する。
# ============================================================

# 注意: set -u は ROS の setup.bash より後に置く。setup.bash は未定義変数を
# 参照するため、先に set -u すると "AMENT_TRACE_SETUP_FILES: unbound variable"
# で即死する。
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
set -u

# ── 実機セッションとの衝突を防ぐ ──────────────────────────
running=$(ps -eo args --no-headers \
  | grep -v grep \
  | grep -cE 'component_container|ros2 launch th_bringup' || true)
if [ "$running" -ne 0 ]; then
  echo "エラー: bringup または component_container が既に動いています。" >&2
  echo "        person_tracker のノード名が衝突し /tf も混ざるため中断します。" >&2
  echo "        bringup を止めてから実行してください。" >&2
  ps -eo pid,etimes,args --no-headers | grep -v grep \
    | grep -E 'component_container|ros2 launch th_bringup' | cut -c1-100 >&2
  exit 1
fi

# ── テスト用コンテナ起動 ───────────────────────────────────
# setsid は使わない。$! が実際のコンテナ PID と一致せず、スクリプト終了後に
# 孤児として残り続ける事故があったため (CLAUDE.md「環境の癖」参照)。
ros2 run rclcpp_components component_container_mt \
  --ros-args -r __node:=spin_test_container > /tmp/spin_container.log 2>&1 &
CONTAINER_PID=$!

cleanup() {
  kill -TERM "$CONTAINER_PID" 2>/dev/null
  for _ in 1 2 3 4 5; do
    kill -0 "$CONTAINER_PID" 2>/dev/null || return
    sleep 1
  done
  echo "コンテナが TERM で終了しないため KILL します (pid $CONTAINER_PID)" >&2
  kill -9 "$CONTAINER_PID" 2>/dev/null
}
trap cleanup EXIT

sleep 5
if ! kill -0 "$CONTAINER_PID" 2>/dev/null; then
  echo "エラー: component_container_mt が起動しませんでした" >&2
  cat /tmp/spin_container.log >&2
  exit 1
fi

# ── PersonTracker をロード ─────────────────────────────────
if ! ros2 component load /spin_test_container multiple_sensor_person_tracking \
  multiple_sensor_person_tracking::PersonTracker \
  -p detection_mode:=leg -p target_frame:=base_link -p scan_frame_name:=laser_link \
  -p odom_frame_name:=odom -p scan_topic_name:=/scan_filtered \
  -p dr_spaam_topic_name:=/dr_spaam/dr_spaam_detections \
  -p leg_tracking_range:=1.10 -p target_range:=3.0 -p display_marker:=false \
  > /tmp/spin_load.log 2>&1
then
  echo "エラー: PersonTracker のロードに失敗しました" >&2
  cat /tmp/spin_load.log >&2
  exit 1
fi
sleep 2

ros2 lifecycle set /person_tracker configure > /dev/null 2>&1
ros2 lifecycle set /person_tracker activate  > /dev/null 2>&1

python3 "$HERE/verify_spin_tracking.py"
