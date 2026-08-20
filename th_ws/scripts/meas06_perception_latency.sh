#!/usr/bin/env bash
# 手順書 §4.3 の 1 — 知覚経路の遅延の実測
#
# これまで測っていたのは**到着間隔**であって**遅延**ではない。
# 全パケットが一律 1 秒遅れて届いても到着間隔は 100ms のままなので、
# 間隔測定は一定遅延に対して構造的に盲目（手順書 §4.0）。
#
# ここでは各段の更新頻度と、`/person/status` に載っている推定値の**古さ**を測る。
# `person_tracker_bridge.py` は上流のタイムスタンプをそのまま載せ替えるため
# （`th_perception/scripts/person_tracker_bridge.py` の `_last_stamp`）、
# `ros2 topic delay /person/status` が知覚経路の遅延そのものになる。
# その前提が成り立っているかは `/scan` の delay と比べれば分かる（§4.4）。
#
# ★ コンテナの中で実行する:
#     docker compose run --rm th_robot bash scripts/meas06_perception_latency.sh
#
#   引数 1: 各項目の測定秒数（既定 30）
#
# 【要件】ラズパイ側の LiDAR が動いていること。**試験員が 1.5m ほど前に立つこと**
#         （立っていないと検出が出ず、頻度も遅延も測れない）。
#         車輪は回らない。通電不要。
set -u
DUR="${1:-30}"
OUT_DIR=/root/th_data/meas06
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUT_DIR/${STAMP}_latency.txt"
mkdir -p "$OUT_DIR"

cd /root/th_ws
# 非対話シェルは .bashrc を読まないので ROS は自分で source する（CLAUDE.md）
source /opt/ros/humble/setup.bash
echo "[meas06] ビルド中..."
colcon build --symlink-install > /tmp/build.log 2>&1 || { tail -20 /tmp/build.log; exit 1; }
source install/setup.bash

echo "[meas06] /scan の到達を確認中（ラズパイ側 LiDAR）..."
if ! timeout 15 ros2 topic echo /scan --once > /dev/null 2>&1; then
  echo "[meas06] /scan が来ていない。ラズパイ側の LiDAR を起動してから再実行する" >&2
  exit 1
fi

# ── 知覚パイプラインだけを起動（Nav2・安全系は起動しない） ──────
# 構成は docs/architecture.md「person_tracker だけを素早く動作確認する」に準拠。
# 単一 LiDAR なので target_frame=laser_link とし TF を不要にする。
echo "[meas06] 知覚パイプライン起動"
setsid ros2 run th_perception lidar_filter.py > /tmp/lidar_filter.log 2>&1 &
FILT=$!
setsid ros2 launch leg_detection_bringup leg_detection.launch.py \
  scan_topic:=/scan_filtered target_frame:=laser_link \
  scan_frame:=laser_link odom_frame:=laser_link \
  use_rviz:=false autostart:=true > /tmp/leg.log 2>&1 &
LEG=$!
setsid ros2 run th_perception person_tracker_bridge.py > /tmp/bridge.log 2>&1 &
PTB=$!
echo "[meas06] 起動待ち 25 秒（DR-SPAAM の重み読み込みに時間がかかる）"
sleep 25

# 追従対象の指定（require_explicit_target_selection: true のため自発取得しない）
SEL="$(ros2 service list 2>/dev/null | grep -m1 select_target || true)"
if [ -n "$SEL" ]; then
  echo "[meas06] 追従対象を正面 1.5m に指定: $SEL"
  timeout 10 ros2 service call "$SEL" \
    multiple_sensor_person_tracking/srv/SelectTarget \
    "{candidate_index: -1, x: 1.5, y: 0.0}" 2>&1 | tail -2
  sleep 3
else
  echo "[meas06] 警告: select_target サービスが見つからない。is_lost のままかもしれない" >&2
fi

# ── 測定 ──────────────────────────────────────────────────────
measure() {  # $1=種別(hz|delay) $2=トピック
  local v
  # パイプに流すと Python が block-buffer するので PYTHONUNBUFFERED が要る。
  # SIGTERM だと ros2 CLI が途中出力を捨てることがあるため -s INT で止める。
  v="$(PYTHONUNBUFFERED=1 timeout -s INT "$DUR" ros2 topic "$1" "$2" 2>/dev/null \
       | grep -oP "average (rate|delay): \K[\d.]+" | tail -1)"
  echo "${v:-—}"
}

{
  echo "===================================================================="
  echo "■ 知覚経路の遅延（$(date '+%F %T')、各 ${DUR} 秒）"
  echo "===================================================================="
  echo
  printf '%-42s %10s %10s\n' 'トピック' '頻度[Hz]' '遅延[s]'
  printf '%-42s %10s %10s\n' '------------------------------------------' '--------' '-------'
  for t in /scan /scan_filtered /dr_spaam/dr_spaam_detections /person/status; do
    printf '%-42s %10s %10s\n' "$t" "$(measure hz "$t")" "$(measure delay "$t")"
  done
  echo
  echo "【読み方】"
  echo " ・/dr_spaam/... が 2Hz 前後なら、位置更新は 500ms に 1 回。"
  echo "   1m/s で走ると更新の合間に機体は 0.5m 進む（蛇行の主因の候補）。"
  echo " ・/person/status の遅延 ≫ /scan の遅延 なら、上流のタイムスタンプが"
  echo "   保たれており、その差が知覚経路そのものの遅延。"
  echo " ・/person/status の遅延 ≒ /scan の遅延 なら、途中でスタンプが打ち直され"
  echo "   ている＝この方法では遅延を測れない（別途 §4.4 の手当てが要る）。"
} | tee "$REPORT"

# ── 停止 ──────────────────────────────────────────────────────
echo
echo "[meas06] 停止処理"
for p in $PTB $LEG $FILT; do kill -TERM -$p 2>/dev/null; kill -TERM $p 2>/dev/null; done
sleep 3
echo "[meas06] 完了 → $REPORT"
