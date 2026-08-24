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
# 非対話シェルは .bashrc を読まないので ROS は自分で source する（CLAUDE.md）。
# ROS の setup.bash は未定義変数を参照するので set -u を一時的に外す
# （AMENT_TRACE_SETUP_FILES: unbound variable で即死する）。
set +u; source /opt/ros/humble/setup.bash; set -u
echo "[meas06] ビルド中..."
colcon build --symlink-install > /tmp/build.log 2>&1 || { tail -20 /tmp/build.log; exit 1; }
set +u; source install/setup.bash; set -u

# ラズパイ (192.168.4.2) の /scan は ESP32 SoftAP 越しで、マルチキャストの
# ディスカバリが通らない（実機検証 2026-07-11）。ユニキャスト初期ピアの
# プロファイルが要る。bringup.launch.py も同じものを additional_env で
# 「リモート /scan を購読するノードにだけ」渡している。コンテナ全体に効かせると
# ローカル発見が不安定になる事象が報告されているため、ここでも同じ流儀にする。
FASTDDS_PROFILE=/root/th_ws/src/th_bringup/config/fastdds_profile.xml

# /scan の到達確認。リトライするのは、時計がずれていると発見が断続的に失敗する
# ため（2026-08-20: 7 秒ずれていた時は 7〜10 割、時計を直したら 5/5 になった）。
# 時刻同期が入った今は基本 1 回で決まるが、保険として残す。
# echo ではなく list で見る（/scan は sensor_data QoS = BEST_EFFORT で、
# ros2 topic echo は既定 RELIABLE のためマッチせず誤判定する）。
echo "[meas06] /scan の到達を確認中（ラズパイ側 LiDAR）..."
FOUND=0
for i in 1 2 3 4 5; do
  if ( export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
       timeout 30 ros2 topic list --no-daemon 2>/dev/null | grep -q "^/scan$" ); then
    FOUND=1; echo "[meas06] /scan 確認 (試行 $i)"; break
  fi
  echo "[meas06]   試行 $i: まだ見えない"
done
if [ "$FOUND" -eq 0 ]; then
  echo "[meas06] /scan が 5 回とも見えなかった。ラズパイ側を確認する:" >&2
  echo "  ssh mirs2602@192.168.4.2 'systemctl is-active rplidar; date'" >&2
  exit 1
fi

# ── 知覚パイプラインだけを起動（Nav2・安全系は起動しない） ──────
# 構成は docs/architecture.md「person_tracker だけを素早く動作確認する」に準拠。
# 単一 LiDAR なので target_frame=laser_link とし TF を不要にする。
echo "[meas06] 知覚パイプライン起動"
# lidar_filter だけがリモート /scan を購読するので、プロファイルはこれに渡す
setsid env FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE" \
  ros2 run th_perception lidar_filter.py > /tmp/lidar_filter.log 2>&1 &
FILT=$!
setsid ros2 launch leg_detection_bringup leg_detection.launch.py \
  scan_topic:=/scan_filtered target_frame:=laser_link \
  scan_frame:=laser_link odom_frame:=laser_link \
  use_rviz:=false autostart:=true > /tmp/leg.log 2>&1 &
LEG=$!
setsid ros2 run th_perception person_tracker_bridge.py > /tmp/bridge.log 2>&1 &
PTB=$!
echo "[meas06] 起動待ち 45 秒（DR-SPAAM の重み読み込みに時間がかかる）"
sleep 45

# 追従対象の指定。require_explicit_target_selection: true なので自発取得しない。
# 指定しないと following_position が出ず、person_tracker_bridge の _last_stamp が
# None のままになり、/person/status のスタンプが自ノードの now() に落ちる
# （＝ topic delay が 0 になり遅延を測れない）。実測 2026-08-20 で踏んだ。
SEL=""
for i in 1 2 3; do
  SEL="$(timeout 25 ros2 service list --no-daemon 2>/dev/null | grep -m1 select_target || true)"
  [ -n "$SEL" ] && break
  echo "[meas06]   select_target 探索 試行 $i: まだ見えない"
done
if [ -n "$SEL" ]; then
  echo "[meas06] 追従対象を正面 1.5m に指定: $SEL"
  timeout 15 ros2 service call "$SEL" \
    multiple_sensor_person_tracking/srv/SelectTarget \
    "{candidate_index: -1, x: 1.5, y: 0.0}" 2>&1 | tail -2
  sleep 5
else
  echo "[meas06] 警告: select_target が見つからない。診断:" >&2
  timeout 20 ros2 node list --no-daemon 2>&1 | tail -8 >&2
  tail -15 /tmp/leg.log >&2
fi

# ── 測定 ──────────────────────────────────────────────────────
measure() {  # $1=種別(hz|delay) $2=トピック
  local v
  # パイプに流すと Python が block-buffer するので PYTHONUNBUFFERED が要る。
  # SIGTERM だと ros2 CLI が途中出力を捨てることがあるため -s INT で止める。
  local env_pfx=""
  # /scan はリモート（ラズパイ）なのでプロファイルが要る
  [ "$2" = "/scan" ] && env_pfx="FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_PROFILE"
  v="$(env $env_pfx PYTHONUNBUFFERED=1 timeout -s INT "$DUR" ros2 topic "$1" "$2" 2>/dev/null \
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
