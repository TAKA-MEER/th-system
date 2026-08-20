#!/usr/bin/env bash
# 手順書 §4.3 の 2 — ESP32 の受信ギャップの 3 択切り分け
#
#   A. 電線上もギャップ ＋ TCP 再送あり  → 伝送方式（TCP）。AP では直らない
#   B. 電線上もギャップ ＋ 再送なし      → ESP32 が送っていない（ファーム）
#   C. 電線上はギャップ無し（ROS だけ）  → esp32_bridge（Python）の詰まり
#
# ★ コンテナの中で実行する（host network ＋ privileged なので sudo は要らない）:
#     docker compose run --rm th_robot bash scripts/meas05_esp32_triage.sh
#
#   引数 1: 記録秒数（既定 1200 ＝ 20 分）
#   引数 2: 跳ねとみなす閾値 ms（既定 250）
#
# 車輪は回さない（/cmd_vel は 0）。通電も機体の固定も不要。
set -u
DUR="${1:-1200}"
THRESH="${2:-250}"
# 構成によって変わる。ラズパイ AP 構成では ESP32 は子機 (DHCP)、
# 中継するのはラズパイ。ping 先は ESP32 本体と、経路上の親機の 2 つ。
ESP32_IP="${ESP32_IP:-192.168.4.1}"
RELAY_IP="${RELAY_IP:-192.168.4.2}"
OUT_DIR=/root/th_data/meas05
# /scan の購読に使う FastDDS プロファイル。既定はラズパイ無線 (192.168.4.2)。
# 有線経由に切り替えて比べる場合は FASTDDS_PROFILE=... で上書きする。
FASTDDS_PROFILE="${FASTDDS_PROFILE:-/root/th_ws/src/th_bringup/config/fastdds_profile.xml}"
STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"

# ── 前提の確認 ────────────────────────────────────────────────
IFACE="$(ip route get "$ESP32_IP" 2>/dev/null | grep -oP 'dev \K\S+')"
if [ -z "$IFACE" ]; then
  echo "[meas05] ESP32 ($ESP32_IP) への経路が無い。AP に接続してから再実行する" >&2
  exit 1
fi
echo "[meas05] ESP32 経路: $IFACE"
ip -br addr show "$IFACE"

if ! command -v tcpdump > /dev/null; then
  echo "[meas05] tcpdump を導入中..."
  apt-get update -qq && apt-get install -y -qq tcpdump || {
    echo "[meas05] tcpdump を導入できなかった" >&2; exit 1; }
fi

# ── ビルド（CLAUDE.md: ビルドと使用は同一コンテナで完結させる） ──
cd /root/th_ws
# 非対話シェルは .bashrc を読まないので ROS は自分で source する（CLAUDE.md）。
# ROS の setup.bash は未定義変数を参照するので set -u を一時的に外す
# （AMENT_TRACE_SETUP_FILES: unbound variable で即死する）。
set +u; source /opt/ros/humble/setup.bash; set -u
echo "[meas05] ビルド中..."
colcon build --symlink-install > /tmp/build.log 2>&1 || { tail -20 /tmp/build.log; exit 1; }
set +u; source install/setup.bash; set -u

# ── 記録開始 ──────────────────────────────────────────────────
PCAP="$OUT_DIR/${STAMP}.pcap"
ARRIVALS="$OUT_DIR/${STAMP}_arrivals.csv"
DUMPTXT="$OUT_DIR/${STAMP}_tcpdump.txt"
REPORT="$OUT_DIR/${STAMP}_verdict.txt"

echo "[meas05] tcpdump 開始 → $PCAP"
setsid tcpdump -i "$IFACE" -n -s 96 -w "$PCAP" 'tcp port 8766' \
  > /tmp/tcpdump.log 2>&1 &
TCPD=$!
sleep 2

# ESP32 は AP でもあるので、ラズパイの /scan も ESP32 の中継を通る。
# 2026-08-20 の 1 本目で「ESP32 と LiDAR の跳ねが 3 回とも同時刻」だったため、
# 原因が ESP32 の無線タスクにあるのか受信端にあるのかを分ける必要が出た。
#   .1 = ESP32 自身（LwIP が返す。アプリが止まっていても返る）
#   .2 = ラズパイ（ESP32 の中継を通る）
echo "[meas05] ping を並走させる（.1=ESP32本体 / .2=ラズパイ中継）"
setsid ping -i 0.1 -D "$ESP32_IP" > "$OUT_DIR/${STAMP}_ping_esp32.txt" 2>&1 &
PING1=$!
setsid ping -i 0.1 -D "$RELAY_IP" > "$OUT_DIR/${STAMP}_ping_rpi.txt" 2>&1 &
PING2=$!

echo "[meas05] ノード起動"
setsid ros2 run th_esp32_bridge esp32_bridge.py --ros-args \
  --params-file src/th_esp32_bridge/config/params.yaml > /tmp/bridge.log 2>&1 &
BRIDGE=$!
# 記録器は /scan も購読する。ラズパイ側なのでユニキャスト初期ピアの
# プロファイルが要る（無いと lidar 行が空になる）。
setsid env FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE" \
  python3 scripts/meas05_arrival_recorder.py "$ARRIVALS" \
  > /tmp/recorder.log 2>&1 &
REC=$!
# 実運用と同じ上り負荷を掛ける。車輪は回さない（linear.x = 0）
setsid ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1 &
CMDVEL=$!

sleep 15
if ! grep -q "ESP32 接続" /tmp/bridge.log 2>/dev/null; then
  echo "[meas05] 警告: ESP32 がまだ接続してきていない。ログ:" >&2
  tail -5 /tmp/bridge.log >&2
fi

echo "[meas05] 記録中: ${DUR} 秒（$(date +%H:%M:%S) 開始）"
sleep "$DUR"

# ── 停止 ──────────────────────────────────────────────────────
echo "[meas05] 停止処理"
for p in $CMDVEL $REC $BRIDGE $PING1 $PING2; do kill -TERM -$p 2>/dev/null; kill -TERM $p 2>/dev/null; done
sleep 2
kill -TERM -$TCPD 2>/dev/null; kill -TERM $TCPD 2>/dev/null
sleep 1

# ── 解析 ──────────────────────────────────────────────────────
echo "[meas05] pcap を展開 → $DUMPTXT"
tcpdump -r "$PCAP" -tt -n > "$DUMPTXT" 2> /dev/null

python3 scripts/meas05_analyze.py "$ARRIVALS" "$DUMPTXT" "$THRESH" | tee "$REPORT"

echo
echo "[meas05] 完了。出力:"
echo "  判定    $REPORT"
echo "  ROS到着 $ARRIVALS"
echo "  電線上  $PCAP / $DUMPTXT"
