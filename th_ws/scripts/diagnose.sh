#!/usr/bin/env bash
# ============================================================
# diagnose.sh — TH システム 稼働診断スクリプト
#
# docs/operation.md の「トラブルシューティング」節と
# docs/network.md の「復旧手順」で手動で1個ずつ実行していた
# 確認コマンド群をまとめて実行し、PASS/FAIL/WARN で一覧表示する。
#
# 実行場所: th_robot コンテナ内 (docker exec -it th_robot bash)
#   cd /root/th_ws && bash scripts/diagnose.sh
#
# 個々のチェックが失敗してもスクリプト全体を止めず、全項目を
# 実行してから最後にサマリを出すため set -e は使わない。
# ============================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

set +u
source /opt/ros/humble/setup.bash
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash"
set -u

# ── オプション解析 ────────────────────────────────────────
RUN_NETWORK=true

for arg in "$@"; do
  case "$arg" in
    --no-network) RUN_NETWORK=false ;;
    --help)
      echo "使い方: $0 [--no-network]"
      echo "  --no-network  ネットワーク層チェック (ping/ポート確認) をスキップする"
      echo "                (AP に接続していない開発機での実行用)"
      exit 0
      ;;
  esac
done

# ── PASS/FAIL/WARN カウンタ ──────────────────────────────
PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0
declare -a FAIL_HINTS=()
declare -a WARN_HINTS=()

pass() {
  printf "  [PASS] %s\n" "$1"
  PASS_COUNT=$((PASS_COUNT + 1))
}

fail() {
  printf "  [FAIL] %s\n" "$1"
  FAIL_COUNT=$((FAIL_COUNT + 1))
  FAIL_HINTS+=("$1 — $2")
}

warn() {
  printf "  [WARN] %s\n" "$1"
  WARN_COUNT=$((WARN_COUNT + 1))
  WARN_HINTS+=("$1 — $2")
}

echo "=================================================="
echo " TH システム 稼働診断"
echo "=================================================="

# ── 1. ROS2 ノード稼働確認 ───────────────────────────────
echo ""
echo "[1] ROS2 ノード"
NODE_LIST="$(timeout 5 ros2 node list 2>/dev/null || true)"

if [ -z "$NODE_LIST" ]; then
  fail "ros2 node list" "ノードが1件も見つからない。bringup.launch.py が起動しているか確認 (docs/operation.md の『ロボットが動かない』)"
else
  for n in safety_monitor mode_manager esp32_bridge; do
    if echo "$NODE_LIST" | grep -qx "/$n"; then
      pass "ノード /$n 起動中"
    else
      fail "ノード /$n 未検出" "docs/operation.md の『ロボットが動かない』を参照。該当ノードが launch されているか確認"
    fi
  done
fi

# ── 2. 安全チェーン ──────────────────────────────────────
echo ""
echo "[2] 安全チェーン"

ESTOP_RAW="$(timeout 3 ros2 topic echo /safety/estop --once 2>/dev/null || true)"
if [ -z "$ESTOP_RAW" ]; then
  fail "/safety/estop 取得不可" "safety_monitor が起動していない可能性 (上記[1]参照)"
elif echo "$ESTOP_RAW" | grep -q "data: true"; then
  warn "/safety/estop = true (E-Stop 作動中)" "物理E-Stop/タブレット緊急停止のどちらかが有効。docs/operation.md の『モード早見表 / フォルト対応』を参照"
else
  pass "/safety/estop = false"
fi

FAULT_RAW="$(timeout 3 ros2 topic echo /safety/fault --once 2>/dev/null || true)"
if [ -z "$FAULT_RAW" ]; then
  fail "/safety/fault 取得不可" "safety_monitor が起動していない可能性 (上記[1]参照)"
elif echo "$FAULT_RAW" | grep -q "active: true"; then
  FAULT_TYPE="$(echo "$FAULT_RAW" | grep -oE "fault_type:.*" | sed "s/fault_type: *//;s/[\"']//g")"
  case "$FAULT_TYPE" in
    LIDAR_LOST)
      warn "フォルト発生中: LIDAR_LOST" "docs/network.md の『/scan がコンテナに届かない』を参照" ;;
    ESP32_DISCONNECTED)
      warn "フォルト発生中: ESP32_DISCONNECTED" "docs/network.md の『ESP32 が WS に繋がらない』を参照" ;;
    PERSON_TRACKER_LOST)
      warn "フォルト発生中: PERSON_TRACKER_LOST" "docs/operation.md の『脚検知 (/person/status) が来ない』を参照" ;;
    *)
      warn "フォルト発生中: ${FAULT_TYPE:-不明}" "docs/operation.md の『モード早見表 / フォルト対応』を参照" ;;
  esac
else
  pass "/safety/fault = 発生なし"
fi

# ── 3. モード ────────────────────────────────────────────
echo ""
echo "[3] モード"

declare -A MODE_NAMES=(
  [0]="INIT" [1]="IDLE" [2]="FOLLOWING" [3]="MOVING_TO_PANEL"
  [4]="AT_PANEL" [5]="MANUAL" [6]="ESTOP" [7]="FOLLOWING_MAPLESS"
)

MODE_RAW="$(timeout 3 ros2 topic echo /robot/mode --once --qos-durability transient_local 2>/dev/null || true)"
if [ -z "$MODE_RAW" ]; then
  fail "/robot/mode 取得不可" "mode_manager が起動していない可能性 (上記[1]参照)"
else
  MODE_NUM="$(echo "$MODE_RAW" | grep -oE "mode: [0-9]+" | awk '{print $2}')"
  MODE_NAME="${MODE_NAMES[$MODE_NUM]:-不明 ($MODE_NUM)}"
  if [ "$MODE_NUM" = "6" ]; then
    warn "/robot/mode = $MODE_NAME ($MODE_NUM)" "ESTOP 中。上記[2]の estop/fault 原因を解消後、IDLE への遷移操作が必要"
  else
    pass "/robot/mode = $MODE_NAME ($MODE_NUM)"
  fi
fi

# ── 4. トピック生存確認 ──────────────────────────────────
# safety_monitor.yaml の実タイムアウト (lidar/esp32=2000ms, person=2500ms) より
# 長い待ち時間をとり、その間に1件でも届けば PASS とする。実測レートは参考表示のみ
# (WiFi ジッタで一時的にレートが落ちるのは safety_monitor 設計上も正常のため)。
echo ""
echo "[4] トピック生存確認 (レートは参考値、閾値判定はしない)"

check_topic_alive() {
  local topic="$1" hint="$2"
  local hz_out
  hz_out="$(timeout 3.5 ros2 topic hz "$topic" --window 20 2>/dev/null | grep "average rate" || true)"
  if [ -n "$hz_out" ]; then
    pass "$topic 受信中 ($(echo "$hz_out" | head -1 | xargs))"
  else
    fail "$topic 受信なし" "$hint"
  fi
}

check_topic_alive "/scan" "docs/network.md の『/scan がコンテナに届かない』を参照"
check_topic_alive "/esp32/wheel_feedback" "docs/network.md の『ESP32 が WS に繋がらない』を参照"
check_topic_alive "/person/status" "docs/operation.md の『脚検知 (/person/status) が来ない』を参照"

# ── 5. ネットワーク層 ────────────────────────────────────
if [ "$RUN_NETWORK" = true ]; then
  echo ""
  echo "[5] ネットワーク層"
  echo "    (Windows 側の WiFi 再接続・portproxy 確認等は対象外。"
  echo "     docs/network.md の該当手順を参照すること)"

  if ! command -v ping >/dev/null 2>&1; then
    warn "ping コマンドが未導入のため疎通確認をスキップ" "Dockerfile に iputils-ping が入っているか確認し、docker compose build し直す"
  else
    if ping -c 2 -W 1 192.168.4.1 >/dev/null 2>&1; then
      pass "ESP32 AP (192.168.4.1) 到達可能"
    else
      fail "ESP32 AP (192.168.4.1) 到達不可" "docs/network.md の『PC が AP に繋がらない』を参照"
    fi

    PI_FOUND=""
    for last in 2 3 4 5; do
      ip="192.168.4.$last"
      if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
        PI_FOUND="$ip"
        break
      fi
    done
    if [ -n "$PI_FOUND" ]; then
      pass "ラズパイ候補 ($PI_FOUND) 到達可能"
    else
      fail "ラズパイ (192.168.4.2〜.5) 到達不可" "docs/network.md の『ラズパイに ssh できない』を参照"
    fi
  fi

  if command -v ss >/dev/null 2>&1; then
    if ss -tln 2>/dev/null | grep -q ":8766 "; then
      pass "esp32_bridge WS ポート 8766 LISTEN 中"
    else
      fail "8766 番ポート未 LISTEN" "docs/network.md の『ESP32 が WS に繋がらない』3. を参照 (esp32_bridge 未起動 or 多重起動で bind 失敗)"
    fi
  else
    warn "ss コマンドが無く 8766 番ポート確認をスキップ" "コンテナに iproute2 が入っているか確認"
  fi

  if [ "${ROS_DOMAIN_ID:-}" = "10" ]; then
    pass "ROS_DOMAIN_ID = 10"
  else
    fail "ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-未設定}" "docs/network.md の『/scan がコンテナに届かない』3. を参照 (ROS_DOMAIN_ID=10 と不一致)"
  fi
else
  echo ""
  echo "[5] ネットワーク層 — --no-network によりスキップ"
fi

# ── 6. サマリ ────────────────────────────────────────────
echo ""
echo "=================================================="
echo " サマリ: PASS=$PASS_COUNT  WARN=$WARN_COUNT  FAIL=$FAIL_COUNT"
echo "=================================================="

if [ "${#WARN_HINTS[@]}" -gt 0 ]; then
  echo ""
  echo "WARN 項目:"
  for h in "${WARN_HINTS[@]}"; do
    echo "  - $h"
  done
fi

if [ "${#FAIL_HINTS[@]}" -gt 0 ]; then
  echo ""
  echo "FAIL 項目:"
  for h in "${FAIL_HINTS[@]}"; do
    echo "  - $h"
  done
fi

if [ "$FAIL_COUNT" -gt 0 ]; then
  exit 1
fi
exit 0
