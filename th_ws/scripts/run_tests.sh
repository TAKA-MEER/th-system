#!/usr/bin/env bash
# ============================================================
# run_tests.sh — TH システム テスト実行スクリプト
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_ROOT="$SCRIPT_DIR"

source /opt/ros/humble/setup.bash

echo "=================================================="
echo " TH システム テスト実行"
echo "=================================================="

# ── オプション解析 ────────────────────────────────────────
RUN_UNIT=true
RUN_INTEGRATION=false
RUN_SIM=false

for arg in "$@"; do
  case "$arg" in
    --all)         RUN_UNIT=true; RUN_INTEGRATION=true ;;
    --integration) RUN_INTEGRATION=true ;;
    --sim)         RUN_SIM=true ;;
    --unit-only)   RUN_INTEGRATION=false; RUN_SIM=false ;;
    --help)
      echo "使い方: $0 [--all] [--integration] [--sim] [--unit-only]"
      echo "  --all         全テスト実行（単体 + 統合）"
      echo "  --integration 統合テストも実行（ROS2 ノード必要）"
      echo "  --sim         シミュレーションテストも実行（Gazebo 不要版）"
      echo "  --unit-only   純粋単体テストのみ（デフォルト）"
      exit 0
      ;;
  esac
done

# ── ビルド ────────────────────────────────────────────────
echo ""
echo "[1/3] ビルド中..."
cd "$WS_ROOT"
colcon build \
  --symlink-install \
  --packages-select \
    th_system_msgs \
    th_esp32_bridge \
    th_safety \
    th_mode_manager \
    th_perception \
    th_planning \
    th_calibration \
    th_bringup \
    th_testing \
  2>&1 | grep -E "(Building|Finished|Failed|error:|warning:)" || true

source install/setup.bash

# ── 純粋単体テスト（ROS2 不要）────────────────────────────
if [ "$RUN_UNIT" = true ]; then
  echo ""
  echo "[2/3] 純粋単体テスト (pytest)..."
  python3 -m pytest \
    src/th_testing/test/test_follow_planner_logic.py \
    -v \
    --tb=short \
    --color=yes \
    2>&1
  echo "      ✓ 純粋単体テスト完了"
fi

# ── ROS2 統合テスト ───────────────────────────────────────
if [ "$RUN_INTEGRATION" = true ]; then
  echo ""
  echo "[3/3] ROS2 統合テスト (colcon test)..."

  export TH_SKIP_SIM=1
  if [ "$RUN_SIM" = true ]; then
    export TH_SKIP_SIM=0
    echo "      シミュレーションテスト有効 (TH_SKIP_SIM=0)"
  fi

  colcon test \
    --packages-select th_testing \
    --event-handlers console_direct+ \
    --return-code-on-test-failure \
    2>&1

  colcon test-result --verbose
fi

echo ""
echo "=================================================="
echo " テスト完了"
echo "=================================================="
