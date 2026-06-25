#!/usr/bin/env bash
# ============================================================
# setup.sh — TH システム 初回セットアップ
# ============================================================
set -euo pipefail

echo "=== TH システム セットアップ ==="

# ── udev ルール ────────────────────────────────────────────
echo "[1/5] udev ルールを適用..."
sudo cp udev/99-th-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "      完了 (デバイス接続後に /dev/lidar, /dev/esp32 が現れます)"

# ── Docker イメージビルド ───────────────────────────────────
echo "[2/5] Docker イメージをビルド..."
docker compose build
echo "      完了"

# ── Web UI 依存パッケージ ───────────────────────────────────
echo "[3/5] Web UI (npm) をセットアップ..."
cd web_ui && npm install && cd ..
echo "      完了"

# ── maps ディレクトリ ───────────────────────────────────────
echo "[4/5] maps ディレクトリを作成..."
mkdir -p src/th_bringup/maps
echo "      完了 (SLAM 後に map_saver で保存してください)"

# ── PlatformIO (ESP32) ─────────────────────────────────────
echo "[5/5] PlatformIO のセットアップ確認..."
if command -v pio &>/dev/null; then
    echo "      PlatformIO 検出済み"
    echo "      ESP32 ビルド: cd esp32 && pio run"
    echo "      ESP32 書込み: cd esp32 && pio run --target upload"
else
    echo "      PlatformIO が見つかりません"
    echo "      インストール: pip install platformio"
    echo "      または VS Code 拡張 'PlatformIO IDE' をインストールしてください"
fi

echo ""
echo "=== セットアップ完了 ==="
echo ""
echo "【次のステップ】"
echo "  1. ESP32 を接続: cd esp32 && pio run -t upload"
echo "  2. RPLIDAR を接続"
echo "  3. 地図作成: docker compose run --rm th_robot ros2 launch th_bringup slam.launch.py"
echo "  4. 地図保存: ros2 run nav2_map_server map_saver_cli -f ~/maps/th_map"
echo "  5. キャリブ: ros2 run th_calibration linear_calib.py"
echo "  6. フル起動: docker compose run --rm th_robot ros2 launch th_bringup bringup.launch.py"
echo "  7. Web UI  : cd web_ui && npm run dev"
echo "  8. タブレットブラウザで http://<ロボットPCのIP>:5173 を開く"
