#!/usr/bin/env bash
# ============================================================
# setup.sh — TH システム 導入（1 台につき 1 回）
#
# 仕様: docs/plan/spec/Spec-ops.md §1
#
# PC 側の導入だけを行う。LiDAR も ESP32 も PC には繋がない
# （2026-09-05 以降どちらもラズパイ側）ため、PC への udev ルール
# 導入は行わない（udev/99-th-robot.rules は直結構成に戻す場合の
# 手順として残す。docs/setup.md §9）。
# ラズパイ側の導入（pi_serial_relay の systemd 登録）は sudo の
# パスワード入力が要り非対話ではできないため、案内だけにする。
# PC のロボット回線（th-rpi-ap-wlo1）の設定も自動では書き換えず、
# 存在の確認と案内だけにする。詳しくは docs/network.md。
#
# 冪等（2 回実行しても壊れない）。
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --help|-h)
            echo "使い方: ./setup.sh [--dry-run]"
            echo "  導入（PC 側）を 1 回で行う。2 回目以降の実行も壊れない。"
            echo "  --dry-run は何も実行せず、何をするかだけ表示する。"
            exit 0
            ;;
        *) echo "[setup.sh] エラー: 未知のオプション: $arg（--help を参照）" >&2; exit 2 ;;
    esac
done

if [ "$DRY_RUN" -eq 1 ]; then
    cat <<DRY
[setup.sh] --dry-run: 何も実行しない。実行時の手順は次のとおり:
  1. docker compose build（th_ws/ で）
  2. コンテナ内で colcon build --symlink-install（無ければ起動してから。ビルドからまとめて 1 回で）
  3. WebUI: npm ci → npm run build（本番ビルド）
  4. 実行時データの置き場 th_ws/data/ が無ければ作る
  5. PlatformIO の有無を確認する（ESP32 の書き込みに要る。書き込みは docs/esp32.md）
  6. PC のロボット回線（th-rpi-ap-wlo1）の存在を確認する（無ければ docs/network.md を案内。自動では書き換えない）
  7. ラズパイ側の導入は行わず docs/network.md の該当節を案内する
  次のステップ: ./start.sh を実行する
DRY
    exit 0
fi

echo "=== TH システム セットアップ（導入・1 台につき 1 回）==="

# ── 1. Docker イメージ ──────────────────────────────────────
echo "[1/6] Docker イメージをビルド..."
(cd "$SCRIPT_DIR" && docker compose build)
echo "      完了"

# ── 2. ROS2 ワークスペース ──────────────────────────────────
# ビルドからまとめて 1 回の bash -lc で行う。分けるとテスト側から
# ビルド成果が見えない構成があるため（CLAUDE.md「環境の癖」）。
echo "[2/6] コンテナ内で colcon build..."
if [ "$(docker inspect -f '{{.State.Running}}' th_robot 2>/dev/null || echo missing)" != "true" ]; then
    (cd "$SCRIPT_DIR" && docker compose up -d th_robot)
fi
docker exec th_robot bash -lc 'source /opt/ros/humble/setup.bash && cd /root/th_ws && colcon build --symlink-install'
echo "      完了"

# ── 3. Web UI ───────────────────────────────────────────────
echo "[3/6] Web UI をセットアップ..."
(cd "$SCRIPT_DIR/web_ui" && npm ci && npm run build)
echo "      完了（本番ビルド dist/ を作成）"

# ── 4. 実行時データの置き場 ─────────────────────────────────
echo "[4/6] 実行時データの置き場を確認..."
mkdir -p "$SCRIPT_DIR/data"
echo "      完了（th_ws/data/。コンテナの /root/th_data）"

# ── 5. PlatformIO (ESP32) ───────────────────────────────────
echo "[5/6] PlatformIO のセットアップ確認..."
if command -v pio &>/dev/null; then
    echo "      PlatformIO 検出済み"
    echo "      ESP32 ビルド: cd esp32 && pio run"
    echo "      ESP32 書込み: cd esp32 && pio run --target upload（手順は docs/esp32.md）"
else
    echo "      PlatformIO が見つかりません（ESP32 の書き込みに要る）"
    echo "      インストール: pip install platformio"
    echo "      または VS Code 拡張 'PlatformIO IDE' をインストールしてください"
    echo "      書き込み手順は docs/esp32.md"
fi

# ── 6. PC のロボット回線 ────────────────────────────────────
# nmcli の自動書き換えはしない。存在の確認と案内だけ。
echo "[6/6] PC のロボット回線を確認..."
if nmcli connection show th-rpi-ap-wlo1 &>/dev/null; then
    echo "      th-rpi-ap-wlo1 あり（固定 192.168.5.50・省電力 OFF であること）"
else
    echo "      th-rpi-ap-wlo1 が見つかりません。自動では設定しません。"
    echo "      docs/network.md を参照して設定してください。"
fi

echo ""
echo "=== セットアップ完了 ==="
echo ""
echo "ラズパイ側の導入（pi_serial_relay の systemd 登録）はこのスクリプトでは"
echo "行いません。docs/network.md の該当節を参照してください。"
echo ""
echo "【次のステップ】"
echo "  ./start.sh を実行する"
