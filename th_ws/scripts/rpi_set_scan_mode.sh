#!/usr/bin/env bash
# ラズパイの rplidar サービスの scan_mode を切り替える（要 sudo・ラズパイ上で実行）
#
#   sudo bash rpi_set_scan_mode.sh Standard    # 推奨（docs/setup.md）
#   sudo bash rpi_set_scan_mode.sh DenseBoost  # 元に戻す
#
# 背景: 稼働中の unit は `ros2 launch rplidar_ros rplidar_s1_launch.py` を叩いており、
# この launch ファイルは scan_mode を**引数として宣言していない**ため指定できない
# （実機確認 2026-08-20）。docs/setup.md が記載している `ros2 run` 形式に置き換える。
#
# docs/setup.md より:
#   「既定の DenseBoost は点数が多く DR-SPAAM の CPU 推論が約2Hz まで落ちて
#     歩行者を見失いやすい。Standard(点数半減)で追跡が安定する。」
set -euo pipefail
MODE="${1:?Standard か DenseBoost を指定する}"
case "$MODE" in Standard|DenseBoost|Boost|Sensitivity) ;; *) echo "不明なモード: $MODE" >&2; exit 1;; esac

UNIT=/etc/systemd/system/rplidar.service
PORT="${PORT:-/dev/ttyUSB0}"
BACKUP="${UNIT}.bak.$(date +%Y%m%d_%H%M%S)"

[ -f "$UNIT" ] || { echo "$UNIT が無い" >&2; exit 1; }
cp -a "$UNIT" "$BACKUP"
echo "[rpi] バックアップ: $BACKUP"

# ExecStart 行だけを差し替える。他の行（Environment 等）は触らない。
NEW_EXEC='ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && source /home/mirs2602/ros2_ws/install/setup.bash && exec ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:='"$PORT"' -p serial_baudrate:=256000 -p frame_id:=laser_link -p angle_compensate:=true -p scan_mode:='"$MODE"'"'
python3 - "$UNIT" "$NEW_EXEC" <<'PY'
import sys
path, new = sys.argv[1], sys.argv[2]
lines = open(path).read().split('\n')
out, done = [], False
for l in lines:
    if l.startswith('ExecStart='):
        out.append(new); done = True
    else:
        out.append(l)
if not done:
    sys.exit('ExecStart= の行が見つからない')
open(path, 'w').write('\n'.join(out))
print('[rpi] ExecStart を書き換えた')
PY

systemctl daemon-reload
systemctl restart rplidar
echo "[rpi] 再起動した。起動を待つ..."
sleep 12

echo "[rpi] === 実際の scan mode ==="
if journalctl -u rplidar --since "-40 seconds" --no-pager 2>/dev/null | grep -o "current scan mode: [A-Za-z]*.*" | tail -1; then :; else
  echo "[rpi] ログから読めなかった。手動で確認する:"
  echo "  journalctl -u rplidar -n 30 --no-pager | grep 'scan mode'"
fi
systemctl is-active rplidar
echo
echo "[rpi] 元に戻す場合: sudo cp -a $BACKUP $UNIT && sudo systemctl daemon-reload && sudo systemctl restart rplidar"
