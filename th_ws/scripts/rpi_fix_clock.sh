#!/usr/bin/env bash
# ラズパイの時計を恒久的に直す（要 sudo・ラズパイ上で実行）
#   ★ 先に PC 側で pc_setup_ntp_server.sh を実行しておくこと
#
#   sudo bash rpi_fix_clock.sh
#
# やること:
#   1. systemd-time-wait-sync を有効化する（既定で disabled だった）
#   2. rplidar.service が time-sync.target を待ってから起動するようにする
#
# なぜ 2 が要るか（実機で踏んだ）:
#   ラズパイには RTC が無く、起動直後の時計は過去（2026-08-17）に戻っている。
#   NTP が合わせるまで数十秒かかるが、その間に rplidar が起動してしまうと
#   **DDS のディスカバリが成立しない**。パケットは PC との間を双方向に流れて
#   いるのに、ノードもトピックも一切見えない状態になる（2026-08-20 に 2 回遭遇）。
#   時計を合わせただけでは既に起動済みのノードは直らないので、順序を強制する。
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "sudo で実行する" >&2; exit 1; }

UNIT=/etc/systemd/system/rplidar.service
[ -f "$UNIT" ] || { echo "$UNIT が無い" >&2; exit 1; }

echo "[rpi] 1. systemd-time-wait-sync を有効化"
systemctl enable systemd-time-wait-sync
systemctl restart systemd-timesyncd

echo "[rpi] 2. rplidar.service に time-sync.target 待ちを追加"
BACKUP="${UNIT}.bak.$(date +%Y%m%d_%H%M%S)"
cp -a "$UNIT" "$BACKUP"
echo "[rpi]    バックアップ: $BACKUP"
python3 - "$UNIT" <<'PY'
import sys
path = sys.argv[1]
lines = open(path).read().split('\n')
if any('time-sync.target' in l for l in lines):
    print('[rpi]    既に設定済み'); sys.exit(0)
out = []
for l in lines:
    out.append(l)
    if l.startswith('After='):
        # RTC が無く起動直後の時計が過去に戻るため、同期完了まで待つ。
        # ずれたまま起動すると DDS のディスカバリが成立しない（2026-08-20 実測）。
        out[-1] = l.rstrip() + ' time-sync.target'
        out.append('Wants=time-sync.target')
open(path, 'w').write('\n'.join(out))
print('[rpi]    After= に time-sync.target を追加した')
PY

systemctl daemon-reload

echo "[rpi] 3. 同期を待つ（最大 90 秒）"
for i in $(seq 1 18); do
  if [ "$(timedatectl show -p NTPSynchronized --value)" = "yes" ]; then
    echo "[rpi]    同期した（${i}回目の確認）"; break
  fi
  sleep 5
done

echo "[rpi] === 結果 ==="
timedatectl show -p NTPSynchronized -p TimeUSec
timedatectl timesync-status 2>&1 | grep -E "Server|Packet count|Offset" || true

if [ "$(timedatectl show -p NTPSynchronized --value)" != "yes" ]; then
  echo "[rpi] まだ同期していない。PC 側で 123/udp が開いているか確認する:" >&2
  echo "  ss -lunp | grep :123" >&2
  exit 1
fi

echo "[rpi] 4. rplidar を再起動"
systemctl restart rplidar
sleep 12
journalctl -u rplidar --since "-40 seconds" --no-pager 2>/dev/null \
  | grep -o "current scan mode: [A-Za-z]*.*" | tail -1 || true
systemctl is-active rplidar
echo
echo "[rpi] 完了。元に戻す場合:"
echo "  sudo cp -a $BACKUP $UNIT && sudo systemctl daemon-reload && sudo systemctl restart rplidar"
echo "  sudo systemctl disable systemd-time-wait-sync"
