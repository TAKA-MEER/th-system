#!/usr/bin/env bash
# PC を 192.168.4.0/24 向けの NTP サーバーにする（要 sudo・PC 上で実行）
#
#   sudo bash pc_setup_ntp_server.sh
#
# 背景（実機確認 2026-08-20）:
#   ラズパイには RTC が無く、起動のたびに時計が過去へ戻る（実際に 3 日ずれた）。
#   さらに**時計がずれていると DDS のディスカバリ自体が成立しない**（パケットは
#   双方向に流れているのにノードが一切見えない）。時計を合わせた直後に直った。
#   ラズパイ側は /etc/systemd/timesyncd.conf で NTP=192.168.4.50 を向いているが、
#   Packet count: 0 ＝ 問い合わせても返事が無い状態だった。
#   PC は systemd-timesyncd（クライアント専用）しか入っておらず 123/udp を
#   待ち受けていない。docs/setup.md の「PC の Windows Time を NTP サーバー化」は
#   PC が Ubuntu になった時点で成立しなくなっていた。
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "sudo で実行する" >&2; exit 1; }

echo "[pc] chrony を導入中..."
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq chrony

CONF=/etc/chrony/chrony.conf
DROPIN=/etc/chrony/conf.d/th-robot.conf
BLOCK='# TH システム: ラズパイ (192.168.4.2) へ時刻を配る。
# local stratum 10 = 上流に繋がっていなくても配る（現場は隔離 AP のため必須）。
allow 192.168.4.0/24
local stratum 10'

if grep -qE '^\s*confdir\s+/etc/chrony/conf\.d' "$CONF" 2>/dev/null; then
  mkdir -p /etc/chrony/conf.d
  printf '%s\n' "$BLOCK" > "$DROPIN"
  echo "[pc] 設定を書いた: $DROPIN"
else
  # conf.d を読まない版。chrony.conf に直接足す（二重追記を避ける）
  if grep -q "TH システム: ラズパイ" "$CONF"; then
    echo "[pc] 設定は既に $CONF にある"
  else
    cp -a "$CONF" "${CONF}.bak.$(date +%Y%m%d_%H%M%S)"
    printf '\n%s\n' "$BLOCK" >> "$CONF"
    echo "[pc] 設定を $CONF に追記した（バックアップ済み）"
  fi
fi

systemctl enable --now chrony
systemctl restart chrony
sleep 3

echo "[pc] === 123/udp の待ち受け ==="
ss -lunp | grep ':123' || { echo "[pc] 待ち受けていない。chrony のログを見る:" >&2
                            journalctl -u chrony -n 20 --no-pager >&2; exit 1; }
echo "[pc] === 自身の同期状態 ==="
chronyc tracking | grep -E 'Reference ID|Stratum|System time' || true
echo
echo "[pc] 完了。次にラズパイ側で rpi_fix_clock.sh を実行する。"
