#!/usr/bin/env bash
# ラズパイを親機（AP）にする（要 sudo・ラズパイ上で実行）
#
#   sudo bash rpi_setup_ap.sh [SSID] [パスフレーズ] [チャンネル]
#   sudo bash rpi_setup_ap.sh --revert     # AP を止める
#
# 背景（実測 2026-08-20）:
#   ESP32 を SoftAP にしていた構成では、ESP32 の無線が負荷時に 520〜640ms
#   丸ごと停止していた（20 分で 10 回）。ラズパイの /scan も ESP32 が中継して
#   いたため同時に途切れる。LiDAR 中継を外すと 10 回 → 3 回に減った。
#   詳細: docs/plan/detailed/data/meas05/README.md
#
# ★ 事前に有線の退路を作っておくこと（無線設定を誤ると機体に触れなくなる）:
#     ラズパイ: nmcli con add type ethernet ifname eth0 con-name th-wired \
#                 ipv4.method manual ipv4.addresses 10.42.0.2/24 ipv4.never-default yes
#     PC:       nmcli con mod "<有線接続名>" ipv4.method manual \
#                 ipv4.addresses 10.42.0.1/24 ipv4.never-default yes
#
# hostapd / dnsmasq パッケージは不要（ラズパイは NetworkManager 管理下で、
# NM の共有モードが dnsmasq-base のバイナリを使う）。現場の AP は外に出られず
# apt が使えないので、これは重要な性質。
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "sudo で実行する" >&2; exit 1; }
IFACE="${IFACE:-wlan0}"

if [ "${1:-}" = "--revert" ]; then
  nmcli con down th-rpi-ap 2>/dev/null || true
  nmcli con mod th-rpi-ap connection.autoconnect no 2>/dev/null || true
  nmcli dev connect "$IFACE" 2>/dev/null || true
  echo "[rpi] AP を停止した"
  nmcli -t -f NAME,DEVICE,STATE con show --active | grep "$IFACE" || true
  exit 0
fi

SSID="${1:-th-rpi-ap}"
PSK="${2:-throbot2026}"
CH="${3:-6}"
AP_IP="${AP_IP:-192.168.5.1}"
[ ${#PSK} -ge 8 ] || { echo "パスフレーズは 8 文字以上" >&2; exit 1; }

echo "[rpi] AP を作成: SSID=$SSID ch=$CH IP=$AP_IP"
nmcli con delete th-rpi-ap 2>/dev/null || true
nmcli con add type wifi ifname "$IFACE" con-name th-rpi-ap autoconnect yes \
  ssid "$SSID" \
  802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel "$CH" \
  802-11-wireless.powersave 2 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" \
  ipv4.method shared ipv4.addresses "${AP_IP}/24" \
  ipv4.never-default yes ipv6.method ignore > /dev/null

# ★ 重要: ESP32 (Arduino) は WPA2-PSK/CCMP のみで、PMF(802.11w) が有効だと
#   association に失敗する。NetworkManager の既定は PMF=optional なので必ず切る。
#   2026-08-20: これが無いと「旧 AP は消えたのに DHCP 要求すら来ない」状態になる。
#   ESP32 側にログが出ない（このシールド基板はシリアル出力が取れない）ので
#   原因の特定に時間がかかる。
nmcli con mod th-rpi-ap \
  wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
  wifi-sec.pmf disable

nmcli con up th-rpi-ap
sleep 3
echo "[rpi] === 状態 ==="
ip -br addr show "$IFACE"
nmcli -f 802-11-wireless-security.pmf,802-11-wireless-security.pairwise con show th-rpi-ap
echo
echo "[rpi] 子機の確認（ESP32 が繋がると出る）:"
echo "  sudo cat /var/lib/NetworkManager/dnsmasq-${IFACE}.leases"
echo "[rpi] 戻す: sudo bash rpi_setup_ap.sh --revert"
