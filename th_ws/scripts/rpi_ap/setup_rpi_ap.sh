#!/usr/bin/env bash
# ラズパイを 2.4GHz の親機にする（AP 選定の候補 B-2）
#   sudo bash setup_rpi_ap.sh <SSID> <パスフレーズ> <チャンネル>
#
# ★ 適用と同時に「15 分後にクライアントモードへ戻す」ジョブを予約する。
#   切り替え後に疎通が取れたら cancel_revert.sh で取り消すこと。
#   取り消さなければ自動で元に戻る（設定を誤ってもラズパイを失わないため）。
set -euo pipefail
SSID="${1:?SSID}"
PSK="${2:?パスフレーズ（8文字以上）}"
CH="${3:-6}"
IFACE="${IFACE:-wlan0}"
AP_IP="192.168.5.1"

echo "== 1/5 必要なパッケージ =="
apt-get update
apt-get install -y hostapd dnsmasq
systemctl unmask hostapd

echo "== 2/5 復旧ジョブを先に予約（15 分後にクライアントへ戻す）=="
cat > /usr/local/sbin/th_revert_client.sh <<'REVERT'
#!/usr/bin/env bash
# 親機モードをやめてクライアントに戻す（自動復旧用）
systemctl stop hostapd dnsmasq || true
systemctl disable hostapd dnsmasq || true
ip addr flush dev "${IFACE:-wlan0}" || true
systemctl restart wpa_supplicant || true
systemctl restart NetworkManager 2>/dev/null || true
systemctl restart dhcpcd 2>/dev/null || true
logger -t th_ap "自動復旧: クライアントモードへ戻した"
REVERT
chmod +x /usr/local/sbin/th_revert_client.sh
systemd-run --unit=th_ap_revert --on-active=15min /usr/local/sbin/th_revert_client.sh
echo "   → 15 分後に自動復旧します（取り消しは systemctl stop th_ap_revert.timer）"

echo "== 3/5 hostapd の設定 =="
cat > /etc/hostapd/hostapd.conf <<CONF
interface=${IFACE}
driver=nl80211
ssid=${SSID}
hw_mode=g
channel=${CH}
wmm_enabled=1
auth_algs=1
wpa=2
wpa_passphrase=${PSK}
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
ieee80211n=1
CONF
chmod 600 /etc/hostapd/hostapd.conf
sed -i 's|^#\?DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd || true

echo "== 4/5 IP と DHCP =="
cat > /etc/dnsmasq.d/th_ap.conf <<DNS
interface=${IFACE}
dhcp-range=192.168.5.50,192.168.5.150,255.255.255.0,24h
DNS

echo "== 5/5 切り替え =="
systemctl stop wpa_supplicant 2>/dev/null || true
nmcli dev set "${IFACE}" managed no 2>/dev/null || true
ip addr flush dev "${IFACE}"
ip addr add ${AP_IP}/24 dev "${IFACE}"
ip link set "${IFACE}" up
systemctl restart dnsmasq
systemctl restart hostapd
sleep 3
systemctl is-active hostapd && echo "hostapd 起動" || { echo "hostapd 起動失敗"; journalctl -u hostapd -n 20 --no-pager; }
echo "完了: SSID=${SSID} ch=${CH} 親機IP=${AP_IP}"
