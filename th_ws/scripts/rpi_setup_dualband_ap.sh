#!/bin/bash
# TH: ラズパイ AP を「内蔵 2.4GHz + Elecom 5GHz を br0 で 1 つの L2 に束ねる」構成にする。
# ラズパイ上で sudo 実行する。戻すときは rpi_dualband_ap_revert.sh。
#
#   sudo bash rpi_setup_dualband_ap.sh
#
# 内蔵 (wlan0, 2.4GHz ch1, SSID th-rpi-ap): UnitV2・昇降機 ESP32 用。旧 NM の AP と同じ設定
# Elecom (wlx3897a478b19d, 5GHz ch48 40MHz, SSID th-rpi-5g): PC 専用。屋外では rpi_ap_band.sh outdoor
# どちらも br0 (192.168.5.1/24, NM の shared = DHCP) の下にあるので IP は今までどおり。
#
# 前提（手順と理由は docs/network.md「ラズパイ: 2 バンド AP の導入」）:
#   - 8821au ドライバをビルド・インストール済み (/etc/modprobe.d/8821au.conf)
#   - /etc/modprobe.d/cfg80211.conf で規制ドメイン JP (00 のままだと 5GHz の AP が起動しない)
#   - hostapd・iw を dpkg -i 済み (ラズパイはインターネットに出られない)
#   - 有線の退路 (eth0 th-wired 10.42.0.2) がある
# 内蔵側に ieee80211w=0 を書かないこと。brcmfmac が接続を status 38 で拒否する (2026-10-21)。
set -e
DONGLE=wlx3897a478b19d
PSK=$(nmcli -s -g 802-11-wireless-security.psk con show th-rpi-ap)
[ -n "$PSK" ] || { echo "th-rpi-ap の PSK を読めない"; exit 1; }

install -d /etc/hostapd
cat > /etc/hostapd/wlan0.conf <<CONF
# 内蔵 brcmfmac: UnitV2・昇降機 ESP32 用。旧 NM の th-rpi-ap と同じ SSID・ch・暗号
interface=wlan0
bridge=br0
driver=nl80211
ssid=th-rpi-ap
country_code=JP
hw_mode=g
channel=1
ieee80211n=1
wmm_enabled=1
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
wpa_passphrase=$PSK
CONF
chmod 600 /etc/hostapd/wlan0.conf
# PC 用 (Elecom) は屋内用 5GHz で書く。屋外用への切り替えも同じスクリプト
bash "$(dirname "$0")/rpi_ap_band.sh" indoor --no-restart

install -d /etc/systemd/system/hostapd@.service.d
cat > /etc/systemd/system/hostapd@.service.d/th.conf <<'CONF'
# TH: br0 は NetworkManager が作る。hostapd が先に自前で br0 を作らないよう、出来るまで待つ
[Unit]
After=NetworkManager.service
Wants=NetworkManager.service
[Service]
ExecStartPre=/bin/sh -c 'for i in $(seq 1 120); do [ -d /sys/class/net/br0 ] && exit 0; sleep 0.5; done; echo "br0 が無い"; exit 1'
ExecStartPost=-/usr/sbin/iw dev %i set power_save off
CONF

cat > /etc/NetworkManager/conf.d/90-th-hostapd.conf <<CONF
# TH: 2 つの無線は hostapd が持つ。eth0 (th-wired) は NM のまま
[keyfile]
unmanaged-devices=interface-name:wlan0;interface-name:$DONGLE
CONF
cat > /etc/NetworkManager/dnsmasq-shared.d/th-reserve.conf <<'CONF'
# TH: br0 移行でリースファイルが変わっても IP を変えない
dhcp-host=e0:5a:1b:15:c3:1c,192.168.5.142
dhcp-host=c8:cb:9e:d8:a5:9a,192.168.5.50
dhcp-host=00:e0:99:7c:99:e7,192.168.5.129
CONF

nmcli con modify th-rpi-ap connection.autoconnect no
nmcli con show th-br0 >/dev/null 2>&1 || nmcli con add type bridge ifname br0 con-name th-br0 \
  bridge.stp no ipv4.method shared ipv4.addresses 192.168.5.1/24 ipv6.method disabled connection.autoconnect yes
nmcli con down th-rpi-ap || true
systemctl daemon-reload
systemctl reload NetworkManager
sleep 2
nmcli con up th-br0
systemctl enable hostapd@wlan0.service hostapd@$DONGLE.service
systemctl restart hostapd@wlan0.service hostapd@$DONGLE.service
echo setup-done
