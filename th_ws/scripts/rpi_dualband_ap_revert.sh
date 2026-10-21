#!/bin/bash
# TH: rpi_setup_dualband_ap.sh を取り消し、NM の th-rpi-ap (内蔵 2.4GHz 単独) に戻す。
# ラズパイ上で sudo 実行する。PC 側は th-rpi-ap-wlo1 を autoconnect yes に戻すこと。
DONGLE=wlx3897a478b19d
systemctl disable --now hostapd@wlan0.service hostapd@$DONGLE.service
rm -f /etc/NetworkManager/conf.d/90-th-hostapd.conf
nmcli con delete th-br0
systemctl reload NetworkManager
sleep 2
nmcli con modify th-rpi-ap connection.autoconnect yes
nmcli con up th-rpi-ap
echo rollback-done
