#!/bin/bash
# TH: PC 用の AP（Elecom WDC-433SU2M2）の帯域を屋内用／屋外用で切り替える。ラズパイ上で sudo 実行する。
#
#   sudo bash rpi_ap_band.sh indoor            # 5GHz W52 ch48・40MHz (SSID th-rpi-5g)。屋内専用
#   sudo bash rpi_ap_band.sh outdoor           # 5GHz の AP を止める。PC も内蔵 2.4GHz の th-rpi-ap につなぐ
#   sudo bash rpi_ap_band.sh status            # いまの設定
#   (--no-restart を付けると設定を書くだけで hostapd を再起動しない)
#
# W52 は登録局でない限り屋外で使えない（電波法。docs/network.md「屋外で使うとき」）。
# 内蔵 2.4GHz の th-rpi-ap (ch1。UnitV2・昇降機 ESP32) はどちらのモードでも変えない。
# outdoor では映像と PC が同じ 2.4GHz ch1 を分け合うので、映像の帯域は 2 バンド化の前と同じに戻る。
# Elecom WDC-433SU2M2 は 5GHz 専用で、2.4GHz の AP にすると起動したと出るのに電波が出ない（2026-10-21）。
# モードは hostapd@<Elecom> の enable/disable で残るので、ラズパイを再起動しても同じモードで上がる。
# PC 側の接続の切り替えは ap_band_switch.sh（PC で実行）がこのスクリプトと一緒にやる。
set -e
DONGLE=wlx3897a478b19d
CONF=/etc/hostapd/$DONGLE.conf
MODE=$1
RESTART=1
[ "$2" = "--no-restart" ] && RESTART=0

if [ "$MODE" = status ]; then
  if systemctl is-enabled --quiet "hostapd@$DONGLE"; then echo "indoor (5GHz の AP を使う)"; else echo "outdoor (5GHz の AP は止めている)"; fi
  systemctl is-active "hostapd@$DONGLE" || true
  iw dev "$DONGLE" info 2>/dev/null | grep -E 'ssid|channel' || true
  exit 0
fi

PSK=$(grep -h '^wpa_passphrase=' /etc/hostapd/wlan0.conf | cut -d= -f2-)
[ -n "$PSK" ] || { echo "/etc/hostapd/wlan0.conf から PSK を読めない（先に rpi_setup_dualband_ap.sh）"; exit 1; }

case "$MODE" in
indoor)
  cat > "$CONF" <<EOF
# Elecom WDC-433SU2M2 (rtl8821au): PC 専用。屋内用 5GHz W52 の ch48 (44+48 の HT40-)。rpi_ap_band.sh が書く
interface=$DONGLE
bridge=br0
driver=nl80211
ssid=th-rpi-5g
country_code=JP
ieee80211d=1
hw_mode=a
channel=48
ieee80211n=1
ht_capab=[HT40-][SHORT-GI-20][SHORT-GI-40]
ieee80211ac=1
vht_oper_chwidth=0
wmm_enabled=1
auth_algs=1
wpa=2
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
ieee80211w=0
wpa_passphrase=$PSK
EOF
  ;;
outdoor)
  systemctl disable "hostapd@$DONGLE" >/dev/null 2>&1
  echo "outdoor: 5GHz の AP を止める (設定 $CONF は残す)"
  # 5GHz 越しの ssh から呼ばれても止めきれるよう切り離す
  setsid systemctl stop "hostapd@$DONGLE" < /dev/null > /dev/null 2>&1 &
  exit 0
  ;;
*)
  sed -n '3,6p' "$0"; exit 1 ;;
esac
chmod 600 "$CONF"
echo "wrote $MODE -> $CONF"
systemctl enable "hostapd@$DONGLE" >/dev/null 2>&1
# ssh が無線越しでも切れずに最後まで走るよう、再起動は切り離して行う
if [ "$RESTART" = 1 ]; then
  setsid systemctl restart "hostapd@$DONGLE" < /dev/null > /dev/null 2>&1 &
fi
exit 0
