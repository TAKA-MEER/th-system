#!/bin/bash
# TH: PC↔ラズパイの無線を屋内用／屋外用で切り替える。**PC で実行する**（ラズパイの sudo パスワードを聞かれる）。
#
#   bash ap_band_switch.sh indoor     # PC は 5GHz W52 の th-rpi-5g。屋内専用・速い
#   bash ap_band_switch.sh outdoor    # 5GHz の AP を止め、PC も 2.4GHz ch1 の th-rpi-ap。屋外でも使える
#   bash ap_band_switch.sh status
#
# W52 は登録局でない限り屋外で使えない（電波法。docs/network.md「屋外で使うとき」）。
# outdoor では PC と UnitV2 が同じ 2.4GHz ch1 を分け合うので、映像の帯域は 2 バンド化の前と同じ。
# ラズパイ側は rpi_ap_band.sh（ラズパイのホームに置く）、PC 側は wlo1 の接続を付け替える。
# 切り替え中は PC↔ラズパイの無線が数秒〜1 分切れる。bringup 中にやらないこと。
set -e
RPI=${RPI:-mirs2602@192.168.5.1}
IN=th-rpi-5g-wlo1
OUT=th-rpi-ap-wlo1

up_wait() {   # 新しい AP が上がるまで待ってつなぐ（5GHz は起動に時間がかかる）
  for _ in $(seq 1 20); do
    nmcli con up "$1" >/dev/null 2>&1 && { echo "PC: $1 に接続"; return 0; }
    sleep 3
  done
  echo "PC: $1 に繋がらない。もう片方で入って直す: nmcli connection up <接続名>"; return 1
}

case "$1" in
status)
  nmcli -f SSID,CHAN,RATE,ACTIVE dev wifi list ifname wlo1 | grep -E 'はい|yes' || echo "wlo1 は未接続"
  ssh "$RPI" 'bash ~/rpi_ap_band.sh status'
  ;;
indoor)
  ssh -t "$RPI" "sudo bash ~/rpi_ap_band.sh indoor"
  nmcli con modify "$OUT" connection.autoconnect no
  nmcli con modify "$IN" connection.autoconnect yes
  up_wait "$IN"
  ping -c 3 -q 192.168.5.1 | tail -1
  ;;
outdoor)
  # 5GHz を止める前に PC を 2.4GHz へ移す（先に止めると ssh ごと切れる）
  nmcli con modify "$IN" connection.autoconnect no
  nmcli con modify "$OUT" connection.autoconnect yes connection.autoconnect-priority 20 connection.autoconnect-retries 0
  up_wait "$OUT"
  ssh -t "$RPI" "sudo bash ~/rpi_ap_band.sh outdoor"
  ping -c 3 -q 192.168.5.1 | tail -1
  ;;
*)
  sed -n '3,6p' "$0"; exit 1 ;;
esac
