#!/bin/bash
# TH: PC↔ラズパイの無線を屋内用／屋外用で切り替える。**PC で実行する**（ラズパイの sudo パスワードを聞かれる）。
#
#   bash ap_band_switch.sh indoor     # ロボット回線は AIC8800 → 5GHz W52 の th-rpi-5g。屋内専用・速い
#   bash ap_band_switch.sh outdoor    # 5GHz の AP を止め、内蔵 wlo1 → 2.4GHz ch1 の th-rpi-ap。屋外でも使える
#   bash ap_band_switch.sh status
#
# W52 は登録局でない限り屋外で使えない（電波法。docs/network.md「屋外で使うとき」）。
# 屋内: AIC8800 (wlx6c1ff789d5d4) はロボット専用。挿せば th-rpi-5g-aic で自動接続し、内蔵 wlo1 は
#       インターネット用（保存済みの Wi-Fi に自動接続）。AIC8800 が挿さっていなければ wlo1 を 5GHz へ。
# 屋外: AIC8800 は 2.4GHz が遅く途切れる（2026-10-21 実測）ので使わない。wlo1 をロボット回線にする。
#       PC と UnitV2 が同じ ch1 を分け合うので、映像の帯域は 2 バンド化の前と同じ。
# どちらも PC は 192.168.5.50 固定。2 枚を同時にロボット AP へつながない（IP が重なる）。
# 切り替え中は PC↔ラズパイの無線が数秒〜1 分切れる。bringup 中にやらないこと。
set -e
RPI=${RPI:-mirs2602@192.168.5.1}
AIC=wlx6c1ff789d5d4
AIC_IN=th-rpi-5g-aic
WLO_IN=th-rpi-5g-wlo1
WLO_OUT=th-rpi-ap-wlo1

up_wait() {   # 新しい AP が上がるまで待ってつなぐ（5GHz は起動に時間がかかる）
  for _ in $(seq 1 20); do
    nmcli con up "$1" >/dev/null 2>&1 && { echo "PC: $1 に接続"; return 0; }
    sleep 3
  done
  echo "PC: $1 に繋がらない。docs/network.md「PC が AP に繋がらない」"; return 1
}
aic_present() { [ -e "/sys/class/net/$AIC" ]; }

case "$1" in
status)
  nmcli -t -f DEVICE,CONNECTION dev | grep -E "^(wlo1|$AIC):" || true
  ssh "$RPI" 'bash ~/rpi_ap_band.sh status'
  ;;
indoor)
  ssh -t "$RPI" "sudo bash ~/rpi_ap_band.sh indoor"
  nmcli con modify "$WLO_OUT" connection.autoconnect no
  if aic_present; then
    nmcli con down "$WLO_OUT" >/dev/null 2>&1 || true
    sleep 2
    nmcli con modify "$AIC_IN" connection.autoconnect yes
    up_wait "$AIC_IN"
    # nmcli dev connect は autoconnect no の th-rpi-ap-wlo1 も選び直す（IP が重なる）ので使わない。
    # wlo1 の自動接続を許して、autoconnect yes の保存済み Wi-Fi から NM に選ばせる
    nmcli dev set wlo1 autoconnect yes
  else
    echo "AIC8800 が挿さっていないので wlo1 を 5GHz のロボット回線にする"
    up_wait "$WLO_IN"
  fi
  ping -c 3 -q 192.168.5.1 | tail -1
  ;;
outdoor)
  # 5GHz を止める前に PC を 2.4GHz へ移す（先に止めると ssh ごと切れる）。AIC8800 は先に外す（IP が重なる）
  aic_present && nmcli dev disconnect "$AIC" >/dev/null 2>&1 || true
  nmcli con modify "$WLO_OUT" connection.autoconnect yes connection.autoconnect-priority 20 connection.autoconnect-retries 0
  up_wait "$WLO_OUT"
  ssh -t "$RPI" "sudo bash ~/rpi_ap_band.sh outdoor"
  ping -c 3 -q 192.168.5.1 | tail -1
  ;;
*)
  sed -n '3,6p' "$0"; exit 1 ;;
esac
