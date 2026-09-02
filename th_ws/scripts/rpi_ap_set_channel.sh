#!/bin/sh
# ============================================================
# ラズパイ AP (th-rpi-ap) のチャネルを変更する。**ラズパイ上で sudo 実行する。**
#
#   sudo sh rpi_ap_set_channel.sh 6
#
# なぜ必要か (2026-09-02 実機計測):
#   AP は 2.4GHz ch1 で動いていたが、構内 AP が同じ ch1 に 9 局あり、うち 6 局が
#   信号 60 以上 (eduroam / NCT-WL* が 65〜69 で、ロボット AP と同等の強さ)。
#   この状態で PC→ラズパイ・PC→ESP32 の**両方**が
#     ロス 22〜30% / RTT min 1.2ms・avg 500〜700ms・max 3〜6 秒
#   という壊れ方をしており、ESP32 の WebSocket 切断が頻発していた。
#   LiDAR の /scan を止めても改善しない＝トラフィック起因ではなく純粋に電波。
#
# なぜ ch6 か:
#   PC 側で 5 回スキャンを統合 (一意 BSSID 33 件) して重み付け評価した結果。
#     ch 1: 重み付け 505 / 最強干渉源 69 / 信号60以上 6 局   ← 現行
#     ch 6: 重み付け 464 / 最強干渉源 40 / 信号60以上 0 局   ← 最良
#     ch11: 重み付け 505 / 最強干渉源 75 / 信号60以上 6 局
#   ch6 は局数こそ多いが**すべて弱い**（最強 40）。効くのは局数ではなく
#   強い干渉源の有無なので ch6 が最良。
#
# なぜ 5GHz にしないか:
#   **ESP32 は 2.4GHz 専用**（5GHz 無線を持たない）。AP を 5GHz にすると
#   ESP32 が一切繋がらなくなる。2.4GHz 内でのチャネル選択が唯一の手段。
#
# 安全網:
#   AP を落とすと ssh も切れる。このスクリプトは切り離して動き、変更後に
#   AP が復帰しなければ**自動で元のチャネルに戻す**ので、ロボットが到達不能に
#   なったまま取り残されることはない。
# ============================================================
set -e

CON=th-rpi-ap
NEW_CH="${1:?使い方: sudo sh rpi_ap_set_channel.sh <チャネル番号(1-13)>}"
LOG=/tmp/rpi_ap_set_channel.log

case "$NEW_CH" in
  ''|*[!0-9]*) echo "チャネルは数字で指定すること: $NEW_CH" >&2; exit 1 ;;
esac
if [ "$NEW_CH" -lt 1 ] || [ "$NEW_CH" -gt 13 ]; then
  echo "2.4GHz のチャネルは 1-13。ESP32 が 2.4GHz 専用なのでバンドは変えない" >&2
  exit 1
fi

OLD_CH="$(nmcli -g 802-11-wireless.channel con show "$CON")"
echo "現在のチャネル: $OLD_CH  → 変更先: $NEW_CH"
if [ "$OLD_CH" = "$NEW_CH" ]; then
  echo "同じチャネルなので何もしない"
  exit 0
fi

# 切り離して実行する。ssh が切れてもこのブロックは走りきる。
setsid nohup sh -c "
  exec >>'$LOG' 2>&1
  echo \"=== \$(date -Is) ch$OLD_CH -> ch$NEW_CH ===\"

  nmcli con modify '$CON' 802-11-wireless.channel $NEW_CH
  nmcli con down '$CON' || true
  sleep 3
  nmcli con up '$CON' || true
  sleep 10

  # 復帰確認: wlan0 が activated (state 100) になっているか
  if nmcli -t -f GENERAL.STATE dev show wlan0 | grep -q '100'; then
    echo \"OK: ch$NEW_CH で AP 復帰\"
  else
    echo \"NG: 復帰せず。ch$OLD_CH へ自動で戻す\"
    nmcli con modify '$CON' 802-11-wireless.channel $OLD_CH
    nmcli con down '$CON' || true
    sleep 2
    nmcli con up '$CON' || true
    sleep 8
    nmcli -t -f GENERAL.STATE dev show wlan0 | grep -q '100' \
      && echo 'ロールバック成功' || echo '致命的: AP が上がらない。有線か画面で復旧が必要'
  fi
  echo \"最終チャネル: \$(nmcli -g 802-11-wireless.channel con show '$CON')\"
" >/dev/null 2>&1 &

echo
echo "切り離して実行を開始した。ssh はこの直後に切れる。"
echo "20〜30 秒待って PC を SSID '$CON' に再接続し、結果を確認する:"
echo "  ssh mirs2602@192.168.5.1 'cat $LOG; nmcli -g 802-11-wireless.channel con show $CON'"
