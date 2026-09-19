#!/bin/sh
# ============================================================
# ラズパイ AP (th-rpi-ap) のチャネルを変更する。**ラズパイ上で sudo 実行する。**
#
#   sudo sh rpi_ap_set_channel.sh 6
#
# ★ 重要 (2026-09-02): このスクリプトは「チャネル混雑が原因」という仮説で
#   作ったが、**その仮説は実機の対照実験で否定された**。今の実機に対しては
#   これを実行しても意味が無い。真因は PC 側の USB WiFi ドングル
#   (AIC8800, wlx6c1ff789d5d4)。同じ AP・同じ ch1・同じ時刻での比較:
#
#     ドングル単独      上り 3.35 / 下り 6.71 Mbps  ロス 18%  RTT avg 151ms max 970ms
#     内蔵Intel単独     上り 28.6 / 下り 31.9 Mbps  ロス  0%  RTT avg 2.2ms max 11ms
#
#   内蔵カードは混雑した ch1 のままロス 0% / RTT 2.2ms を出す。つまり ch1 の
#   混雑は律速ではない。対処は「ロボット回線を内蔵 Intel カードに移す」か
#   「ドングルを別のものに替える」。docs/network.md と CLAUDE.md を参照。
#
# それでもこのスクリプトを残してある理由:
#   会場が変わって本当にチャネル混雑が問題になったときに、AP を安全に
#   （失敗しても自動復旧して）移すための道具として有効なため。
#
# なぜ 5GHz にしないか (これは今も有効な制約):
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
