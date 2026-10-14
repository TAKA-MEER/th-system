#!/usr/bin/env bash
# pi_serial_relay の起動ラッパー（ラズパイに置く。rpi-serial-relay.service の ExecStart から呼ぶ）
#
# 導入先: /home/mirs2602/rpi_serial_relay_run.sh （実行権限つき）
#
# 理由（実機 2026-10-14）: ESP32 を交換したら by-id のシリアルIDが変わり、ユニットに固定で書いた
#   パスが消えて中継が起動できなくなった（ESP32_DISCONNECTED のまま）。
#   → ESP32 のIDを決め打ちせず、「LiDAR 以外の USB-UART」を ESP32 として選ぶ。
#   LiDAR は rpi_rplidar_run.sh と同じ既知のID（RPLIDAR_DEVICE）で見分ける。
#
# 選び方: /dev/serial/by-id/ のうち LiDAR のID以外が「ちょうど 1 つ」なら、それを使う。
#   0 個 → 現れるまで最大 ${RELAY_WAIT_MAX} 秒待ち、超えたら終了（systemd が立て直す）。
#   2 個以上 → 取り違えを避けるため起動せず終了し、候補をログに出す
#   （RELAY_SERIAL_PORT で明示指定すれば従う）。
#   LiDAR を交換したときは rplidar.service 側と一緒にこのスクリプトの LiDAR ID も直すこと。
#
# 環境変数（試験用。実機は既定のまま）:
#   RELAY_SERIAL_PORT  明示指定（あれば自動選択しない）
#   RPLIDAR_DEVICE     LiDAR の by-id パス（既定は rpi_rplidar_run.sh と同じ）
#   RELAY_BYID_DIR     by-id ディレクトリ（既定 /dev/serial/by-id）
#   RELAY_WAIT_MAX     候補が現れるまでの待ち上限（秒・既定 60）
#   RELAY_PY / RELAY_SCRIPT / RELAY_ARGS  起動するもの（既定は pi_serial_relay.py と --ws-host/--ws-port）
LIDAR_DEFAULT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0b443775f827c8419a0592b44475a0a2-if00-port0"
LIDAR="${RPLIDAR_DEVICE:-$LIDAR_DEFAULT}"
DIR="${RELAY_BYID_DIR:-/dev/serial/by-id}"
WAIT_MAX="${RELAY_WAIT_MAX:-60}"
PY="${RELAY_PY:-/usr/bin/python3}"
SCRIPT="${RELAY_SCRIPT:-/home/mirs2602/pi_serial_relay.py}"
ARGS="${RELAY_ARGS:---ws-host 192.168.5.50 --ws-port 8766}"

find_candidates() {
  local p
  for p in "$DIR"/*; do
    [ -e "$p" ] || continue
    [ "$p" = "$LIDAR" ] && continue
    echo "$p"
  done
}

PORT="${RELAY_SERIAL_PORT:-}"
if [ -z "$PORT" ]; then
  waited=0
  while :; do
    mapfile -t CANDS < <(find_candidates)
    [ "${#CANDS[@]}" -ge 1 ] && break
    if [ "$waited" -ge "$WAIT_MAX" ]; then
      echo "rpi_serial_relay_run: ESP32 の USB-UART が ${WAIT_MAX} 秒たっても見つからない（LiDAR 以外のデバイスが無い）: $DIR" >&2
      exit 1
    fi
    sleep 1; waited=$((waited + 1))
  done
  if [ "${#CANDS[@]}" -ne 1 ]; then
    echo "rpi_serial_relay_run: ESP32 の候補が ${#CANDS[@]} 個ある。取り違えを避けるため起動しない。RELAY_SERIAL_PORT で指定すること:" >&2
    printf '  %s\n' "${CANDS[@]}" >&2
    exit 1
  fi
  PORT="${CANDS[0]}"
fi
echo "rpi_serial_relay_run: ESP32 のポート = $PORT"
/usr/bin/stty -F "$PORT" -hupcl 2>/dev/null || true
# shellcheck disable=SC2086
exec "$PY" "$SCRIPT" --serial-port "$PORT" $ARGS
