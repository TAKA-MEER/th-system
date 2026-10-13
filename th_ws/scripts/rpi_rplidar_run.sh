#!/usr/bin/env bash
# 見張り付きの RPLIDAR 起動スクリプト（ラズパイに置く。rplidar.service の ExecStart から呼ぶ）
#
# 導入先: /home/mirs2602/rpi_rplidar_run.sh （実行権限つき）
#         導入手順は docs/network.md「ラズパイ: rplidar の自動復帰の導入」
#
# 理由（実機 2026-10-13）: LiDAR の USB を抜いて挿し直すと、rplidar_node は抜く前の
#   デバイス（/dev/ttyUSB0）を握ったまま、エラーも出さず終了もしないので、systemd の
#   Restart=always が働かず /scan が戻らなかった（LIDAR_LOST が 982 秒続いた）。
#   /dev/serial/by-id/... のリンクは挿し直しで別の ttyUSB を指す（ttyUSB0 -> 2 -> 1）。
#   → このスクリプトが 1 秒ごとにリンクを見張り、変わったらコマンドを止めて非ゼロで
#   終了する。systemd（Restart=always, RestartSec=5）が新しい接続で立て直す。
#   挿し直しから /scan が戻るまで約 10 秒以内（Spec-safety §3.5.6）。
#
# 見張る対象（どれか 1 つでも変わったら止める）:
#   - リンクが消えた  - 指す先（readlink -f）が変わった
#   - リンク自体が作り直された（inode・更新時刻・状態変更時刻の変化。
#     同じ番号に戻る再挿入もここで拾う）
#
# 使い方:
#   rpi_rplidar_run.sh [デバイスのパス]
# 環境変数（試験で差し替えるためのもの。実機は既定のまま）:
#   RPLIDAR_DEVICE      見張るリンク（引数が優先。既定は by-id の CP2102）
#   RPLIDAR_CMD         起動するコマンド（bash -c に渡す文字列）。既定は ros2 run rplidar_ros ...
#   RPLIDAR_WAIT_MAX    デバイスが現れるまでの待ち上限（秒・既定 60）。超えたら終了
#   RPLIDAR_POLL        見張る間隔（秒・既定 1）
#   RPLIDAR_KILL_GRACE  TERM から KILL までの猶予（秒・既定 5）
#
# 止め方: コマンドはプロセスグループで起動し、グループ全体へ TERM（孫の ros2 run の子まで届く）。
#   止まらなければ猶予後に KILL（kill -9 は最後の手段）。

DEV_DEFAULT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0b443775f827c8419a0592b44475a0a2-if00-port0"
DEV="${1:-${RPLIDAR_DEVICE:-$DEV_DEFAULT}}"
CMD_DEFAULT='source /opt/ros/humble/setup.bash && source /home/mirs2602/ros2_ws/install/setup.bash && exec ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:="$RPLIDAR_DEVICE" -p serial_baudrate:=256000 -p frame_id:=laser_link -p angle_compensate:=true -p scan_mode:=Standard'
CMD="${RPLIDAR_CMD:-$CMD_DEFAULT}"
WAIT_MAX="${RPLIDAR_WAIT_MAX:-60}"
POLL="${RPLIDAR_POLL:-1}"
GRACE="${RPLIDAR_KILL_GRACE:-5}"
export RPLIDAR_DEVICE="$DEV"

log() { echo "[rplidar_run] $*"; }

# リンクの「同一性」を表す文字列。消えていれば GONE。
signature() {
  if [ ! -L "$DEV" ] && [ ! -e "$DEV" ]; then
    echo GONE
    return
  fi
  echo "$(readlink -f "$DEV") $(stat -c '%i %.9Y %.9Z' "$DEV" 2>/dev/null)"
}

# 1. デバイスが現れるまで待つ（上限つき）
waited=0
while [ "$(signature)" = GONE ]; do
  if [ "$waited" -ge "$WAIT_MAX" ]; then
    log "デバイス $DEV が ${WAIT_MAX} 秒待っても現れない。終了する（systemd が再起動）"
    exit 2
  fi
  [ "$waited" -eq 0 ] && log "デバイス $DEV の出現を待つ"
  sleep 1
  waited=$((waited + 1))
done

BASE="$(signature)"
log "デバイス確認: $BASE"

# 2. コマンドを新しいプロセスグループで起動する
setsid bash -c "$CMD" &
PID=$!
log "起動: pid=$PID（プロセスグループ $PID）"

stop_group() {
  kill -TERM -- "-$PID" 2>/dev/null
  local i=0
  while kill -0 -- "-$PID" 2>/dev/null; do
    if [ "$i" -ge "$GRACE" ]; then
      log "TERM で止まらないので KILL する"
      kill -KILL -- "-$PID" 2>/dev/null
      break
    fi
    sleep 1
    i=$((i + 1))
  done
  wait "$PID" 2>/dev/null
}

on_signal() {
  log "シグナルを受けた。コマンドを止めて終了する"
  stop_group
  exit 143
}
trap on_signal TERM INT

# 3. 見張り
while true; do
  if ! kill -0 "$PID" 2>/dev/null; then
    wait "$PID"
    rc=$?
    log "コマンドが終了した（終了コード $rc）"
    kill -TERM -- "-$PID" 2>/dev/null   # 残った子があれば片付ける
    exit "$rc"
  fi
  now="$(signature)"
  if [ "$now" != "$BASE" ]; then
    log "デバイスが変わった: [$BASE] -> [$now]。コマンドを止めて終了する（systemd が再起動）"
    stop_group
    exit 3
  fi
  sleep "$POLL" &
  wait $! 2>/dev/null
done
