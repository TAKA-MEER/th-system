#!/bin/bash
# 計測用コンテナの中で動く。
#   rec.sh router       … zenoh ルータだけを立てっぱなしにする（計測の間ずっと別端末で動かしておく）
#   rec.sh probe        … スマホから届く Imu の周期・欠け・加速度の大きさを見る
#   rec.sh record NAME  … スマホ(zenoh)と機体(FastDDS ドメイン10)を同時に bag へ。Ctrl-C で止める
# 出力は /root/th_data/vibe/NAME/{phone,robot}
source /opt/ros/humble/setup.bash
source /root/vibe_ws/install/setup.bash 2>/dev/null   # WheelFeedback 型のため

PHONE_DOMAIN=${PHONE_DOMAIN:-0}      # Conduit の Domain ID と合わせる
ROBOT_TOPICS="/cmd_vel /cmd_vel_muxed /esp32/wheel_cmd_speed /esp32/wheel_feedback /odom /esp32/imu_data /system/state"

zenoh_env() { env RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=$PHONE_DOMAIN "$@"; }

ROUTER=
start_router() {
  # 既にルータがいれば（vibe.sh router を別端末で動かしていれば）それを使う。
  # 記録のたびにルータを立て直すと、スマホが毎回つなぎ直しになる
  if (echo > /dev/tcp/127.0.0.1/7447) 2>/dev/null; then
    echo "zenoh ルータは起動済み（7447）。それを使う"
    return
  fi
  zenoh_env ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 &
  ROUTER=$!
  sleep 2
  echo "zenoh ルータ起動 (pid $ROUTER)。スマホの接続先: tcp/<このPCのIP>:7447"
  ip -4 -o addr | awk '$4 ~ /^192\.168\.5\./ {print "  候補: tcp/" substr($4,1,index($4,"/")-1) ":7447"}'
}
stop_router() { [ -n "$ROUTER" ] && kill -INT $ROUTER 2>/dev/null; }

case "$1" in
router)
  start_router
  [ -z "$ROUTER" ] && exit 0
  trap 'stop_router; exit 0' INT TERM
  echo "ルータを動かし続けます（Ctrl-C で終了）。この間スマホはつながったまま"
  while true; do sleep 1; done ;;
probe)
  start_router
  echo "スマホの Imu を 5 秒ごとに確かめます（Ctrl-C で終了）"
  zenoh_env python3 /vibe/phone_rec.py --probe 5 &
  PR=$!
  trap 'kill -INT $PR; stop_router; exit 0' INT TERM
  while true; do
    sleep 10; echo "---- $(date +%T) 見えているトピック: $(zenoh_env ros2 topic list --no-daemon 2>/dev/null | grep -v -E '^/(rosout|parameter_events)$' | tr '\n' ' ')"
  done ;;
record)
  NAME=${2:?run 名を指定}
  OUT=/root/th_data/vibe/$NAME
  [ -e "$OUT" ] && { echo "$OUT は既にあります。別の名前にしてください"; exit 1; }
  mkdir -p "$OUT"
  start_router
  zenoh_env python3 /vibe/phone_rec.py "$OUT/phone" > >(tee "$OUT/phone.log") 2>&1 &
  PB=$!
  # th_robot とは /dev/shm を共有できない（IPC が private）。共有メモリで送られると 0 件になるので UDP だけで受ける
  FASTRTPS_DEFAULT_PROFILES_FILE=/vibe/fastdds_udp.xml ros2 bag record -o "$OUT/robot" $ROBOT_TOPICS > "$OUT/robot.log" 2>&1 &
  RB=$!
  echo "記録中: $OUT  （Ctrl-C で止める）"
  echo "  「記録開始: <トピック>」が出て件数が増え始めてから 3 秒止まったまま待ち、それから走らせる"
  stop() {
    kill -INT $PB $RB 2>/dev/null
    for i in $(seq 1 50); do kill -0 $PB 2>/dev/null || kill -0 $RB 2>/dev/null || break; sleep 0.2; done
    kill -TERM $PB $RB 2>/dev/null; wait $PB $RB 2>/dev/null   # 10 秒で閉じなければ TERM（bag は書けた分まで残る）
    stop_router
    [ -f $OUT/robot/metadata.yaml ] || ros2 bag reindex $OUT/robot sqlite3 >> $OUT/robot.log 2>&1
    chown -R 1000:1000 /root/th_data/vibe
    echo "保存しました: $OUT"
    echo "  スマホ: $(cat $OUT/phone/*.csv 2>/dev/null | grep -c '^[0-9]') 件"
    echo "  機体: $(grep -m1 -E '^ *message_count' $OUT/robot/metadata.yaml 2>/dev/null || echo 'metadata なし')"
    exit 0
  }
  trap stop INT TERM
  while true; do sleep 1; done ;;
*) echo "使い方: rec.sh router | rec.sh probe | rec.sh record NAME"; exit 1 ;;
esac
