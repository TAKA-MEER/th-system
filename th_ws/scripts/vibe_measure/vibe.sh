#!/bin/bash
# 昇降機上部の揺れ計測（スマホ Conduit の IMU ＋ 機体の速度・台車 IMU）。ホストで実行する。
# 実機の th_robot には触れず、別コンテナ（th_vibe）で動く。手順は README.md。
#   ./vibe.sh build              計測用イメージを作る（初回と th_system_msgs を変えたとき）
#   ./vibe.sh router             zenoh ルータを立てっぱなしにする（計測の間、別端末で動かしておく）
#   ./vibe.sh probe              スマホがつながっているかを見る
#   ./vibe.sh record <名前>      記録（Ctrl-C で終了）。th_ws/data/vibe/<名前>/ に残る
#   ./vibe.sh analyze <名前>...  解析。同じ場所に summary.md・timeseries.png・psd.png を出す
#   ./vibe.sh selftest           合成データで解析が正しく動くかを見る
D=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$D/../.." && pwd)
TTY=$([ -t 0 ] && echo -it)
RUN="docker run --rm $TTY --network host -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10} -e PHONE_DOMAIN=${PHONE_DOMAIN:-0}
     -v $WS/data:/root/th_data -v $D:/vibe th_vibe:latest"
ROS='source /opt/ros/humble/setup.bash; source /root/vibe_ws/install/setup.bash'
case "$1" in
  build)
    ctx=$(mktemp -d) && cp "$D/Dockerfile" "$ctx/" && cp -r "$WS/src/th_system_msgs" "$ctx/" &&
      docker build --network host -t th_vibe:latest "$ctx"; rc=$?; rm -rf "$ctx"; exit $rc ;;
  router|probe|record) exec $RUN bash /vibe/rec.sh "$@" ;;
  analyze) shift; exec $RUN bash -c "$ROS; python3 /vibe/analyze.py \"\$@\"; chown -R 1000:1000 /root/th_data/vibe" _ "$@" ;;
  selftest)
    exec $RUN bash -c "$ROS; rm -rf /root/th_data/vibe/_selftest; python3 /vibe/synth.py /root/th_data/vibe/_selftest 90 &&
      python3 /vibe/analyze.py _selftest | grep -E '上部\(スマホ\)|上部: 前後|遅れ|倍'; rm -rf /root/th_data/vibe/_selftest
      echo '期待: 向き -90°・一致度 0.95 以上・オドメトリ比 1.0 前後・前後の山の先頭が 2.4 Hz 前後・遅れ +100〜140 ms'" ;;
  *) sed -n 2,10p "$0" ;;
esac
