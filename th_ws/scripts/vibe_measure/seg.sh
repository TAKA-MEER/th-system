#!/bin/bash
# 1 区間を記録つきで走らせる（ホストで実行）。ルータは vibe.sh router か vibe_router コンテナで起動済みのこと
#   ./seg.sh <記録名> <drive.py の引数...>
D=$(cd "$(dirname "$0")" && pwd); WS=$(cd "$D/../.." && pwd)
NAME=$1; shift
docker rm -f vibe_rec >/dev/null 2>&1
docker run -d --name vibe_rec --network host -e ROS_DOMAIN_ID=10 -v $WS/data:/root/th_data -v $D:/vibe th_vibe:latest bash /vibe/rec.sh record $NAME >/dev/null
timeout 20 bash -c 'until docker logs vibe_rec 2>&1 | grep -q "conduit/imu: [0-9]"; do sleep 0.5; done' || { echo "スマホのデータが来ない"; docker stop vibe_rec; exit 1; }
sleep 4
docker run --rm --network host -e ROS_DOMAIN_ID=10 -e FASTRTPS_DEFAULT_PROFILES_FILE=/vibe/fastdds_udp.xml -v $D:/vibe th_vibe:latest \
  bash -c 'source /opt/ros/humble/setup.bash; source /root/vibe_ws/install/setup.bash; python3 /vibe/drive.py "$@"' _ "$@"
rc=$?
sleep 5
docker stop -t 30 vibe_rec >/dev/null; docker logs vibe_rec 2>&1 | tail -2; docker rm vibe_rec >/dev/null
exit $rc
