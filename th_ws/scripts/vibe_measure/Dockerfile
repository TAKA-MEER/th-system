# 揺れ計測用。th_robot のイメージに rmw_zenoh（Conduit の受け口）と日本語フォントを足し、
# 機体の bag を読み書きするため th_system_msgs だけをビルドする。vibe.sh build で作る
FROM th_ws-th_robot:latest
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rmw-zenoh-cpp fonts-noto-cjk && fc-cache -f && rm -rf /var/lib/apt/lists/*
COPY th_system_msgs /root/vibe_ws/src/th_system_msgs
RUN bash -c 'source /opt/ros/humble/setup.bash && cd /root/vibe_ws && colcon build --packages-select th_system_msgs'
