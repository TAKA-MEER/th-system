FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=10

# ── 基本ツール ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-twist-mux \
    ros-humble-rosbridge-suite \
    ros-humble-micro-ros-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    ros-humble-xacro \
    xterm \
    gazebo \
    git \
    vim \
    wget \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ── Python パッケージ ────────────────────────────────────────────────
RUN pip3 install numpy scipy transforms3d

# ── rosdep 初期化 ────────────────────────────────────────────────────
RUN rosdep init || true && rosdep update

# ── ワークスペース作成 & ソースコードのクローン ───────────────────────
WORKDIR /root/th_ws/src

# aptにないパッケージをGitHubからクローン
RUN git clone https://github.com/Slamtec/sllidar_ros2.git

# ── 依存関係の解決 & ビルド ───────────────────────────────────────────
WORKDIR /root/th_ws

# クローンしたパッケージに必要な依存ツールを rosdep で自動インストール
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# ROS 2環境を読み込んだ上で colcon build を実行
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# ── micro-ROS Agent (micro_ros_setup 経由の公式手順でビルド) ───────────
# 生の Micro-ROS-Agent リポジトリを colcon build するだけだと ROS グラフへの
# 橋渡し(rclcpp/rmw 統合)に必要なセットアップが欠け、XRCEセッションは確立
# できてもトピック/ノードが ros2 topic list 等に一切現れない症状が起きるため、
# 公式の micro_ros_setup ワークフローでビルドする。
WORKDIR /root/microros_ws
RUN mkdir -p src && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
        apt-get update && \
        rosdep install --from-paths src --ignore-src -y && \
        rm -rf /var/lib/apt/lists/* && \
        colcon build --symlink-install && \
        source install/setup.bash && \
        ros2 run micro_ros_setup create_agent_ws.sh && \
        ros2 run micro_ros_setup build_agent.sh"
WORKDIR /root/th_ws

# ── 便利エイリアス ───────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/microros_ws/install/setup.bash ] && source /root/microros_ws/install/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/th_ws/install/setup.bash ] && source /root/th_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=10" >> /root/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbs='colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias si='source install/setup.bash'" >> /root/.bashrc && \
    echo "alias ri='rosdep install --from-paths src --ignore-src -r -y'" >> /root/.bashrc && \
    echo "alias bringup='ros2 launch th_bringup bringup.launch.py'" >> /root/.bashrc && \
    echo "alias slam='ros2 launch th_bringup slam.launch.py'" >> /root/.bashrc && \
    echo "alias nav='ros2 launch th_bringup nav.launch.py'" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]