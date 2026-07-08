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
    ros-humble-vision-msgs \
    ros-humble-pcl-ros \
    ros-humble-laser-geometry \
    ros-humble-nav2-lifecycle-manager \
    xterm \
    gazebo \
    git \
    vim \
    wget \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# ── Python パッケージ ────────────────────────────────────────────────
RUN pip3 install numpy scipy transforms3d websockets

# ── rosdep 初期化 ────────────────────────────────────────────────────
RUN rosdep init || true && rosdep update

# ── ワークスペース作成 & ソースコードのクローン ───────────────────────
WORKDIR /root/th_ws/src

# aptにないパッケージをGitHubからクローン
RUN git clone https://github.com/Slamtec/sllidar_ros2.git

# person_tracker 本番実装: human_kenchi (LiDARのみの脚検出・追跡ワークスペース)
# リポジトリ直下に独自の src/ を持つ多パッケージ構成だが、colcon は th_ws/src 以下を
# 再帰的に探索するため human_kenchi/src/{multiple_observation_kalman_filter,
# multiple_sensor_person_tracking, leg_detection_bringup} はそのままビルド対象になる。
RUN git clone https://github.com/TAKA-MEER/human_kenchi.git

# DR-SPAAM 脚検出器 (TeamSOBITS, human_kenchi の install.sh が参照する外部依存)
RUN git clone -b jazzy-devel https://github.com/TeamSOBITS/2d_lidar_person_detection.git \
    human_kenchi/src/2d_lidar_person_detection
RUN cd human_kenchi/src/2d_lidar_person_detection/dr_spaam && python3 setup.py install
# 重みファイル (ckpt_jrdb_ann_ft_dr_spaam_e20.pth, 約360MB) はイメージに含めない。
# docker-compose.yml の bind mount 経由でホストの th_ws/dr_spaam_weights/ から供給する。
# 詳細は README.md の「DR-SPAAM 重みファイルの配置」を参照。

# ── 依存関係の解決 & ビルド ───────────────────────────────────────────
WORKDIR /root/th_ws

# クローンしたパッケージに必要な依存ツールを rosdep で自動インストール
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# ROS 2環境を読み込んだ上で colcon build を実行
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# ── 便利エイリアス ───────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
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