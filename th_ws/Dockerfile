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
    iputils-ping \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# ── Python パッケージ ────────────────────────────────────────────────
RUN pip3 install numpy scipy transforms3d websockets ruamel.yaml

# ── rosdep 初期化 ────────────────────────────────────────────────────
RUN rosdep init || true && rosdep update

# ── ワークスペース作成 & ソースコードのクローン ───────────────────────
WORKDIR /root/th_ws/src

# aptにないパッケージをGitHubからクローン
RUN git clone https://github.com/Slamtec/sllidar_ros2.git

# person_tracker 本番実装: human_kenchi (TAKA-MEER/human_kenchi, LiDARのみの脚検出・追跡)
# 由来リポジトリはプライベートのため Dockerfile から git clone できない。
# multiple_observation_kalman_filter / multiple_sensor_person_tracking /
# leg_detection_bringup の3パッケージは th_ws/src にそのままコミット（vendoring）してある。
# アップデートを取り込む場合は該当パッケージを手動で再取得して上書きすること。

# DR-SPAAM 脚検出器 (TeamSOBITS, human_kenchi の install.sh が参照する外部依存。公開リポジトリ)
# dr_spaam の setup.py は依存を宣言していない (install_requires 無し) ため torch を別途導入する。
# dr_spaam_param.yaml で use_gpu:false (CPU 推論) のため CPU 版の軽量な wheel で十分。
RUN pip3 install torch --index-url https://download.pytorch.org/whl/cpu
RUN git clone -b jazzy-devel https://github.com/TeamSOBITS/2d_lidar_person_detection.git
# 公式チェックポイントは GPU (CUDA) 保存のため、CPU 専用機では torch.load が
# 「Attempting to deserialize object on a CUDA device」で失敗する。
# map_location を明示することで use_gpu:false (CPU 推論) でも読み込めるようにする。
# (実機検証で判明。上流にはまだ取り込まれていないためソースに直接パッチする)
RUN sed -i \
    's/ckpt = torch.load(ckpt_file)/ckpt = torch.load(ckpt_file, map_location=torch.device("cuda" if gpu else "cpu"))/' \
    2d_lidar_person_detection/dr_spaam/dr_spaam/detector.py
# setuptools (numpy/scipy/torch 導入時に新しいものへ更新される) は
# packaging>=22 の canonicalize_version(strip_trailing_zero=...) を前提とするが、
# ベースイメージの packaging は 21.3 のままで非互換のため egg_info 生成が失敗する。
# 明示的に packaging を更新してから install する。
RUN pip3 install -U packaging
RUN cd 2d_lidar_person_detection/dr_spaam && pip3 install .
# 重みファイルはイメージに含めない。docker-compose.yml の bind mount 経由で
# ホストの th_ws/dr_spaam_weights/ から供給する。
# 詳細は docs/setup.md「5. PC: コンテナのビルド」を参照。

# ── 依存関係の解決 & ビルド ───────────────────────────────────────────
WORKDIR /root/th_ws

# クローンしたパッケージに必要な依存ツールを rosdep で自動インストール
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# ROS 2環境を読み込んだ上で colcon build を実行
# 注意: --symlink-install は使わない。ここでビルドする外部依存 (sllidar_ros2,
# dr_spaam_ros 等) はイメージの src/ を参照するが、実行時は docker-compose.yml の
# bind mount (./src:/root/th_ws/src) でホストの th_ws/src に置き換わるため、
# symlink-install が作るシンボリックリンク (install/ → build/ → イメージ内 src/) が
# 実行時に壊れてしまう。コピーインストールにして install/ を自己完結させる。
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

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