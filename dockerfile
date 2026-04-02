# ROS 2 Jazzy のベースイメージ
FROM osrf/ros:jazzy-desktop

# 作業ディレクトリ
WORKDIR /ros2_ws

# ワークスペースコピー
COPY . src

# 依存関係をインストール
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-rosdep \
    figlet && \
    (rosdep init || true) && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*

# ビルド（事前ビルド）
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc


# 環境設定
ENV ROS_WS=/ros2_ws
ENV ROS_PACKAGE_PATH=/ros2_ws/src


