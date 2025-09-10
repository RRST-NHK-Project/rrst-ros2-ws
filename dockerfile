# ROS 2 Jazzy のベースイメージ
FROM osrf/ros:jazzy-desktop

# 作業ディレクトリ
WORKDIR /ros2_ws

# ワークスペースコピー
COPY . src

# 依存関係をインストール
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    apt-get install figlet -y 

# ビルド（事前ビルド）
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc


# 環境設定
ENV ROS_WS=/ros2_ws
ENV ROS_PACKAGE_PATH=/ros2_ws/src


