"""
依存関係の一括インストール

＊実行方法＊
cd ~/ros2_ws/src
sudo chmod +x setup.sh
./setup.sh

"""

# いつもの
sudo apt update -y
sudo apt upgrade -y

# figletのインストール
sudo apt install -y figlet

# 以下LD19用
cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh
sudo apt install libudev-dev
sudo apt install -y ros-${ROS_DISTRO}-diagnostic-updater
sudo apt install -y ros-${ROS_DISTRO}-nav2-util