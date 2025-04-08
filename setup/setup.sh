
# 依存関係の一括インストール
# ＊実行方法＊
# cd ~/ros2_ws/src/setup
# sudo chmod +x setup.sh
# ./setup.sh



# いつもの
sudo apt update -y

# figletのインストール
sudo apt install -y figlet

# 以下LD19用
cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh
sudo apt install ros-jazzy-nav2-lifecycle-manager
sudo apt install libudev-dev
sudo apt install -y ros-${ROS_DISTRO}-diagnostic-updater
sudo apt install -y ros-${ROS_DISTRO}-nav2-util

# 他のスクリプトに一括で実行権限を渡す
sudo chmod +x ~/ros2_ws/src/setup/*.sh