
# 依存関係の一括インストール
# ＊実行方法＊
# cd ~/ros2_ws/src/setup
# sudo chmod +x setup.sh
# ./setup.sh



# いつもの
sudo apt-get update -y

# figletのインストール
sudo apt-get install -y figlet

# 以下LD19用
cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh
sudo apt-get install -y ros-jazzy-nav2-lifecycle-manager
sudo apt-get install -y libudev-dev
sudo apt-get install -y ros-jazzy-diagnostic-updater
sudo apt-get install -y ros-jazzy-nav2-util

# 他のスクリプトに一括で実行権限を渡す
sudo chmod +x "${HOME}/ros2_ws/src/setup/"*.sh || true #エラーが出ても無視
