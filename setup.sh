sudo apt update -y
sudo apt upgrade -y
sudo apt install -y figlet
cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/
./create_udev_rules.sh
sudo apt install libudev-dev
sudo apt install -y ros-${ROS_DISTRO}-diagnostic-updater
sudo apt install -y ros-${ROS_DISTRO}-nav2-util