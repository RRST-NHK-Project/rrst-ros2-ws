#!/bin/bash

set -e

echo "システムアップデートと依存パッケージのインストール "
sudo apt update
sudo apt install -y python3-pip python3-venv python3-colcon-common-extensions

echo "Python仮想環境 (venv) の作成 "
# pythonの権限の関係で仮想環境を作る必要性有
# 既存のvenvがあれば削除して作り直し（クリーンインストール用）
if [ -d "~/venv" ]; then
    rm -rf ~/venv
fi
python3 -m venv ~/venv
source ~/venv/bin/activate

echo "機械学習・画像処理ライブラリのインストール"
pip install --upgrade pip
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install numpy opencv-python setuptools

echo "ROS2 依存関係の解決"
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "パッケージのクリーンビルド"
rm -rf build/kfs_pkg install/kfs_pkg
colcon build --packages-select kfs_pkg --symlink-install

echo "環境設定の追加"
# ターミナル起動時に自動でsourceするようにbashrcへ追記（任意）
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "完了！新しいターミナルを開くか 'source ~/venv/bin/activate && source install/setup.bash' を実行してください。"