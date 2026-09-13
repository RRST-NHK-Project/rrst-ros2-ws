#!/bin/bash
# nhk27_fieldパッケージの依存関係(package.xmlのexec_depend)をrosdep経由で
# インストールする。ros_gz_sim/ros_gz_bridge(Gazebo Harmonic本体)を含む。
#
# 注意: このスクリプト自体はsudoを付けずに実行すること。apt-get install等
# 必要な箇所は内部でrosdepがsudoを呼ぶ。先頭からsudoを付けると環境変数
# (ROS_DISTRO等)が引き継がれずrosdepが失敗する。
#
# 使い方:
#   src/nhk27_field/scripts/install_deps.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
ROSDISTRO="${ROS_DISTRO:-jazzy}"

if [ "$(id -u)" -eq 0 ]; then
    echo "このスクリプトはsudoを付けずに実行してください(内部で必要時にsudoを呼びます)" >&2
    exit 1
fi

if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdepが見つかりません。先にpython3-rosdepをインストールしてください" >&2
    exit 1
fi

if [ ! -d /etc/ros/rosdep/sources.list.d ] || [ -z "$(ls -A /etc/ros/rosdep/sources.list.d 2>/dev/null)" ]; then
    sudo rosdep init
fi
rosdep update --rosdistro "$ROSDISTRO"

rosdep install --from-paths "$PKG_DIR" --ignore-src -y --rosdistro "$ROSDISTRO"
