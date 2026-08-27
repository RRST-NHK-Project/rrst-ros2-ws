#!/bin/bash
# soki_simパッケージの依存関係(package.xmlのexec_depend)をrosdep経由で
# インストールする。command_gui_nodeが必要とするpython3-tkもここに含まれる。
#
# 使い方:
#   src/soki_sim/scripts/install_deps.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdepが見つかりません。先にpython3-rosdepをインストールしてください" >&2
    exit 1
fi

if [ ! -d /etc/ros/rosdep/sources.list.d ] || [ -z "$(ls -A /etc/ros/rosdep/sources.list.d 2>/dev/null)" ]; then
    sudo rosdep init
fi
rosdep update

rosdep install --from-paths "$PKG_DIR" --ignore-src -y
