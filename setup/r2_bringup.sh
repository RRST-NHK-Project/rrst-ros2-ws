#!/bin/bash
# ============================================================
# r2_bringup.sh  —  NR26 R2 一括起動スクリプト
#
# 起動する launch ファイルをこのスクリプト内の LAUNCHES に列挙する。
# 形式: "<パッケージ名>/<ファイル名>.launch.py"
# ============================================================

# ▼▼▼ 起動する launch ファイルを列挙 ▼▼▼
LAUNCHES=(
    nr26_r2_hw_ctrl/bringup.launch.py
    realsense_ros2/bringup.launch.py
    cube_detection/cube_detection.launch.py
    kfs_akaze_detection/kfs_akaze_detection.launch.py
    kfs_cube_fusion/kfs_cube_fusion.launch.py
    r2_planner/task_manager.launch.py
)
# ▲▲▲ ここまで ▲▲▲

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SETUP_BASH="$WS_ROOT/install/setup.bash"

if [[ ! -f "$SETUP_BASH" ]]; then
    echo "[ERROR] install/setup.bash が見つかりません: $SETUP_BASH"
    echo "       先に 'colcon build' を実行してください。"
    exit 1
fi

# shellcheck disable=SC1090
source "$SETUP_BASH"

PIDS=()

cleanup() {
    echo ""
    echo "[r2_bringup] シャットダウン中..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null
        fi
    done
    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null
    done
    echo "[r2_bringup] 全ノード停止完了。"
    exit 0
}

trap cleanup SIGINT SIGTERM

for spec in "${LAUNCHES[@]}"; do
    pkg=$(echo "$spec" | cut -d'/' -f1)
    file=$(echo "$spec" | cut -d'/' -f2-)
    launch_path=$(find "$WS_ROOT/src" -path "*/${pkg}/launch/${file}" 2>/dev/null | head -1)

    if [[ -z "$launch_path" ]]; then
        echo "[WARN] launch ファイルが見つかりません: $spec (スキップ)"
        continue
    fi

    echo "[r2_bringup] 起動: $spec"
    ros2 launch "$launch_path" &
    PIDS+=($!)
done

if [[ ${#PIDS[@]} -eq 0 ]]; then
    echo "[ERROR] 起動できる launch ファイルがありませんでした。"
    exit 1
fi

echo ""
echo "[r2_bringup] ${#PIDS[@]} 個のノードグループを起動しました。"
echo "             Ctrl+C で全ノードを一括停止します。"
echo ""

wait
