#!/bin/bash
# ============================================================
# r2_bringup_tmux.sh  —  NR26 R2 一括起動 (tmux 分割ペイン版)
#
# 起動する launch ファイルをこのスクリプト内の LAUNCHES に列挙する。
# 形式: "<パッケージ名>/<ファイル名>.launch.py"
#
# 既定動作:
#   tmux の分割ペインで launch ごとに個別起動する。
#
# オプション:
#   --bg   バックグラウンド起動
# ============================================================

# ▼▼▼ 起動する launch ファイルを列挙 ▼▼▼
LAUNCHES=(
    r2_planner/task_manager.launch.py
    wall_detection/wall_detection_with_lidar.launch.py
    realsense_ros2/bringup.launch.py
    cube_detection/cube_detection.launch.py
    serial_bridge/serial_bridge.launch.py
    nr26_r2_hw_ctrl/bringup.launch.py
)
# ▲▲▲ ここまで ▲▲▲

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SETUP_BASH="$WS_ROOT/install/setup.bash"
MODE="tmux"

if [[ "${1:-}" == "--bg" ]]; then
    MODE="bg"
fi

if [[ ! -f "$SETUP_BASH" ]]; then
    echo "[ERROR] install/setup.bash が見つかりません: $SETUP_BASH"
    echo "       先に 'colcon build' を実行してください。"
    exit 1
fi

# shellcheck disable=SC1090
source "$SETUP_BASH"

VALID_SPECS=()
VALID_PATHS=()

for spec in "${LAUNCHES[@]}"; do
    pkg=$(echo "$spec" | cut -d'/' -f1)
    file=$(echo "$spec" | cut -d'/' -f2-)
    launch_path=$(find "$WS_ROOT/src" -path "*/${pkg}/launch/${file}" 2>/dev/null | head -1)

    if [[ -z "$launch_path" ]]; then
        echo "[WARN] launch ファイルが見つかりません: $spec (スキップ)"
        continue
    fi

    VALID_SPECS+=("$spec")
    VALID_PATHS+=("$launch_path")
done

if [[ ${#VALID_PATHS[@]} -eq 0 ]]; then
    echo "[ERROR] 起動できる launch ファイルがありませんでした。"
    exit 1
fi

if [[ "$MODE" == "tmux" ]]; then
    if ! command -v tmux >/dev/null 2>&1; then
        echo "[WARN] tmux が見つからないため、バックグラウンド起動にフォールバックします。"
        MODE="bg"
    fi
fi

if [[ "$MODE" == "tmux" ]]; then
    SESSION_NAME="r2_bringup_$(date +%Y%m%d_%H%M%S)"

    first_cmd="source \"$SETUP_BASH\"; export PYTHONDONTWRITEBYTECODE=1; ros2 launch \"${VALID_PATHS[0]}\""
    echo "[r2_bringup_tmux] 起動: ${VALID_SPECS[0]}"
    tmux new-session -d -s "$SESSION_NAME" "bash -lc '$first_cmd'"

    for ((i = 1; i < ${#VALID_PATHS[@]}; i++)); do
        cmd="source \"$SETUP_BASH\"; export PYTHONDONTWRITEBYTECODE=1; ros2 launch \"${VALID_PATHS[$i]}\""
        echo "[r2_bringup_tmux] 起動: ${VALID_SPECS[$i]}"
        tmux split-window -t "$SESSION_NAME":0 "bash -lc '$cmd'"
        tmux select-layout -t "$SESSION_NAME":0 tiled >/dev/null
    done

    echo ""
    echo "[r2_bringup_tmux] ${#VALID_PATHS[@]} 個のノードグループを tmux ペインで起動しました。"
    echo "                  セッション名: $SESSION_NAME"
    echo ""

    if [[ -n "${TMUX:-}" ]]; then
        tmux switch-client -t "$SESSION_NAME"
    else
        tmux attach -t "$SESSION_NAME"
    fi
    exit 0
fi

PIDS=()

cleanup() {
    echo ""
    echo "[r2_bringup_tmux] シャットダウン中..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null
        fi
    done
    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null
    done
    echo "[r2_bringup_tmux] 全ノード停止完了。"
    exit 0
}

trap cleanup SIGINT SIGTERM

for i in "${!VALID_PATHS[@]}"; do
    echo "[r2_bringup_tmux] 起動: ${VALID_SPECS[$i]}"
    ros2 launch "${VALID_PATHS[$i]}" &
    PIDS+=($!)
done

echo ""
echo "[r2_bringup_tmux] ${#PIDS[@]} 個のノードグループを起動しました。"
echo "                  Ctrl+C で全ノードを一括停止します。"
echo ""

wait
