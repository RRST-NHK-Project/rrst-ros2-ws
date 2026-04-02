#!/bin/bash

# R2 Console - 起動スクリプト
# rosbridge と GUI を起動します

set -e

# スクリプトの場所を取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONSOLE_DIR="${SCRIPT_DIR}"

cd "${CONSOLE_DIR}"

BRIDGE_PORT="${BRIDGE_PORT:-9090}"
BRIDGE_PID=""
CONSOLE_BACKEND_PORT="${CONSOLE_BACKEND_PORT:-3031}"
BACKEND_PID=""

start_rosbridge() {
  if [ -n "${VIRTUAL_ENV:-}" ]; then
    echo "Python仮想環境を検出: ${VIRTUAL_ENV}"
    echo "rosbridge はシステムPython環境で起動します"

    local CLEAN_PATH="${PATH}"
    CLEAN_PATH="${CLEAN_PATH#"${VIRTUAL_ENV}/bin:"}"

    (
      unset VIRTUAL_ENV
      export PATH="${CLEAN_PATH}"
      exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${BRIDGE_PORT}
    ) >/tmp/r2_console_rosbridge.log 2>&1 &
  else
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${BRIDGE_PORT} >/tmp/r2_console_rosbridge.log 2>&1 &
  fi

  BRIDGE_PID=$!
}

cleanup() {
  if [ -n "${BACKEND_PID}" ] && kill -0 "${BACKEND_PID}" >/dev/null 2>&1; then
    echo ""
    echo "console backend を停止しています..."
    kill "${BACKEND_PID}" >/dev/null 2>&1 || true
    wait "${BACKEND_PID}" 2>/dev/null || true
  fi

  if [ -n "${BRIDGE_PID}" ] && kill -0 "${BRIDGE_PID}" >/dev/null 2>&1; then
    echo ""
    echo "rosbridge を停止しています..."
    kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
    wait "${BRIDGE_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if command -v ros2 >/dev/null 2>&1; then
  if command -v ss >/dev/null 2>&1 && ss -ltn "sport = :${BRIDGE_PORT}" | grep -q ":${BRIDGE_PORT}"; then
    echo "rosbridge は既にポート ${BRIDGE_PORT} で起動中です"
  else
    echo "rosbridge を起動中... (port: ${BRIDGE_PORT})"
    start_rosbridge
    sleep 1

    if ! kill -0 "${BRIDGE_PID}" >/dev/null 2>&1; then
      echo "rosbridge の起動に失敗しました。ログ: /tmp/r2_console_rosbridge.log"
      exit 1
    fi
  fi
else
  echo "ros2 コマンドが見つかりません。先に ROS2 環境を有効化してください。"
  exit 1
fi

if command -v node >/dev/null 2>&1; then
  if command -v ss >/dev/null 2>&1 && ss -ltn "sport = :${CONSOLE_BACKEND_PORT}" | grep -q ":${CONSOLE_BACKEND_PORT}"; then
    echo "console backend は既にポート ${CONSOLE_BACKEND_PORT} で起動中です"
  else
    echo "console backend を起動中... (port: ${CONSOLE_BACKEND_PORT})"
    CONSOLE_BACKEND_PORT="${CONSOLE_BACKEND_PORT}" node ./tools/console_backend.js >/tmp/r2_console_backend.log 2>&1 &
    BACKEND_PID=$!
    sleep 1

    if ! kill -0 "${BACKEND_PID}" >/dev/null 2>&1; then
      echo "console backend の起動に失敗しました。ログ: /tmp/r2_console_backend.log"
      exit 1
    fi
  fi
else
  echo "node コマンドが見つかりません。console backend を起動できません。"
fi

# node_modulesが存在しない場合はインストール
if [ ! -d "node_modules" ]; then
  echo "依存関係をインストール中..."
  npm install
fi

# 開発サーバーを起動
echo "=================================="
echo "R2 Console を起動しています..."
echo "=================================="
echo ""
echo "rosbridge: ws://localhost:${BRIDGE_PORT}"
echo "console backend: http://localhost:${CONSOLE_BACKEND_PORT}"
echo "ブラウザで http://localhost:3000 を開いてください"
echo "終了するには Ctrl+C を押してください"
echo ""

npm start
