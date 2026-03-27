#!/bin/bash

# R2 Console - 起動スクリプト
# npm startでGUIを起動します

set -e

# スクリプトの場所を取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONSOLE_DIR="${SCRIPT_DIR}"

cd "${CONSOLE_DIR}"

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
echo "ブラウザで http://localhost:3000 を開いてください"
echo "終了するには Ctrl+C を押してください"
echo ""

npm start
