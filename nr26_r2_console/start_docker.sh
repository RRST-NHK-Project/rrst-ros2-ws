#!/bin/bash

# R2 Console - Docker用起動スクリプト
# Dockerコンテナ内でR2 Consoleを起動します

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="${SCRIPT_DIR}"

# Dockerコンテナ内で起動スクリプトを実行
docker run --rm -it \
  -v "${PROJECT_DIR}:/app/nr26_r2_console" \
  -p 3000:3000 \
  -e NODE_ENV=development \
  --name r2_console \
  nr26_r2_console:latest \
  bash -c "cd /app/nr26_r2_console && npm install && npm start"
