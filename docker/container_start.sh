#!/usr/bin/env bash
# 빌드된 이미지로 firmware + basic 컨테이너를 켠다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

xhost +local:docker >/dev/null 2>&1 || true

docker compose -f compose.yml up -d
