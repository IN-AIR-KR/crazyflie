#!/usr/bin/env bash
# firmware + basic 컨테이너를 켠다 (최초 실행 시 이미지도 같이 빌드).
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

[ -f .env ] || { echo "UID=$(id -u)" > .env; echo "GID=$(id -g)" >> .env; }
xhost +local:docker >/dev/null 2>&1 || true

docker compose -f compose.yml up -d --build
