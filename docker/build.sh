#!/usr/bin/env bash
# firmware + basic 이미지를 빌드한다. UID/GID가 안 맞으면 build args가 틀어지므로
# .env가 없으면 먼저 만든다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

[ -f .env ] || { echo "UID=$(id -u)" > .env; echo "GID=$(id -g)" >> .env; }

docker compose -f compose.yml build
