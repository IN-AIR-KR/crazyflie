#!/usr/bin/env bash
# basic 컨테이너 안으로 들어간다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
docker compose -f compose.yml exec basic bash
