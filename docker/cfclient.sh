#!/usr/bin/env bash
# firmware 컨테이너에서 cfclient(드론 제어 GUI)를 띄운다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
docker compose -f compose.yml exec firmware cfclient
