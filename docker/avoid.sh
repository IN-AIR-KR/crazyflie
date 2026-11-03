#!/usr/bin/env bash
# 실행 중인 crazyflie_server에 연결해 멀티레인저 장애물 회피를 실행한다.
# --selftest 는 서버 없이(센서 없이) 로직만 검증한다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

docker compose -f compose.yml exec basic bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  cd /workspace/cf_ws/src/crazyflie-basic/crazyflie_test
  python3 -m crazyflie_test.avoid "$@"
' _ "$@"
