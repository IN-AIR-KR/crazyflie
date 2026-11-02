#!/usr/bin/env bash
# 실행 중인 crazyflie_server에 연결해 편대 키보드 조종(이륙/전진/착륙)을 실행한다.
# 키 입력을 직접 읽으므로 이 스크립트는 대화형 터미널에서 실행해야 한다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

docker compose -f compose.yml exec basic bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  cd /workspace/cf_ws/src/crazyflie-basic/crazyflie_test
  python3 -m crazyflie_test.formation_teleop "$@"
' _ "$@"
