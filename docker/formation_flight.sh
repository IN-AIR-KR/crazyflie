#!/usr/bin/env bash
# 실행 중인 crazyflie_server에 연결해 편대 비행 예제를 실행한다.
# 사용법: ./formation_flight.sh [--shape line|v|circle[,...]] [--mode auto|absolute|broadcast] ...
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

docker compose -f compose.yml exec basic bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  # 소스는 bind mount되어 있으므로 새 console entry point의 colcon 재빌드
  # 여부와 관계없이 방금 추가한 모듈을 바로 실행할 수 있다.
  cd /workspace/cf_ws/src/crazyflie-basic/crazyflie_test
  python3 -m crazyflie_test.formation_flight "$@"
' _ "$@"
