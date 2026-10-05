#!/usr/bin/env bash
# 실행 중인 crazyflie_server에 연결해 단일 기체 키보드 텔레옵을 실행한다.
# 키 입력을 직접 읽으므로 이 스크립트는 대화형 터미널에서 실행해야 한다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

docker compose -f compose.yml exec basic bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  # 소스는 bind mount되어 있으므로 새 console entry point의 colcon 재빌드
  # 여부와 관계없이 방금 추가한 모듈을 바로 실행할 수 있다.
  cd /workspace/cf_ws/src/crazyflie-basic/crazyflie_test
  python3 -m crazyflie_test.single_cf "$@"
' _ "$@"
