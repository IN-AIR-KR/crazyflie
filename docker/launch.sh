#!/usr/bin/env bash
# crazyflie_test 예제를 실행하고 rviz2로 시각화한다.
# backend:=sim이면 Gazebo Sim도 함께 실행하고 드론 pose를 동기화한다.
# rviz2 창을 닫으면 백그라운드로 띄운 ros2 launch도 같이 종료된다.
#
# 사용법: ./launch.sh [ros2 launch 인자...]
#   예) ./launch.sh                       (launch.py 기본값 그대로)
#       ./launch.sh backend:=sim
#       ./launch.sh backend:=sim gazebo:=False  (Gazebo 비활성화)
#       ./launch.sh mode:=mocap backend:=cflib
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

docker compose -f compose.yml exec basic bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  export PYTHONPATH="/workspace/crazyflie-firmware/build:$PYTHONPATH"
  ros2 launch crazyflie_test launch.py "$@" &
  launch_pid=$!
  trap "kill $launch_pid 2>/dev/null" EXIT
  rviz2 -d /workspace/cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflie.rviz
' _ "$@"
