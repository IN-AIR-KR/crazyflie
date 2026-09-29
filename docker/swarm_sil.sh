#!/usr/bin/env bash
# Stage 1 SIL — Crazyswarm2 sim(4기) 위에서 군집 TinyMPC 를 돌린다.
#
# 사용법:
#   ./swarm_sil.sh server [gazebo:=False]      터미널 1: sim 서버 + RViz2 (+ Gazebo)
#   ./swarm_sil.sh run [--controller central]  터미널 2: MPC 노드 (끝나면 sil_log.npz 저장)
#   ./swarm_sil.sh eval [sil_log.npz]          로그를 Python 시뮬레이션과 같은 지표로 평가
#
# 전제: stage1 이 /workspace/stage1 로 마운트돼 있어야 한다(compose.yml). 마운트를 새로
# 추가했다면 container_start.sh 로 컨테이너를 다시 띄워야 반영된다.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
cmd="${1:-}"; shift || true

case "$cmd" in
  server)
    docker compose -f compose.yml exec basic bash -c '
      set -e
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      export PYTHONPATH="/workspace/crazyflie-firmware/build:$PYTHONPATH"
      ros2 launch /workspace/stage1/sil/swarm_sim.launch.py "$@"
    ' _ "$@" ;;
  run)
    docker compose -f compose.yml exec basic bash -c '
      set -e
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      python3 /workspace/stage1/sil/swarm_mpc_node.py "$@"
    ' _ "$@" ;;
  eval)
    log="${1:-/workspace/stage1/sil/sil_log.npz}"
    docker compose -f compose.yml exec basic bash -c "
      cd /workspace/stage1 && python3 sil/sil_eval.py $log
    " ;;
  *)
    sed -n 2,11p "$0"; exit 1 ;;
esac
