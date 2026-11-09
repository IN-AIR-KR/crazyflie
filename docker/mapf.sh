#!/usr/bin/env bash
# crazyflie_mapf 실행 모음. 호스트의 ~/crazyflie 어디서나 실행한다.
#
# 사용법: docker/mapf.sh <명령> [ros2 launch 인자...]
#
#   준비
#     build                 이미지(OSQP 포함) + crazyflie_mapf 빌드. 처음 한 번, 새 파일 추가 시
#     up                    basic 컨테이너 켜기 (GPU가 있으면 GPU 구성)
#     shell                 컨테이너 셸 열기 (ROS 환경 설정됨)
#
#   실행 (Ctrl+C·stop·Gazebo/RViz 창 닫기 모두 같은 종료 처리. 실기체가 있으면
#         시뮬레이션 종료 → 실기체 착륙 → 모터 정지 → 전원 끔. MAPF_POWER_OFF=0이면 전원은 켜 둠)
#     sim                   시뮬레이션 5대만 (3.6m, mirror 설정, real:=none)
#     mirror                시뮬레이션 5대 + 무선 응답하는 실기체가 따라 함 (real:=auto)
#     mirror real:=cf3,cf5  지정한 실기체만 따라 함
#     mirror mocap:=false   모캡 없이: 기체 자체 위치 추정(Flow/Lighthouse/Loco 덱 필요)으로 따라 함
#     hardware              실기체만 (config/hardware)
#     legacy [none|static|dynamic]  기존 시뮬레이션 7대 (9m)
#
#   점검
#     scan                  지금 무선으로 잡히는 기체 (설정 이름, 주소, 배터리, 덱)
#
#   조작 (실행 중 다른 터미널에서)
#     start                 이륙 시작 (실기체가 있을 때만 필요)
#     land                  전체 착륙 (도착하면 자동 착륙도 함)
#     emergency             비상 정지: 실행 중인 시뮬레이션·실기체 전체 모터 즉시 정지
#     estop <번호...>       지정한 실기체만 강제 정지 (예: estop 2 / estop 2 5 / estop all)
#                           서버가 없어도 무선으로 직접 보낸다. 날던 기체는 떨어진다
#     status                도착·실기체 상태 1회 출력
#     stop                  실행 중인 launch 강제 종료
#
#   예) docker/mapf.sh sim
#       docker/mapf.sh mirror real:=cf3,cf5
#       docker/mapf.sh sim gazebo:=false        (RViz만)
#       MAPF_GPU=0 docker/mapf.sh sim           (GPU 없이 소프트웨어 렌더링)
set -euo pipefail
cf_script="$(realpath "${BASH_SOURCE[0]}")"
cd "$(dirname "$cf_script")"

cf_command="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi

# GPU: MAPF_GPU=1/0으로 강제, 비우면 호스트 nvidia-smi와 docker nvidia 런타임으로 판단한다.
cf_gpu="${MAPF_GPU:-}"
if [[ -z "$cf_gpu" ]]; then
  cf_gpu=0
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
     && docker info 2>/dev/null | grep -qi 'runtimes:.*nvidia'; then
    cf_gpu=1
  fi
fi
cf_compose=(docker compose -f compose.yml)
cf_rendering=true
if [[ "$cf_gpu" == 1 ]]; then
  cf_compose+=(-f compose.gpu.yml)
  cf_rendering=false
fi

cf_ros_env='source /opt/ros/jazzy/setup.bash && source /workspace/cf_ws/install/setup.bash'

# 컨테이너 안에서 시뮬레이션 launch가 끝난 뒤 실행한다(작은따옴표 없이 유지).
# 실기체 쪽(mirror_real.launch.py)은 별도 세션이라 시뮬레이션과 함께 죽지 않고,
# mirror 노드가 시뮬레이션이 사라진 것을 보고 착륙 -> 모터 정지 -> 종료한다. 그 뒤 전원을 끈다.
cf_finish='
  real="^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch crazyflie_mapf mirror_real[.]launch[.]py"
  if pgrep -f "$real" >/dev/null; then
    echo "[mapf] 시뮬레이션 종료: 실기체 착륙과 모터 정지를 기다립니다 (최대 25초)"
    for i in {1..250}; do pgrep -f "$real" >/dev/null || break; sleep 0.1; done
    if pgrep -f "$real" >/dev/null; then
      echo "[mapf] 시간 초과: 실기체 쪽을 강제 종료합니다 (docker/mapf.sh estop 으로 모터를 끄세요)"
      pkill -INT -f "$real"; sleep 3
    fi
  fi
  pkill -KILL -f "^(gz sim|/opt/ros/jazzy/lib/(rviz2|ros_gz_bridge)/|(/usr/bin/python3 )?/workspace/cf_ws/install/)" 2>/dev/null || true
  rm -f /tmp/mapf_real_plan.json
  if [[ "${MAPF_POWER_OFF:-1}" == 1 ]]; then
    python3 -m crazyflie_mapf.mirror.shutdown power-off 2>/dev/null | grep -v "^WARNING" || true
  else
    rm -f /tmp/mapf_real.json
  fi
'

cf_up() {
  # 같은 구성이면 그대로 두고, GPU 여부가 바뀌었으면 컨테이너를 다시 만든다.
  "${cf_compose[@]}" up -d basic >/dev/null 2>&1 || "${cf_compose[@]}" up -d basic
  if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true
  fi
}

cf_exec() {
  local tty=(-T)
  if [[ -t 0 && -t 1 ]]; then tty=(); fi
  "${cf_compose[@]}" exec "${tty[@]}" ${DISPLAY:+--env "DISPLAY=$DISPLAY"} --env "MAPF_POWER_OFF=${MAPF_POWER_OFF:-1}" basic bash -c "$@"
}

cf_control() {
  # ros2 topic pub --once waits for a subscriber; without a running sim/mirror it would wait forever.
  cf_exec "$cf_ros_env && if timeout 5 ros2 topic pub --once -w 1 /fleet/control std_msgs/msg/String '{data: \"{\\\"op\\\": \\\"$1\\\"}\"}' >/dev/null 2>\&1; then echo '$1 sent'; else echo '$1 보내지 못함: 받을 실행기가 없습니다 (sim/mirror/hardware가 실행 중이 아님)'; exit 1; fi"
}

cf_launch() {
  # $1: launch 파일, 나머지: launch 인자
  cf_up
  cf_exec '
    set -e
    '"$cf_ros_env"'
    if ! python3 -c "import osqp" >/dev/null 2>&1; then
      printf "OSQP가 없습니다. 호스트에서 이미지를 다시 빌드하세요: docker/mapf.sh build\n" >&2
      exit 1
    fi
    setsid ros2 launch crazyflie_mapf "$@" &
    launch_pid=$!
    cleanup() {
      trap - EXIT TERM
      trap "" INT  # 실기체 착륙 대기는 최대 25초로 제한되어 있다.
      kill -INT -- "-$launch_pid" 2>/dev/null || true
      for i in {1..50}; do kill -0 -- "-$launch_pid" 2>/dev/null || break; sleep 0.1; done
      kill -KILL -- "-$launch_pid" 2>/dev/null || true
      wait "$launch_pid" 2>/dev/null || true
      '"$cf_finish"'
    }
    trap cleanup EXIT
    trap "exit 130" INT
    trap "exit 143" TERM
    wait "$launch_pid"
  ' _ "$@"
}

case "$cf_command" in
  build)
    "${cf_compose[@]}" build basic
    cf_up
    cf_exec 'source /opt/ros/jazzy/setup.bash && cd /workspace/cf_ws && colcon build --symlink-install --packages-select crazyflie_mapf'
    ;;
  up)
    cf_up
    printf 'basic 컨테이너 실행 중 (GPU: %s)\n' "$([[ $cf_gpu == 1 ]] && echo 사용 || echo 미사용)"
    ;;
  shell)
    cf_up
    cf_exec "$cf_ros_env && cd /workspace/cf_ws && exec bash"
    ;;
  sim)
    cf_launch mirror.launch.py real:=none software_rendering:="$cf_rendering" "$@"
    ;;
  mirror)
    cf_launch mirror.launch.py software_rendering:="$cf_rendering" "$@"
    ;;
  hardware)
    cf_launch hardware.launch.py "$@"
    ;;
  legacy)
    cf_mode="${1:-none}"
    if [[ $# -gt 0 ]]; then shift; fi
    case "$cf_mode" in
      none) cf_file=no_obstacles.launch.py ;;
      static) cf_file=static_obstacles.launch.py ;;
      dynamic) cf_file=dynamic_obstacles.launch.py ;;
      *) printf 'legacy 모드는 none / static / dynamic입니다.\n' >&2; exit 2 ;;
    esac
    cf_launch "$cf_file" software_rendering:="$cf_rendering" "$@"
    ;;
  scan)
    cf_up
    cf_exec "$cf_ros_env && if pgrep -f '^(/workspace/cf_ws/install/crazyflie_server_cpp/|/usr/bin/python3 /workspace/cf_ws/install/crazyflie_server_py/)' >/dev/null; then
      echo '실기체 서버가 동글을 쓰는 중입니다. mirror/hardware를 끄고(Ctrl+C 또는 docker/mapf.sh stop) 다시 실행하세요.'; exit 1; fi
      python3 -u -m crazyflie_mapf.mirror.scan $* 2>&1 | grep --line-buffered -vE '^(WARNING|Error no LogEntry)'"
    ;;
  start|land|emergency)
    cf_control "$cf_command"
    ;;
  status)
    cf_exec "$cf_ros_env && echo '[fleet]' && timeout 10 ros2 topic echo --once --field data /fleet/status std_msgs/msg/String 2>/dev/null | python3 -c '
import json, sys
d = json.loads(sys.stdin.readline())
print(\"finished:\", d[\"finished\"], \" reached:\", d[\"reached\"], \"/\", len(d[\"robots\"]), \" mission_time:\", round(d[\"mission_time\"], 2))
for name, goal in d[\"robots\"].items(): print(\" \", name, goal[\"status\"])
' 2>/dev/null || { if pgrep -f '^/usr/bin/python3 /workspace/cf_ws/install/crazyflie_mapf/lib/crazyflie_mapf/fleet_manager' >/dev/null; then echo '  (시뮬레이션은 실행 중이지만 상태를 아직 못 받음: 시작 직후면 몇 초 뒤 다시 status)'; else echo '  (시뮬레이션 실행 중이 아님)'; fi; }
      echo '[mirror]' && timeout 5 ros2 topic echo --once --field data /mirror/status std_msgs/msg/String 2>/dev/null | python3 -c '
import json, sys
d = json.loads(sys.stdin.readline())
print(\"  위치:\", \"Qualisys 모캡\" if d.get(\"mode\") != \"onboard\" else \"기체 자체 추정 (모캡 없음)\")
for name, r in d[\"robots\"].items(): print(\" \", name, r[\"state\"], r[\"reason\"] or r[\"blocked\"] or (\"ready (start 가능)\" if r[\"state\"] == \"WAITING\" else \"\"))
' 2>/dev/null || { if pgrep -f '^/usr/bin/python3 /workspace/cf_ws/install/crazyflie_mapf/lib/crazyflie_mapf/mirror_node' >/dev/null; then echo '  (mirror 실행 중이지만 상태를 아직 못 받음: 몇 초 뒤 다시 status)'; else echo '  (실기체 연결 없음: real:=none이거나, 실기체가 꺼져 있어 못 찾았거나, 실행 중이 아님)'; fi; }"
    ;;
  stop)
    # 시뮬레이션 launch만 끈다. 실기체 쪽은 착륙·모터 정지 후 스스로 끝난다(cf_finish).
    # 패턴은 ^(실행 파일 경로)로 고정해, 같은 문자열을 명령줄에 가진 스크립트 셸을 죽이지 않는다.
    cf_exec "$cf_ros_env"'
      sim="^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch crazyflie_mapf (mirror|no_obstacles|static_obstacles|dynamic_obstacles|simulation|hardware)[.]launch[.]py"
      pkill -INT -f "$sim" || true
      for i in {1..60}; do pgrep -f "$sim" >/dev/null || break; sleep 0.1; done
      '"$cf_finish"'
      echo stopped'
    ;;
  estop|kill)
    if [[ $# -eq 0 ]]; then
      printf '사용법: docker/mapf.sh estop <기체 번호...>   예) estop 2 / estop 2 5 / estop all\n' >&2; exit 2
    fi
    cf_up
    cf_exec "$cf_ros_env && python3 -m crazyflie_mapf.mirror.estop $* 2>&1 | grep -vE '^WARNING'"
    ;;
  help|-h|--help)
    sed -n '2,36p' "$cf_script" | sed 's/^# \{0,1\}//'
    ;;
  *)
    printf '알 수 없는 명령: %s (docker/mapf.sh help)\n' "$cf_command" >&2
    exit 2
    ;;
esac
