#!/usr/bin/env bash
# crazyflie_test 예제를 실행하고 Crazyswarm2 RViz 설정으로 시각화한다.
# RViz 또는 launch가 종료되면 나머지 프로세스도 함께 정리한다.
#
# 사용법: ./launch.sh [ros2 launch 인자...]
#   예) ./launch.sh                       (launch.py 기본값 그대로)
#       ./launch.sh backend:=sim
#       ./launch.sh backend:=sim gazebo:=False  (Gazebo 비활성화)
#       ./launch.sh backend:=sim mode:=opticalflow_multi delivery_scene:=True
#       ./launch.sh mode:=mocap backend:=cflib
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

# 기존 컨테이너를 재시작해도 로그인 세션의 DISPLAY는 바뀔 수 있다.
cf_display_args=()
if [[ -n "${DISPLAY:-}" ]]; then
  cf_display_args+=(--env "DISPLAY=$DISPLAY")
fi

docker compose -f compose.yml exec "${cf_display_args[@]}" basic bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  export PYTHONPATH="/workspace/crazyflie-firmware/build:$PYTHONPATH"
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  if [[ "$LIBGL_ALWAYS_SOFTWARE" == 1 ]]; then
    # 호스트와 컨테이너의 Mesa 버전이 달라도 X11 버퍼를 전달할 수 있게 한다.
    export LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-1}"
  fi

  # 이전 이미지로 만든 컨테이너에서도 누락을 창을 띄우기 전에 알린다.
  cf_backend=cflib
  cf_gazebo=true
  cf_delivery=off
  cf_delivery_strategy=baseline
  cf_trajectory=off
  for cf_arg in "$@"; do
    case "$cf_arg" in
      backend:=*) cf_backend="${cf_arg#backend:=}" ;;
      gazebo:=*) cf_gazebo="${cf_arg#gazebo:=}" ;;
      delivery:=*) cf_delivery="${cf_arg#delivery:=}" ;;
      delivery_strategy:=*) cf_delivery_strategy="${cf_arg#delivery_strategy:=}" ;;
      trajectory:=*) cf_trajectory="${cf_arg#trajectory:=}" ;;
    esac
  done
  if [[ "$cf_backend" == sim && "${cf_gazebo,,}" == true ]]; then
    for cf_gz_package in ros_gz_sim ros_gz_bridge ros_gz_interfaces; do
      if ! ros2 pkg prefix "$cf_gz_package" >/dev/null 2>&1; then
        printf "Gazebo 패키지가 없습니다: %s\n" "$cf_gz_package" >&2
        printf "호스트의 ~/crazyflie에서 basic 이미지를 빌드하고 컨테이너를 갱신하세요:\n" >&2
        printf "  docker compose -f docker/compose.yml build basic\n" >&2
        printf "  docker compose -f docker/compose.yml up -d basic\n" >&2
        exit 1
      fi
    done
  fi
  if [[ "$cf_trajectory" != off ]]; then
    if ! python3 -c "import osqp" >/dev/null 2>&1; then
      printf "온라인 궤적 계획에 필요한 OSQP가 없습니다. basic 이미지를 갱신하세요:\n" >&2
      printf "  docker compose -f docker/compose.yml build basic\n" >&2
      printf "  docker compose -f docker/compose.yml up -d basic\n" >&2
      exit 1
    fi
  fi

  # 종료 신호를 launch의 자식 노드에도 전달할 수 있게 별도 프로세스 그룹을 만든다.
  setsid ros2 launch crazyflie_test launch.py "$@" &
  launch_pid=$!
  rviz_pid=
  cleanup() {
    trap - EXIT INT TERM
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    if [[ -n "$rviz_pid" ]]; then
      kill -TERM "$rviz_pid" 2>/dev/null || true
      # ROS 종료 후에도 Qt 창이 남는 경우 cleanup이 무한 대기하지 않게 한다.
      for cf_cleanup_step in {1..20}; do
        kill -0 "$rviz_pid" 2>/dev/null || break
        sleep 0.1
      done
      kill -KILL "$rviz_pid" 2>/dev/null || true
      wait "$rviz_pid" 2>/dev/null || true
    fi
    for cf_cleanup_step in {1..30}; do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for cf_cleanup_step in {1..20}; do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  }
  trap cleanup EXIT
  trap "exit 130" INT
  trap "exit 143" TERM
  cf_rviz_config=/workspace/cf_ws/src/crazyflie-basic/crazyswarm2/crazyflie/config/config.rviz
  if [[ "$cf_delivery" != off ]]; then
    cf_rviz_config=/workspace/cf_ws/src/crazyflie-basic/crazyflie_test/config/delivery.rviz
    if [[ "$cf_delivery_strategy" == baseline ]]; then
      cf_rviz_config=/workspace/cf_ws/src/crazyflie-basic/crazyflie_test/crazyflie_delivery/baseline/config/delivery.rviz
    fi
  fi
  if [[ "$cf_trajectory" != off ]]; then
    cf_rviz_config=/workspace/cf_ws/src/crazyflie-basic/crazyflie_test/config/trajectory.rviz
  fi
  # 예측 마커의 유효 시간도 SIL의 /clock과 같은 시계를 사용한다.
  cf_rviz_sim_time=false
  if [[ "$cf_backend" == sim ]]; then
    cf_rviz_sim_time=true
  fi
  rviz2 -d "$cf_rviz_config" --ros-args -p use_sim_time:="$cf_rviz_sim_time" &
  rviz_pid=$!
  cf_exit_code=0
  wait -n "$launch_pid" "$rviz_pid" || cf_exit_code=$?
  exit "$cf_exit_code"
' _ "$@"
