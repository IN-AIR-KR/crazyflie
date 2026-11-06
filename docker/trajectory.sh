#!/usr/bin/env bash
# 목적지 교차 실험: run은 공통 full-state 실행기, preview는 궤적 표시만 실행.
set -euo pipefail
cf_trajectory_operation="${1:-run}"
case "$cf_trajectory_operation" in
  preview|run) ;;
  *) printf '사용법: %s [preview|run] [trajectory_scenario:=crossing2|crossing4] [추가 launch 인자...]\n' "$0" >&2; exit 2 ;;
esac
if [[ $# -gt 0 ]]; then shift; fi
cf_trajectory_scenario=crossing2
for cf_trajectory_argument in "$@"; do
  case "$cf_trajectory_argument" in
    trajectory_scenario:=*) cf_trajectory_scenario="${cf_trajectory_argument#trajectory_scenario:=}" ;;
  esac
done
case "$cf_trajectory_scenario" in
  crossing2) cf_trajectory_profile=trajectory2 ;;
  crossing4) cf_trajectory_profile=trajectory4 ;;
  *) printf 'trajectory_scenario는 crossing2 또는 crossing4입니다.\n' >&2; exit 2 ;;
esac
exec "$(dirname "${BASH_SOURCE[0]}")/launch.sh" \
  backend:=sim mode:="$cf_trajectory_profile" gazebo:=True \
  delivery:=off delivery_scene:=False trajectory:="$cf_trajectory_operation" \
  trajectory_scenario:="$cf_trajectory_scenario" "$@"
