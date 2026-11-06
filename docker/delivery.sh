#!/usr/bin/env bash
# preview: Gazebo 4대 + RViz 할당 경로. run: 같은 구성에서 실제 시뮬레이션 배송 실행.
set -euo pipefail
cf_delivery_mode="${1:-preview}"
case "$cf_delivery_mode" in
  preview|run) ;;
  *) printf '사용법: %s [preview|run] [추가 launch 인자...]\n' "$0" >&2; exit 2 ;;
esac
if [[ $# -gt 0 ]]; then shift; fi
exec "$(dirname "${BASH_SOURCE[0]}")/launch.sh" \
  backend:=sim mode:=delivery gazebo:=True delivery_scene:=True \
  delivery:="$cf_delivery_mode" "$@"
