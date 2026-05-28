#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PCD_PATH=""
OPEN_RVIZ="true"
CONFIG_OUT="/tmp/landing_marsim_offline.yaml"
GRID_MAP_CONFIG="/home/fractal/landing/src/costmap_ws/src/fast_lio_global_grid_map/config/global_grid_map.yaml"
SAFELAND_CONFIG="/home/fractal/landing/src/costmap_ws/src/safeland/config/safeland.yaml"
INIT_X=""
INIT_Y=""
INIT_Z=""
INIT_YAW=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pcd) PCD_PATH="$2"; shift 2 ;;
    --open-rviz) OPEN_RVIZ="$2"; shift 2 ;;
    --config-out) CONFIG_OUT="$2"; shift 2 ;;
    --grid-map-config) GRID_MAP_CONFIG="$2"; shift 2 ;;
    --safeland-config) SAFELAND_CONFIG="$2"; shift 2 ;;
    --init-x) INIT_X="$2"; shift 2 ;;
    --init-y) INIT_Y="$2"; shift 2 ;;
    --init-z) INIT_Z="$2"; shift 2 ;;
    --init-yaw) INIT_YAW="$2"; shift 2 ;;
    *)
      echo "unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${PCD_PATH}" ]]; then
  echo "--pcd is required" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source /home/fractal/super_ws/devel/setup.bash
source /home/fractal/landing/devel/setup.bash --extend

config_args=(
  --pcd "${PCD_PATH}"
  --out "${CONFIG_OUT}"
)
if [[ -n "${INIT_X}" ]]; then config_args+=(--init-x "${INIT_X}"); fi
if [[ -n "${INIT_Y}" ]]; then config_args+=(--init-y "${INIT_Y}"); fi
if [[ -n "${INIT_Z}" ]]; then config_args+=(--init-z "${INIT_Z}"); fi
if [[ -n "${INIT_YAW}" ]]; then config_args+=(--init-yaw "${INIT_YAW}"); fi

CONFIG_PATH="$("${ROOT_DIR}/tools/marsim/make_marsim_offline_config.sh" "${config_args[@]}")"

exec roslaunch safeland marsim_safeland_offline.launch \
  marsim_config:="${CONFIG_PATH}" \
  grid_map_config:="${GRID_MAP_CONFIG}" \
  safeland_config:="${SAFELAND_CONFIG}" \
  open_rviz:="${OPEN_RVIZ}"
