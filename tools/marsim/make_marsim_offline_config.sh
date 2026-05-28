#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TEMPLATE="${ROOT_DIR}/src/costmap_ws/src/safeland/config/marsim_mid360_template.yaml"

PCD_PATH=""
OUT_PATH="/tmp/landing_marsim_offline.yaml"
INIT_X=""
INIT_Y=""
INIT_Z=""
INIT_ROLL="3.1415926"
INIT_PITCH="-0.5235988"
INIT_YAW="0.0"
POLAR_RES="0.2"
DOWNSAMPLE_RES="0.1"
VERTICAL_FOV="60.0"
SENSING_HORIZON="30.0"
SENSING_RATE="10"
LIDAR_TYPE="3"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pcd) PCD_PATH="$2"; shift 2 ;;
    --out) OUT_PATH="$2"; shift 2 ;;
    --init-x) INIT_X="$2"; shift 2 ;;
    --init-y) INIT_Y="$2"; shift 2 ;;
    --init-z) INIT_Z="$2"; shift 2 ;;
    --init-roll) INIT_ROLL="$2"; shift 2 ;;
    --init-pitch) INIT_PITCH="$2"; shift 2 ;;
    --init-yaw) INIT_YAW="$2"; shift 2 ;;
    --polar-res) POLAR_RES="$2"; shift 2 ;;
    --downsample-res) DOWNSAMPLE_RES="$2"; shift 2 ;;
    --vertical-fov) VERTICAL_FOV="$2"; shift 2 ;;
    --sensing-horizon) SENSING_HORIZON="$2"; shift 2 ;;
    --sensing-rate) SENSING_RATE="$2"; shift 2 ;;
    --lidar-type) LIDAR_TYPE="$2"; shift 2 ;;
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

PCD_PATH="$(realpath "${PCD_PATH}")"
if [[ ! -f "${PCD_PATH}" ]]; then
  echo "pcd not found: ${PCD_PATH}" >&2
  exit 1
fi

if [[ -z "${INIT_X}" || -z "${INIT_Y}" || -z "${INIT_Z}" ]]; then
  VIEWPOINT_LINE="$(head -n 12 "${PCD_PATH}" | awk '/^VIEWPOINT /{print $2, $3, $4; exit}')"
  if [[ -n "${VIEWPOINT_LINE}" ]]; then
    read -r VIEW_X VIEW_Y VIEW_Z <<< "${VIEWPOINT_LINE}"
    [[ -z "${INIT_X}" ]] && INIT_X="${VIEW_X}"
    [[ -z "${INIT_Y}" ]] && INIT_Y="${VIEW_Y}"
    if [[ -z "${INIT_Z}" ]]; then
      INIT_Z="$(awk -v z="${VIEW_Z}" 'BEGIN{printf "%.6f", z + 1.5}')"
    fi
  fi
fi

[[ -z "${INIT_X}" ]] && INIT_X="0.0"
[[ -z "${INIT_Y}" ]] && INIT_Y="0.0"
[[ -z "${INIT_Z}" ]] && INIT_Z="1.5"

ESCAPED_PCD="$(printf '%s' "${PCD_PATH}" | sed 's/[&/]/\\&/g')"

mkdir -p "$(dirname "${OUT_PATH}")"
sed \
  -e "s/__PCD_PATH__/${ESCAPED_PCD}/g" \
  -e "s/__INIT_X__/${INIT_X}/g" \
  -e "s/__INIT_Y__/${INIT_Y}/g" \
  -e "s/__INIT_Z__/${INIT_Z}/g" \
  -e "s/__INIT_ROLL__/${INIT_ROLL}/g" \
  -e "s/__INIT_PITCH__/${INIT_PITCH}/g" \
  -e "s/__INIT_YAW__/${INIT_YAW}/g" \
  -e "s/__POLAR_RES__/${POLAR_RES}/g" \
  -e "s/__DOWNSAMPLE_RES__/${DOWNSAMPLE_RES}/g" \
  -e "s/__VERTICAL_FOV__/${VERTICAL_FOV}/g" \
  -e "s/__SENSING_HORIZON__/${SENSING_HORIZON}/g" \
  -e "s/__SENSING_RATE__/${SENSING_RATE}/g" \
  -e "s/__LIDAR_TYPE__/${LIDAR_TYPE}/g" \
  "${TEMPLATE}" > "${OUT_PATH}"

echo "${OUT_PATH}"
