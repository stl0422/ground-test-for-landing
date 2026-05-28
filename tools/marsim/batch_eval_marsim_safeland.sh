#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_LIST=""
RESULT_CSV="${ROOT_DIR}/marsim_benchmark_results/results.csv"
TIMEOUT="25"
STABLE_SECS="2"
SLOPE_VALUES="0.12,0.15,0.18"
DEP_VALUES="0.2,0.3,0.4"
VALID_RATIO_VALUES="0.95,0.98,1.00"
OPEN_RVIZ="false"
WARMUP_SECS="8"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --maps) MAP_LIST="$2"; shift 2 ;;
    --csv) RESULT_CSV="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --stable-secs) STABLE_SECS="$2"; shift 2 ;;
    --slope-values) SLOPE_VALUES="$2"; shift 2 ;;
    --dep-values) DEP_VALUES="$2"; shift 2 ;;
    --valid-ratio-values) VALID_RATIO_VALUES="$2"; shift 2 ;;
    --open-rviz) OPEN_RVIZ="$2"; shift 2 ;;
    --warmup-secs) WARMUP_SECS="$2"; shift 2 ;;
    *)
      echo "unknown arg: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${MAP_LIST}" || ! -f "${MAP_LIST}" ]]; then
  echo "--maps must point to a text file containing one PCD path per line" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source /home/fractal/super_ws/devel/setup.bash
source /home/fractal/landing/devel/setup.bash --extend

mkdir -p "$(dirname "${RESULT_CSV}")"
TMP_DIR="$(mktemp -d)"
trap 'pkill -f "roslaunch safeland marsim_safeland_offline.launch" >/dev/null 2>&1 || true; rm -rf "${TMP_DIR}"' EXIT

IFS=',' read -r -a slope_array <<< "${SLOPE_VALUES}"
IFS=',' read -r -a dep_array <<< "${DEP_VALUES}"
IFS=',' read -r -a valid_ratio_array <<< "${VALID_RATIO_VALUES}"

while IFS= read -r map_path; do
  [[ -z "${map_path}" ]] && continue
  map_path="$(realpath "${map_path}")"

  for slope in "${slope_array[@]}"; do
    for dep in "${dep_array[@]}"; do
      for valid_ratio in "${valid_ratio_array[@]}"; do
        tag="$(basename "${map_path}")__s${slope}__d${dep}__v${valid_ratio}"
        marsim_cfg="${TMP_DIR}/${tag}.marsim.yaml"
        safeland_cfg="${TMP_DIR}/${tag}.safeland.yaml"

        "${ROOT_DIR}/tools/marsim/make_marsim_offline_config.sh" \
          --pcd "${map_path}" \
          --out "${marsim_cfg}" >/dev/null

        cp "${ROOT_DIR}/src/costmap_ws/src/safeland/config/safeland.yaml" "${safeland_cfg}"
        sed -i \
          -e "s/^slope_threshold:.*/slope_threshold: ${slope}/" \
          -e "s/^depression_score_threshold:.*/depression_score_threshold: ${dep}/" \
          -e "s/^landing_valid_ratio_threshold:.*/landing_valid_ratio_threshold: ${valid_ratio}/" \
          "${safeland_cfg}"

        roslaunch safeland marsim_safeland_offline.launch \
          marsim_config:="${marsim_cfg}" \
          safeland_config:="${safeland_cfg}" \
          open_rviz:="${OPEN_RVIZ}" >/tmp/"${tag}".log 2>&1 &
        launch_pid=$!

        sleep "${WARMUP_SECS}"
        rosrun safeland collect_safeland_metrics.py \
          --csv "${RESULT_CSV}" \
          --timeout "${TIMEOUT}" \
          --stable-secs "${STABLE_SECS}" \
          --tag "${tag}" \
          --pcd "${map_path}" \
          --slope-threshold "${slope}" \
          --depression-threshold "${dep}" \
          --landing-valid-ratio-threshold "${valid_ratio}" || true

        kill "${launch_pid}" >/dev/null 2>&1 || true
        wait "${launch_pid}" >/dev/null 2>&1 || true
        sleep 2
      done
    done
  done
done < "${MAP_LIST}"
