#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_LIST=""
RESULT_CSV="${ROOT_DIR}/marsim_benchmark_results/round2_results.csv"
TIMEOUT="6"
STABLE_SECS="1"
OPEN_RVIZ="false"
WARMUP_SECS="5"

SLOPE_THRESHOLD="0.15"
DEPRESSION_THRESHOLD="0.30"
LANDING_VALID_RATIO_THRESHOLD="0.98"

LANDING_HEIGHT_RANGE_VALUES="0.05,0.08,0.10"
STEP_VALUES="0.05,0.08,0.10"
LANDING_SIZE_VALUES="0.5,0.4,0.3"
LANDING_SAFETY_MARGIN_VALUES="0.10,0.05,0.00"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --maps) MAP_LIST="$2"; shift 2 ;;
    --csv) RESULT_CSV="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --stable-secs) STABLE_SECS="$2"; shift 2 ;;
    --open-rviz) OPEN_RVIZ="$2"; shift 2 ;;
    --warmup-secs) WARMUP_SECS="$2"; shift 2 ;;
    --slope-threshold) SLOPE_THRESHOLD="$2"; shift 2 ;;
    --depression-threshold) DEPRESSION_THRESHOLD="$2"; shift 2 ;;
    --landing-valid-ratio-threshold) LANDING_VALID_RATIO_THRESHOLD="$2"; shift 2 ;;
    --landing-height-range-values) LANDING_HEIGHT_RANGE_VALUES="$2"; shift 2 ;;
    --step-values) STEP_VALUES="$2"; shift 2 ;;
    --landing-size-values) LANDING_SIZE_VALUES="$2"; shift 2 ;;
    --landing-safety-margin-values) LANDING_SAFETY_MARGIN_VALUES="$2"; shift 2 ;;
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

IFS=',' read -r -a landing_height_range_array <<< "${LANDING_HEIGHT_RANGE_VALUES}"
IFS=',' read -r -a step_array <<< "${STEP_VALUES}"
IFS=',' read -r -a landing_size_array <<< "${LANDING_SIZE_VALUES}"
IFS=',' read -r -a landing_safety_margin_array <<< "${LANDING_SAFETY_MARGIN_VALUES}"

while IFS= read -r map_path; do
  [[ -z "${map_path}" ]] && continue
  map_path="$(realpath "${map_path}")"

  for landing_height_range in "${landing_height_range_array[@]}"; do
    for step_threshold in "${step_array[@]}"; do
      for landing_size in "${landing_size_array[@]}"; do
        for landing_safety_margin in "${landing_safety_margin_array[@]}"; do
          tag="$(basename "${map_path}")__dz${landing_height_range}__step${step_threshold}__size${landing_size}__margin${landing_safety_margin}"
          marsim_cfg="${TMP_DIR}/${tag}.marsim.yaml"
          safeland_cfg="${TMP_DIR}/${tag}.safeland.yaml"

          "${ROOT_DIR}/tools/marsim/make_marsim_offline_config.sh" \
            --pcd "${map_path}" \
            --out "${marsim_cfg}" >/dev/null

          cp "${ROOT_DIR}/src/costmap_ws/src/safeland/config/safeland.yaml" "${safeland_cfg}"
          sed -i \
            -e "s/^slope_threshold:.*/slope_threshold: ${SLOPE_THRESHOLD}/" \
            -e "s/^depression_score_threshold:.*/depression_score_threshold: ${DEPRESSION_THRESHOLD}/" \
            -e "s/^landing_valid_ratio_threshold:.*/landing_valid_ratio_threshold: ${LANDING_VALID_RATIO_THRESHOLD}/" \
            -e "s/^landing_height_range_threshold:.*/landing_height_range_threshold: ${landing_height_range}/" \
            -e "s/^step_threshold:.*/step_threshold: ${step_threshold}/" \
            -e "s/^landing_size:.*/landing_size: ${landing_size}/" \
            -e "s/^landing_safety_margin:.*/landing_safety_margin: ${landing_safety_margin}/" \
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
            --slope-threshold "${SLOPE_THRESHOLD}" \
            --depression-threshold "${DEPRESSION_THRESHOLD}" \
            --landing-valid-ratio-threshold "${LANDING_VALID_RATIO_THRESHOLD}" \
            --landing-height-range-threshold "${landing_height_range}" \
            --step-threshold "${step_threshold}" \
            --landing-size "${landing_size}" \
            --landing-safety-margin "${landing_safety_margin}" || true

          kill "${launch_pid}" >/dev/null 2>&1 || true
          wait "${launch_pid}" >/dev/null 2>&1 || true
          sleep 1
        done
      done
    done
  done
done < "${MAP_LIST}"
