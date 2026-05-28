#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_LIST=""
RESULT_CSV="${ROOT_DIR}/marsim_benchmark_results/round3_results.csv"
TIMEOUT="6"
STABLE_SECS="1"
OPEN_RVIZ="false"
WARMUP_SECS="5"

# Fixed from round2 best combination.
SLOPE_THRESHOLD="0.15"
DEPRESSION_THRESHOLD="0.30"
LANDING_VALID_RATIO_THRESHOLD="0.98"
LANDING_HEIGHT_RANGE_THRESHOLD="0.05"
STEP_THRESHOLD="0.10"
LANDING_SIZE="0.3"
LANDING_SAFETY_MARGIN="0.0"

GRID_RESOLUTION_VALUES="0.10,0.12,0.15"
VOXEL_LEAF_VALUES="0.05,0.08,0.10"
MIN_CELL_POINTS_VALUES="5,3,2"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --maps) MAP_LIST="$2"; shift 2 ;;
    --csv) RESULT_CSV="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --stable-secs) STABLE_SECS="$2"; shift 2 ;;
    --open-rviz) OPEN_RVIZ="$2"; shift 2 ;;
    --warmup-secs) WARMUP_SECS="$2"; shift 2 ;;
    --grid-resolution-values) GRID_RESOLUTION_VALUES="$2"; shift 2 ;;
    --voxel-leaf-values) VOXEL_LEAF_VALUES="$2"; shift 2 ;;
    --min-cell-points-values) MIN_CELL_POINTS_VALUES="$2"; shift 2 ;;
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

IFS=',' read -r -a grid_resolution_array <<< "${GRID_RESOLUTION_VALUES}"
IFS=',' read -r -a voxel_leaf_array <<< "${VOXEL_LEAF_VALUES}"
IFS=',' read -r -a min_cell_points_array <<< "${MIN_CELL_POINTS_VALUES}"

while IFS= read -r map_path; do
  [[ -z "${map_path}" ]] && continue
  map_path="$(realpath "${map_path}")"

  for grid_resolution in "${grid_resolution_array[@]}"; do
    for voxel_leaf_size in "${voxel_leaf_array[@]}"; do
      for min_cell_points in "${min_cell_points_array[@]}"; do
        tag="$(basename "${map_path}")__res${grid_resolution}__voxel${voxel_leaf_size}__minpts${min_cell_points}"
        marsim_cfg="${TMP_DIR}/${tag}.marsim.yaml"
        grid_map_cfg="${TMP_DIR}/${tag}.grid_map.yaml"
        safeland_cfg="${TMP_DIR}/${tag}.safeland.yaml"

        "${ROOT_DIR}/tools/marsim/make_marsim_offline_config.sh" \
          --pcd "${map_path}" \
          --out "${marsim_cfg}" >/dev/null

        cp "${ROOT_DIR}/src/costmap_ws/src/fast_lio_global_grid_map/config/global_grid_map.yaml" "${grid_map_cfg}"
        sed -i \
          -e "s/^resolution:.*/resolution: ${grid_resolution}/" \
          -e "s/^voxel_leaf_size_x:.*/voxel_leaf_size_x: ${voxel_leaf_size}/" \
          -e "s/^voxel_leaf_size_y:.*/voxel_leaf_size_y: ${voxel_leaf_size}/" \
          -e "s/^voxel_leaf_size_z:.*/voxel_leaf_size_z: ${voxel_leaf_size}/" \
          "${grid_map_cfg}"

        cp "${ROOT_DIR}/src/costmap_ws/src/safeland/config/safeland.yaml" "${safeland_cfg}"
        sed -i \
          -e "s/^slope_threshold:.*/slope_threshold: ${SLOPE_THRESHOLD}/" \
          -e "s/^depression_score_threshold:.*/depression_score_threshold: ${DEPRESSION_THRESHOLD}/" \
          -e "s/^landing_valid_ratio_threshold:.*/landing_valid_ratio_threshold: ${LANDING_VALID_RATIO_THRESHOLD}/" \
          -e "s/^landing_height_range_threshold:.*/landing_height_range_threshold: ${LANDING_HEIGHT_RANGE_THRESHOLD}/" \
          -e "s/^step_threshold:.*/step_threshold: ${STEP_THRESHOLD}/" \
          -e "s/^landing_size:.*/landing_size: ${LANDING_SIZE}/" \
          -e "s/^landing_safety_margin:.*/landing_safety_margin: ${LANDING_SAFETY_MARGIN}/" \
          -e "s/^min_cell_points:.*/min_cell_points: ${min_cell_points}/" \
          "${safeland_cfg}"

        roslaunch safeland marsim_safeland_offline.launch \
          marsim_config:="${marsim_cfg}" \
          grid_map_config:="${grid_map_cfg}" \
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
          --landing-height-range-threshold "${LANDING_HEIGHT_RANGE_THRESHOLD}" \
          --step-threshold "${STEP_THRESHOLD}" \
          --landing-size "${LANDING_SIZE}" \
          --landing-safety-margin "${LANDING_SAFETY_MARGIN}" \
          --grid-resolution "${grid_resolution}" \
          --voxel-leaf-size "${voxel_leaf_size}" \
          --min-cell-points "${min_cell_points}" || true

        kill "${launch_pid}" >/dev/null 2>&1 || true
        wait "${launch_pid}" >/dev/null 2>&1 || true
        sleep 1
      done
    done
  done
done < "${MAP_LIST}"
