#!/usr/bin/env bash
# =============================================================================
# batch_eval_marsim_safeland_round4.sh
#
# 第4轮批量评估：聚焦于以下三项优化的联合验证
#
#   1. 环境复杂度提升
#      通过 marsim 仿真参数控制传感器视角、视距、扫描速率等来模拟
#      不同难度的降落场景（开阔地、局部遮挡、近距低空等）
#
#   2. 算法稳定性与效率（网格分辨率 + 降采样）
#      基于 round3 最优组合 (res=0.10, voxel=0.05, min_pts=3)
#      在此基础上测试更细 (0.08) 和略粗 (0.12) 的分辨率，
#      以及更激进降采样 (voxel=0.08) 与更精细降采样 (voxel=0.03)
#
#   3. 应急备降区机制
#      对比 enable_alt_landing=true/false，以及不同 relax_factor (1.2/1.5/2.0)
#      量化：在复杂地形下备降区的命中率和安全得分下降幅度
#
# 输出：
#   ${RESULT_CSV} — 原始记录（每组配置一行）
#   可用 summarize_round4.py 进行汇总分析
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_LIST=""
RESULT_CSV="${ROOT_DIR}/marsim_benchmark_results/round4_results.csv"
TIMEOUT="8"
STABLE_SECS="2"
OPEN_RVIZ="false"
WARMUP_SECS="6"

# ---- 固定自 round3 最优参数 ----
SLOPE_THRESHOLD="0.15"
DEPRESSION_THRESHOLD="0.30"
LANDING_VALID_RATIO_THRESHOLD="0.98"
LANDING_HEIGHT_RANGE_THRESHOLD="0.05"
STEP_THRESHOLD="0.10"
LANDING_SIZE="0.3"
LANDING_SAFETY_MARGIN="0.0"

# ---- 第4轮扫描维度 ----
# 网格分辨率（优化算法效率）
GRID_RESOLUTION_VALUES="0.08,0.10,0.12"
# 体素降采样大小（精度与计算开销平衡）
VOXEL_LEAF_VALUES="0.03,0.05,0.08"
# 备降区放宽系数（应急安全机制）
ALT_RELAX_VALUES="1.2,1.5,2.0"
# 是否启用备降区（对比组）
ENABLE_ALT_VALUES="true,false"

# ---- 环境复杂度：marsim 传感模拟参数 ----
# polar_res：雷达角分辨率（越大越稀疏，模拟远距/弱信号环境）
POLAR_RES_VALUES="0.2,0.4"
# sensing_horizon：传感视距（越小覆盖范围越有限，模拟遮挡/低空环境）
SENSING_HORIZON_VALUES="20,30"

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
    --alt-relax-values) ALT_RELAX_VALUES="$2"; shift 2 ;;
    --enable-alt-values) ENABLE_ALT_VALUES="$2"; shift 2 ;;
    --polar-res-values) POLAR_RES_VALUES="$2"; shift 2 ;;
    --sensing-horizon-values) SENSING_HORIZON_VALUES="$2"; shift 2 ;;
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
IFS=',' read -r -a alt_relax_array <<< "${ALT_RELAX_VALUES}"
IFS=',' read -r -a enable_alt_array <<< "${ENABLE_ALT_VALUES}"
IFS=',' read -r -a polar_res_array <<< "${POLAR_RES_VALUES}"
IFS=',' read -r -a sensing_horizon_array <<< "${SENSING_HORIZON_VALUES}"

while IFS= read -r map_path; do
  [[ -z "${map_path}" ]] && continue
  map_path="$(realpath "${map_path}")"

  for polar_res in "${polar_res_array[@]}"; do
    for sensing_horizon in "${sensing_horizon_array[@]}"; do
      for grid_resolution in "${grid_resolution_array[@]}"; do
        for voxel_leaf_size in "${voxel_leaf_array[@]}"; do
          for alt_relax in "${alt_relax_array[@]}"; do
            for enable_alt in "${enable_alt_array[@]}"; do

              # 当 enable_alt=false 时，只需要跑一次（relax 无意义）
              if [[ "${enable_alt}" == "false" && "${alt_relax}" != "${alt_relax_array[0]}" ]]; then
                continue
              fi

              tag="$(basename "${map_path}")__polar${polar_res}__horizon${sensing_horizon}__res${grid_resolution}__voxel${voxel_leaf_size}__relax${alt_relax}__alt${enable_alt}"
              marsim_cfg="${TMP_DIR}/${tag}.marsim.yaml"
              grid_map_cfg="${TMP_DIR}/${tag}.grid_map.yaml"
              safeland_cfg="${TMP_DIR}/${tag}.safeland.yaml"
              metrics_csv="${TMP_DIR}/${tag}.metrics.csv"

              # 生成 marsim 配置（带传感器参数，模拟不同环境复杂度）
              "${ROOT_DIR}/tools/marsim/make_marsim_offline_config.sh" \
                --pcd "${map_path}" \
                --out "${marsim_cfg}" \
                --polar-res "${polar_res}" \
                --sensing-horizon "${sensing_horizon}" >/dev/null

              # 生成 grid_map 配置（调整分辨率和降采样）
              cp "${ROOT_DIR}/src/costmap_ws/src/fast_lio_global_grid_map/config/global_grid_map.yaml" \
                 "${grid_map_cfg}"
              sed -i \
                -e "s/^resolution:.*/resolution: ${grid_resolution}/" \
                -e "s/^voxel_leaf_size_x:.*/voxel_leaf_size_x: ${voxel_leaf_size}/" \
                -e "s/^voxel_leaf_size_y:.*/voxel_leaf_size_y: ${voxel_leaf_size}/" \
                -e "s/^voxel_leaf_size_z:.*/voxel_leaf_size_z: ${voxel_leaf_size}/" \
                "${grid_map_cfg}"

              # 生成 safeland 配置（备降区参数 + 数据记录路径）
              cp "${ROOT_DIR}/src/costmap_ws/src/safeland/config/safeland.yaml" "${safeland_cfg}"
              sed -i \
                -e "s/^slope_threshold:.*/slope_threshold: ${SLOPE_THRESHOLD}/" \
                -e "s/^depression_score_threshold:.*/depression_score_threshold: ${DEPRESSION_THRESHOLD}/" \
                -e "s/^landing_valid_ratio_threshold:.*/landing_valid_ratio_threshold: ${LANDING_VALID_RATIO_THRESHOLD}/" \
                -e "s/^landing_height_range_threshold:.*/landing_height_range_threshold: ${LANDING_HEIGHT_RANGE_THRESHOLD}/" \
                -e "s/^step_threshold:.*/step_threshold: ${STEP_THRESHOLD}/" \
                -e "s/^landing_size:.*/landing_size: ${LANDING_SIZE}/" \
                -e "s/^landing_safety_margin:.*/landing_safety_margin: ${LANDING_SAFETY_MARGIN}/" \
                -e "s/^enable_alt_landing:.*/enable_alt_landing: ${enable_alt}/" \
                -e "s/^alt_landing_relax_factor:.*/alt_landing_relax_factor: ${alt_relax}/" \
                -e "s|^metrics_csv_path:.*|metrics_csv_path: \"${metrics_csv}\"|" \
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
                --alt-relax-factor "${alt_relax}" \
                --enable-alt-landing "${enable_alt}" \
                --polar-res "${polar_res}" \
                --sensing-horizon "${sensing_horizon}" \
                --metrics-csv "${metrics_csv}" || true

              kill "${launch_pid}" >/dev/null 2>&1 || true
              wait "${launch_pid}" >/dev/null 2>&1 || true
              sleep 1

            done
          done
        done
      done
    done
  done
done < "${MAP_LIST}"

echo "[round4] Done. Results: ${RESULT_CSV}"
