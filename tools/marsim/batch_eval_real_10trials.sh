#!/usr/bin/env bash
# =============================================================================
# batch_eval_real_10trials.sh
#
# 真实仿真测试脚本：10组参数组合 × 多张PCD地图
# 数据从真实 ROS 节点（safeland + marsim）采集
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_LIST="${ROOT_DIR}/tools/marsim/real_test_maps.txt"
RESULT_CSV="${ROOT_DIR}/tools/marsim_benchmark_results/real_10trials_results.csv"
TIMEOUT="25"
STABLE_SECS="3"
WARMUP_SECS="8"
OPEN_RVIZ="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --maps)         MAP_LIST="$2";   shift 2 ;;
    --csv)          RESULT_CSV="$2"; shift 2 ;;
    --timeout)      TIMEOUT="$2";    shift 2 ;;
    --stable-secs)  STABLE_SECS="$2";shift 2 ;;
    --warmup-secs)  WARMUP_SECS="$2";shift 2 ;;
    --open-rviz)    OPEN_RVIZ="$2";  shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 1 ;;
  esac
done

source /opt/ros/noetic/setup.bash
source /home/fractal/super_ws/devel/setup.bash
source /home/fractal/landing/devel/setup.bash --extend

mkdir -p "$(dirname "${RESULT_CSV}")"
TMP_DIR="$(mktemp -d)"
trap 'pkill -f "marsim_safeland_offline.launch" >/dev/null 2>&1 || true; pkill -f "perfect_drone" >/dev/null 2>&1 || true; rm -rf "${TMP_DIR}"' EXIT

# =============================================================================
# 10 组参数配置（grid_res, voxel, polar_res, sensing_horizon,
#                alt_relax, enable_alt, trial_label）
# =============================================================================
declare -a CONFIGS=(
  # trial 1: 基线 res=0.10  voxel=0.05  sparse=密集  备降开
  "0.10 0.05 0.2 30 1.5 true  baseline"
  # trial 2: 细分辨率 res=0.08  voxel=0.03
  "0.08 0.03 0.2 30 1.5 true  hires"
  # trial 3: 粗分辨率 res=0.12  voxel=0.08
  "0.12 0.08 0.2 30 1.5 true  lores"
  # trial 4: 稀疏传感器 polar=0.4 horizon=20
  "0.10 0.05 0.4 20 1.5 true  sparse_sensor"
  # trial 5: 备降关闭
  "0.10 0.05 0.2 30 1.5 false no_alt"
  # trial 6: 备降放宽x2
  "0.10 0.05 0.2 30 2.0 true  alt_relax2x"
  # trial 7: 细res + 稀疏传感器
  "0.08 0.03 0.4 20 1.5 true  hires_sparse"
  # trial 8: 粗res + 备降关闭
  "0.12 0.08 0.2 30 1.5 false lores_noalt"
  # trial 9: 推荐配置 res=0.09 voxel=0.04
  "0.09 0.04 0.2 30 1.8 true  recommended"
  # trial 10: 最激进 res=0.08 voxel=0.03 relax=2.0 视距近
  "0.08 0.03 0.4 20 2.0 true  aggressive"
)

TRIAL_ID=0
TOTAL_RUNS=0
FAILED_RUNS=0

while IFS= read -r map_path; do
  [[ -z "${map_path}" || "${map_path}" == \#* ]] && continue
  map_path="$(realpath "${map_path}")"
  if [[ ! -f "${map_path}" ]]; then
    echo "[WARN] PCD not found: ${map_path}, skip"
    continue
  fi
  map_name="$(basename "${map_path}" .pcd)"
  echo ""
  echo "================================================================"
  echo "[MAP] ${map_name}"
  echo "================================================================"

  for cfg_line in "${CONFIGS[@]}"; do
    read -r grid_res voxel polar_res sensing_horizon alt_relax enable_alt trial_label <<< "${cfg_line}"
    TRIAL_ID=$(( TRIAL_ID + 1 ))
    tag="${map_name}__t${TRIAL_ID}__${trial_label}__res${grid_res}__v${voxel}__polar${polar_res}__h${sensing_horizon}__relax${alt_relax}__alt${enable_alt}"

    echo ""
    echo "[T${TRIAL_ID}] ${trial_label} | res=${grid_res} voxel=${voxel} polar=${polar_res} horizon=${sensing_horizon} relax=${alt_relax} alt=${enable_alt}"
    echo "  map: ${map_name}"

    # ---- 生成 marsim 配置 ----
    marsim_cfg="${TMP_DIR}/${tag}.marsim.yaml"
    "${ROOT_DIR}/tools/marsim/make_marsim_offline_config.sh" \
      --pcd "${map_path}" \
      --out "${marsim_cfg}" \
      --polar-res "${polar_res}" \
      --sensing-horizon "${sensing_horizon}" \
      >/dev/null

    # ---- 生成 grid_map 配置 ----
    gmap_cfg="${TMP_DIR}/${tag}.gmap.yaml"
    cp /home/fractal/landing/src/costmap_ws/src/fast_lio_global_grid_map/config/global_grid_map.yaml "${gmap_cfg}"
    sed -i \
      -e "s/^resolution:.*/resolution: ${grid_res}/" \
      -e "s/^voxel_leaf_size_x:.*/voxel_leaf_size_x: ${voxel}/" \
      -e "s/^voxel_leaf_size_y:.*/voxel_leaf_size_y: ${voxel}/" \
      -e "s/^voxel_leaf_size_z:.*/voxel_leaf_size_z: ${voxel}/" \
      "${gmap_cfg}"

    # ---- 生成 safeland 配置 ----
    sl_cfg="${TMP_DIR}/${tag}.safeland.yaml"
    cp /home/fractal/landing/src/costmap_ws/src/safeland/config/safeland.yaml "${sl_cfg}"

    # 修复/添加备降区参数（原始 yaml 可能没有这些字段）
    if grep -q "^enable_alt_landing:" "${sl_cfg}"; then
      sed -i "s/^enable_alt_landing:.*/enable_alt_landing: ${enable_alt}/" "${sl_cfg}"
    else
      echo "enable_alt_landing: ${enable_alt}" >> "${sl_cfg}"
    fi
    if grep -q "^alt_landing_relax_factor:" "${sl_cfg}"; then
      sed -i "s/^alt_landing_relax_factor:.*/alt_landing_relax_factor: ${alt_relax}/" "${sl_cfg}"
    else
      echo "alt_landing_relax_factor: ${alt_relax}" >> "${sl_cfg}"
    fi

    # max_valid_z: kdxt 是真实室外扫描地图，设高一些以保留更多地面点
    # random_map 是障碍物地图，保持 1.0 过滤障碍物顶部
    if [[ "${map_name}" == *"kdxt"* ]]; then
      sed -i "s/^max_valid_z:.*/max_valid_z: 3.0/" "${sl_cfg}"
    else
      sed -i "s/^max_valid_z:.*/max_valid_z: 0.5/" "${sl_cfg}"
    fi

    # min_cell_points: 降低到3以提高稀疏地图的覆盖率
    sed -i "s/^min_cell_points:.*/min_cell_points: 3/" "${sl_cfg}"

    # ---- 启动 roslaunch ----
    roslaunch safeland marsim_safeland_offline.launch \
      marsim_config:="${marsim_cfg}" \
      grid_map_config:="${gmap_cfg}" \
      safeland_config:="${sl_cfg}" \
      open_rviz:="${OPEN_RVIZ}" \
      > "${TMP_DIR}/${tag}.launch.log" 2>&1 &
    launch_pid=$!

    sleep "${WARMUP_SECS}"

    # ---- 采集指标 ----
    set +e
    rosrun safeland collect_safeland_metrics.py \
      --csv "${RESULT_CSV}" \
      --timeout "${TIMEOUT}" \
      --stable-secs "${STABLE_SECS}" \
      --tag "${tag}" \
      --pcd "${map_path}" \
      --slope-threshold 0.15 \
      --depression-threshold 0.30 \
      --landing-valid-ratio-threshold 0.98 \
      --landing-height-range-threshold 0.05 \
      --step-threshold 0.10 \
      --landing-size 0.3 \
      --landing-safety-margin 0.0 \
      --grid-resolution "${grid_res}" \
      --voxel-leaf-size "${voxel}" \
      --alt-relax-factor "${alt_relax}" \
      --enable-alt-landing "${enable_alt}" \
      --polar-res "${polar_res}" \
      --sensing-horizon "${sensing_horizon}"
    collect_ret=$?
    set -e

    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    sleep 2

    TOTAL_RUNS=$(( TOTAL_RUNS + 1 ))
    if [[ "${collect_ret}" -ne 0 ]]; then
      FAILED_RUNS=$(( FAILED_RUNS + 1 ))
      echo "  [WARN] collect failed (ret=${collect_ret})"
    fi
  done
done < "${MAP_LIST}"

echo ""
echo "================================================================"
echo "[DONE] total=${TOTAL_RUNS}  failed=${FAILED_RUNS}"
echo "       results -> ${RESULT_CSV}"
echo "================================================================"
