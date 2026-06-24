#!/usr/bin/env bash
# =============================================================================
# batch_eval_online_10trials.sh
#
# 利用 start_online.sh 已跑起来的完整在线环境（Gazebo nagetive_terrain +
# PX4 + FAST-LIO + global_grid_map）做 10 组 safeland 参数批量测试。
#
# 使用方式（两步）：
#
#   步骤1：先在一个终端启动完整在线环境并等无人机飞起来建好地图（约2分钟）：
#     cd /home/fractal/ground-test-for-landing && ./start_online.sh
#
#   步骤2：在另一个终端运行本脚本采集数据：
#     bash tools/marsim/batch_eval_online_10trials.sh
#
# 本脚本 **不启动** Gazebo/PX4/FAST-LIO，只循环重启 safeland 节点采集指标。
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RESULT_CSV="${ROOT_DIR}/tools/marsim_benchmark_results/online_10trials_results.csv"
LOG_DIR="${ROOT_DIR}/tools/marsim_benchmark_results/online_logs"
OPEN_RVIZ="false"
TRIAL_TIMEOUT=35   # 每组采集超时
STABLE_SECS=3      # 采集稳定等待
WAIT_MAP=60        # 等地图就绪超时（秒）

while [[ $# -gt 0 ]]; do
  case "$1" in
    --csv)          RESULT_CSV="$2";    shift 2 ;;
    --open-rviz)    OPEN_RVIZ="$2";     shift 2 ;;
    --timeout)      TRIAL_TIMEOUT="$2"; shift 2 ;;
    --wait-map)     WAIT_MAP="$2";      shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 1 ;;
  esac
done

# ---- source 工作空间（仅用于 safeland + collect 脚本）----
source /opt/ros/noetic/setup.bash
source /home/fractal/landing/devel/setup.bash --extend

mkdir -p "$(dirname "${RESULT_CSV}")"
mkdir -p "${LOG_DIR}"
TMP_DIR="$(mktemp -d)"

cleanup() {
  echo ""
  echo "[batch_online] 停止 safeland 节点..."
  pkill -f "safeland_node"         2>/dev/null || true
  pkill -f "safeland.launch"       2>/dev/null || true
  rm -rf "${TMP_DIR}"
  echo "[batch_online] 清理完成（Gazebo/PX4/FAST-LIO 保持运行）"
}
trap cleanup EXIT

# =============================================================================
# 10 组参数配置
# 格式：grid_res  voxel  slope_t  step_t  dep_t  landing_sz  enable_alt  relax  label
# =============================================================================
declare -a CONFIGS=(
  "0.10 0.05 0.15 0.05 0.30 0.5 true  1.5 baseline"
  "0.08 0.03 0.15 0.05 0.30 0.5 true  1.5 hires"
  "0.12 0.08 0.15 0.05 0.30 0.5 true  1.5 lores"
  "0.10 0.05 0.20 0.08 0.40 0.5 true  1.5 relaxed_thresholds"
  "0.10 0.05 0.12 0.04 0.25 0.5 true  1.5 strict_thresholds"
  "0.10 0.05 0.15 0.05 0.30 0.5 false 1.5 no_alt"
  "0.10 0.05 0.15 0.05 0.30 0.5 true  2.0 alt_relax2x"
  "0.08 0.03 0.20 0.08 0.40 0.4 true  2.0 hires_relaxed"
  "0.09 0.04 0.15 0.05 0.30 0.5 true  1.8 recommended"
  "0.10 0.05 0.25 0.10 0.50 0.6 true  2.5 very_relaxed"
)

# =============================================================================
# Step 1: 等待 start_online.sh 环境就绪（/global_grid_map 有数据）
# =============================================================================
echo ""
echo "================================================================"
echo "[Step 1] 等待在线环境就绪 (/global_grid_map 持续发布)"
echo "  ← 请先在另一个终端运行: ./start_online.sh"
echo "  ← 等无人机飞起来并建好地图后本脚本自动继续"
echo "================================================================"

WAITED=0
MAP_READY=false
while [[ ${WAITED} -lt ${WAIT_MAP} ]]; do
  # 检查 /global_grid_map 是否在发布（rostopic hz 检测有无消息）
  if rostopic info /global_grid_map 2>/dev/null | grep -q "Publishers:"; then
    # 话题存在，再确认有实际数据（检查 /cloud_registered 有无消息流）
    if timeout 4 rostopic hz /cloud_registered 2>/dev/null | grep -q "average rate:"; then
      echo "[batch_online] /global_grid_map 就绪，点云正在流入 ✓"
      MAP_READY=true
      break
    fi
  fi
  echo "[batch_online] 等待在线环境... (${WAITED}/${WAIT_MAP}s)"
  sleep 5
  WAITED=$(( WAITED + 5 ))
done

if [[ "${MAP_READY}" == "false" ]]; then
  # 即使超时也继续，safeland 会自己等
  echo "[batch_online] WARNING: 超时等待，继续尝试（safeland 会自己订阅并等）"
fi

# =============================================================================
# Step 2: 10 组 safeland 参数循环
# =============================================================================
echo ""
echo "================================================================"
echo "[Step 2] 开始 10 组 safeland 参数测试"
echo "================================================================"

TRIAL_ID=0
SUCCESS=0
FAIL=0

for cfg_line in "${CONFIGS[@]}"; do
  read -r grid_res voxel slope_t step_t dep_t land_sz enable_alt relax label <<< "${cfg_line}"
  TRIAL_ID=$(( TRIAL_ID + 1 ))

  echo ""
  echo "[T${TRIAL_ID}/10] ${label}"
  echo "  res=${grid_res} voxel=${voxel} slope=${slope_t} step=${step_t} dep=${dep_t}"
  echo "  land_sz=${land_sz} alt=${enable_alt} relax=${relax}"

  # 生成 safeland 配置
  sl_cfg="${TMP_DIR}/online_t${TRIAL_ID}_${label}.safeland.yaml"
  cp /home/fractal/landing/src/costmap_ws/src/safeland/config/safeland.yaml "${sl_cfg}"

  # 修改关键参数
  sed -i "s/^slope_threshold:.*/slope_threshold: ${slope_t}/"                       "${sl_cfg}"
  sed -i "s/^step_threshold:.*/step_threshold: ${step_t}/"                          "${sl_cfg}"
  sed -i "s/^depression_score_threshold:.*/depression_score_threshold: ${dep_t}/"   "${sl_cfg}"
  sed -i "s/^landing_size:.*/landing_size: ${land_sz}/"                             "${sl_cfg}"
  sed -i "s/^max_valid_z:.*/max_valid_z: 3.0/"                                     "${sl_cfg}"
  sed -i "s/^min_cell_points:.*/min_cell_points: 3/"                               "${sl_cfg}"

  # 备降区参数（可能不在原始yaml里）
  if grep -q "^enable_alt_landing:" "${sl_cfg}"; then
    sed -i "s/^enable_alt_landing:.*/enable_alt_landing: ${enable_alt}/" "${sl_cfg}"
  else
    echo "enable_alt_landing: ${enable_alt}" >> "${sl_cfg}"
  fi
  if grep -q "^alt_landing_relax_factor:" "${sl_cfg}"; then
    sed -i "s/^alt_landing_relax_factor:.*/alt_landing_relax_factor: ${relax}/" "${sl_cfg}"
  else
    echo "alt_landing_relax_factor: ${relax}" >> "${sl_cfg}"
  fi

  # 启动 safeland 节点
  roslaunch safeland safeland.launch \
    config_file:="${sl_cfg}" \
    open_rviz:="${OPEN_RVIZ}" \
    input_grid_map_topic:=/global_grid_map \
    output_grid_map_topic:=/safeland/grid_map \
    > "${LOG_DIR}/safeland_t${TRIAL_ID}.log" 2>&1 &
  SL_PID=$!
  sleep 5  # 等 safeland 节点就绪

  # 采集指标
  set +e
  rosrun safeland collect_safeland_metrics.py \
    --csv "${RESULT_CSV}" \
    --timeout "${TRIAL_TIMEOUT}" \
    --stable-secs "${STABLE_SECS}" \
    --tag "online__t${TRIAL_ID}__${label}__res${grid_res}__v${voxel}__slope${slope_t}__dep${dep_t}__alt${enable_alt}__relax${relax}" \
    --pcd "nagetive_terrain.world" \
    --slope-threshold "${slope_t}" \
    --depression-threshold "${dep_t}" \
    --landing-valid-ratio-threshold 0.98 \
    --landing-height-range-threshold 0.05 \
    --step-threshold "${step_t}" \
    --landing-size "${land_sz}" \
    --landing-safety-margin 0.0 \
    --grid-resolution "${grid_res}" \
    --voxel-leaf-size "${voxel}" \
    --alt-relax-factor "${relax}" \
    --enable-alt-landing "${enable_alt}" \
    --polar-res 0.0 \
    --sensing-horizon 0.0
  collect_ret=$?
  set -e

  # 停止 safeland 节点（下一组重新用新配置启动）
  kill "${SL_PID}" 2>/dev/null || true
  wait "${SL_PID}" 2>/dev/null || true
  sleep 2

  if [[ "${collect_ret}" -eq 0 ]]; then
    SUCCESS=$(( SUCCESS + 1 ))
  else
    FAIL=$(( FAIL + 1 ))
    echo "  [WARN] 采集失败 (ret=${collect_ret})"
  fi
done

echo ""
echo "================================================================"
echo "[DONE] online 10 组测试完成"
echo "  成功: ${SUCCESS}  失败: ${FAIL}"
echo "  结果: ${RESULT_CSV}"
echo "================================================================"
