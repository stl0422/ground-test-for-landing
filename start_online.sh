#!/bin/bash
# =============================================================================
# 在线建图 + 安全降落区检测 一键启动脚本
# 模式：无人机边飞边建图，地图成熟后自动选择安全降落点执行降落
#
# 使用方法：
#   chmod +x start_online.sh
#   ./start_online.sh
#
# 依赖：
#   gnome-terminal / terminator / xterm 任意一个终端程序
#
# 工作空间路径（请按实际路径修改）：
#   PX4 仿真环境 : /home/fractal/PX4_Firmware
#   规划/控制 ws : /home/fractal/iros_challenge/devel/setup.bash
#   建图/降落 ws : /home/fractal/costmap_ws/devel/setup.bash
# =============================================================================

# ---- 路径变量 ----------------------------------------------------------------
IROS_WS=~/iros_challenge/devel/setup.bash
COSTMAP_WS=~/landing/devel/setup.bash
SUPER_WS=~/flm_ws/devel/setup.bash
PX4_DIR=~/PX4_Firmware
LOG_DIR=~/landing/start_online_logs

# ---- 等待时间（秒） ----------------------------------------------------------
# 各节点启动之间的等待，确保依赖节点先就绪
T_SIM=18       # Gazebo/MAVROS 启动较慢；等机体和仿真IMU稳定后再启动FAST-LIO
T_LIO=5        # FAST-LIO 需要 IMU/LiDAR 数据流稳定后才开始建图
T_GRID=3       # global_grid_map 依赖 /cloud_registered，等 FAST-LIO 就绪
T_SAFE=3       # safeland 依赖 /global_grid_map
T_MPC=3        # MPC 控制器
T_PLANNER=3    # super_planner（路径规划）
T_SM=2         # 状态机最后启动

mkdir -p "${LOG_DIR}"
rm -f "${LOG_DIR}"/*.log
rm -f "${LOG_DIR}"/*.pid

run_in_terminal() {
  local name="$1"
  local delay="$2"
  local cmd="$3"
  local log="${LOG_DIR}/${name}.log"
  local pid_file="${LOG_DIR}/${name}.pid"
  local wrapped

  wrapped="
set +e
: > '${log}'
echo \$\$ > '${pid_file}'
exec > >(stdbuf -oL tee -a '${log}') 2>&1
echo '[start_online] ${name}'
echo '[log] ${log}'
echo '[pid] ' \$\$
echo '[cmd] ${cmd}'
echo
trap 'status=\$?; echo; echo \"[start_online] ${name} shell exiting with code \${status}\"' EXIT
set -o pipefail
${cmd}
status=\$?
echo
echo '[start_online] ${name} command exited with code' \${status}
echo '[start_online] terminal kept open for debugging; press Ctrl-D or close the window when done.'
exec bash -i
"
  echo "[start_online] 打开终端: ${name}"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="${name}" -- bash -lc "${wrapped}"
  elif command -v terminator >/dev/null 2>&1; then
    terminator -T "${name}" -e "bash -lc \"${wrapped}\"" &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "${name}" -e "bash -lc \"${wrapped}\"" &
  else
    echo "[start_online] ERROR: no supported terminal found." >&2
    exit 1
  fi

  sleep "${delay}"
}

echo "[start_online] 使用真实终端前台启动各模块。"
echo "[start_online] 日志目录: ${LOG_DIR}"

if pgrep -f 'PX4_Firmware/build/px4_sitl_default/bin/px4|gzserver|roslaunch px4 livox_custom.launch' >/dev/null 2>&1; then
  echo "[start_online] 检测到已有在线仿真进程，先执行 ./stop_online.sh 清理。"
  "$(dirname "$0")/stop_online.sh"
  sleep 3
fi

run_in_terminal "01_px4_gazebo_mavros" "${T_SIM}" \
  "cd ${PX4_DIR} && source ${IROS_WS} && export ROS_PACKAGE_PATH=${PX4_DIR}:${PX4_DIR}/Tools/sitl_gazebo:\${ROS_PACKAGE_PATH} && roslaunch px4 livox_custom.launch interactive:=false"

run_in_terminal "02_fast_lio" "${T_LIO}" \
  "source ${IROS_WS} && { rosrun fast_lio fastlio_px4 & roslaunch fast_lio mapping_mid360.launch; }"

run_in_terminal "03_global_grid_map" "${T_GRID}" \
  "source ${COSTMAP_WS} && roslaunch fast_lio_global_grid_map global_grid_map.launch pcd_mode:=online"

run_in_terminal "04_safeland" "${T_SAFE}" \
  "source ${COSTMAP_WS} && roslaunch safeland safeland.launch open_rviz:=true"

run_in_terminal "05_mpc_control" "${T_MPC}" \
  "source ${IROS_WS} && roslaunch mpc_control control_sim_iros2025.launch"

run_in_terminal "06_super_planner" "${T_PLANNER}" \
  "source ${SUPER_WS} && roslaunch super_planner iros_real.launch"

run_in_terminal "07_state_machine" "${T_SM}" \
  "source ${IROS_WS} && export PYTHONPATH=/home/fractal/landing/devel/lib/python3/dist-packages:\${PYTHONPATH} && roslaunch state_machine iros_state_machine_simple_sim.launch"

echo ""
echo "============================================================"
echo "  [start_online] 启动命令已全部发出"
echo "  每个模块都在单独终端中前台运行。日志也同步写入:"
echo "    tail -f ${LOG_DIR}/01_px4_gazebo_mavros.log"
echo "    tail -f ${LOG_DIR}/02_fast_lio.log"
echo "    tail -f ${LOG_DIR}/03_global_grid_map.log"
echo "    tail -f ${LOG_DIR}/04_safeland.log"
echo "    tail -f ${LOG_DIR}/05_mpc_control.log"
echo "    tail -f ${LOG_DIR}/06_super_planner.log"
echo "    tail -f ${LOG_DIR}/07_state_machine.log"
echo "============================================================"
exit 0
