#!/bin/bash
# =============================================================================
# 离线 PCD 模式 + 安全降落区检测 一键启动脚本
# 模式：直接加载已有 PCD 建立高程地图，无需 FAST-LIO 实时建图
#       适用于：已完成在线建图并保存了 scans.pcd 之后的任务复现/重复测试
#
# 使用方法：
#   chmod +x start_offline.sh
#   ./start_offline.sh [pcd文件路径]
#
#   示例（使用 FAST-LIO 上次建图保存的 PCD）：
#     ./start_offline.sh ~/iros_challenge/src/FAST_LIO/PCD/scans.pcd
#
#   不传参数时使用默认 PCD 路径（见下方 PCD_FILE 变量）。
#
# 依赖：
#   terminator   — 多终端窗口管理器
#   xdotool      — 终端模拟按键/输入
#
# 工作空间路径（请按实际路径修改）：
#   PX4 仿真环境 : /home/fractal/PX4_Firmware
#   规划/控制 ws : /home/fractal/iros_challenge/devel/setup.bash
#   建图/降落 ws : /home/fractal/costmap_ws/devel/setup.bash
# =============================================================================

# ---- 路径变量 ----------------------------------------------------------------
IROS_WS=~/iros_challenge/devel/setup.bash
COSTMAP_WS=~/costmap_ws/devel/setup.bash
PX4_DIR=~/PX4_Firmware

# ---- 离线 PCD 文件路径（优先使用命令行参数，否则用默认值）------------------
if [ -n "$1" ]; then
    PCD_FILE="$1"
else
    PCD_FILE=~/iros_challenge/src/FAST_LIO/PCD/scans.pcd
fi

# ---- 检查 PCD 文件是否存在 --------------------------------------------------
if [ ! -f "${PCD_FILE}" ]; then
    echo ""
    echo "  [错误] 找不到 PCD 文件：${PCD_FILE}"
    echo ""
    echo "  请先运行在线模式（./start_online.sh）完成一次建图，"
    echo "  FAST-LIO 退出时会自动保存 PCD 到："
    echo "    ~/iros_challenge/src/FAST_LIO/PCD/scans.pcd"
    echo ""
    echo "  或手动指定 PCD 路径："
    echo "    ./start_offline.sh /path/to/your_map.pcd"
    echo ""
    exit 1
fi

echo ""
echo "============================================================"
echo "  [start_offline] 离线模式"
echo "  PCD 文件：${PCD_FILE}"
echo "============================================================"
echo ""

# ---- 等待时间（秒） ----------------------------------------------------------
T_SIM=8        # Gazebo/MAVROS 启动较慢，给充裕时间
T_GRID=4       # offline 模式加载 PCD 灌图需要一点时间（取决于点云大小）
T_SAFE=3       # safeland 依赖 /global_grid_map
T_MPC=3        # MPC 控制器
T_PLANNER=3    # super_planner（路径规划）
T_SM=2         # 状态机最后启动

# =============================================================================
# 启动 Terminator 并依次创建 7 个分屏 pane
# 离线模式：不需要 FAST-LIO，但需要单独广播 camera_init→world TF，共 7 个 pane
# =============================================================================
echo "[start_offline] 启动 Terminator..."
terminator &
sleep 2

# 切分 6 次，得到 7 个 pane
xdotool key ctrl+shift+O
sleep 0.2
xdotool key ctrl+shift+O
sleep 0.2
xdotool key ctrl+shift+O
sleep 0.2
xdotool key ctrl+shift+O
sleep 0.2
xdotool key ctrl+shift+O
sleep 0.2
xdotool key ctrl+shift+O
sleep 0.4

# =============================================================================
# Pane 1：PX4 SITL + Gazebo + MAVROS + TF
#   - 启动仿真环境（nagetive_terrain.world）
#   - 离线模式下 FAST-LIO 不运行，仍需 MAVROS 提供飞控接口和定位
#   - Gazebo 通过 mavros 给 PX4 EKF2 提供 ground truth，local_position/odom 可正常使用
#   - 广播 body → base_link 静态 TF
# =============================================================================
echo "[start_offline] [1/7] 启动 PX4 SITL + Gazebo..."
xdotool type "cd ${PX4_DIR} && source ${IROS_WS} && roslaunch px4 livox_custom.launch"
xdotool key Return
sleep ${T_SIM}

# =============================================================================
# Pane 2：坐标系 TF 广播（离线模式关键步骤）
#
#   背景：在线模式下，fast_lio_px4 节点在主循环中动态广播如下三个恒等变换：
#           camera_init → world
#           world       → odom
#           world       → map
#   离线模式跳过了 FAST-LIO，这些 TF 无人发布。
#   super_planner、safeland、状态机等所有节点都依赖 "world" 帧，
#   缺少 TF 会导致 tf2 报错并无法进行坐标变换。
#
#   解决方案：用 static_transform_publisher 发布三个恒等（零偏移、零旋转）TF。
#   在仿真中，PX4 起飞点即为 camera_init 原点，三个帧完全对齐，恒等变换正确。
#
#   注意：Gazebo 内置的 EKF2 (local_position/odom) 以 local_origin 为参考，
#         fast_lio_px4 的 vision_pose 注入在离线时不需要（PX4 自己有 barometer+IMU）。
#         MPC 控制器的 z 轴融合（odom_lidar → /Odometry）在无 FAST-LIO 时自动跳过，
#         仅使用 mavros local_position 的高度，功能完整。
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_offline] [2/7] 广播坐标系 TF（camera_init→world→odom→map）..."
xdotool type "rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 camera_init world & rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world odom & rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 world map & wait"
xdotool key Return
sleep 2

# =============================================================================
# Pane 3：global_grid_map（离线模式）
#   - 直接加载 PCD 文件，一次性建立完整高程栅格地图
#   - 不订阅实时点云，地图建立后立即发布 /global_grid_map
#   - pcd_frame_id=camera_init 与 safeland/状态机坐标系统一
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_offline] [3/7] 加载离线 PCD，建立高程地图..."
xdotool type "source ${COSTMAP_WS} && roslaunch fast_lio_global_grid_map global_grid_map.launch pcd_mode:=offline pcd_file:=${PCD_FILE} pcd_frame_id:=camera_init"
xdotool key Return
sleep ${T_GRID}

# =============================================================================
# Pane 4：safeland（安全降落区检测）
#   - 离线模式下 PCD 地图已完整，safeland 一启动就能输出完整结果
#   - 发布 /safeland/grid_map（含 landing_center + landing_score）
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_offline] [4/7] 启动 safeland 安全降落检测..."
xdotool type "source ${COSTMAP_WS} && roslaunch safeland safeland.launch open_rviz:=false"
xdotool key Return
sleep ${T_SAFE}

# =============================================================================
# Pane 5：MPC 轨迹跟踪控制器
#   - 主定位来源：/mavros/local_position/odom（Gazebo→PX4 EKF2，始终可用）
#   - odom_lidar（/Odometry）：FAST-LIO 的 z 轴辅助融合，离线时无信号自动跳过
#     → MPC 回退到 mavros 高度，不影响飞行安全
#   - 发布 /drone_odometry、/mavros/setpoint_raw/attitude
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_offline] [5/7] 启动 MPC 控制器..."
xdotool type "source ${IROS_WS} && roslaunch mpc_control control_sim_iros2025.launch"
xdotool key Return
sleep ${T_MPC}

# =============================================================================
# Pane 6：super_planner（路径规划）—— 离线专用配置
#
#   离线模式与在线模式的区别（两份配置文件，互不干扰）：
#     在线：iros_real.yaml    → load_pcd_en: false，rog_map 订阅 /cloud_registered
#     离线：iros_offline.yaml → load_pcd_en: true，启动时从 PCD 文件一次性加载障碍物地图
#
#   在正式启动前，用 sed 将 iros_offline.yaml 中的 pcd_name 替换为当前 PCD_FILE，
#   保证路径与命令行传入的参数一致。
# =============================================================================
OFFLINE_CFG=~/iros_challenge/src/super_planner/config/iros_offline.yaml
# 将 iros_offline.yaml 中的 pcd_name 替换为当前实际 PCD 路径（原地修改）
# 兼容 macOS(BSD sed) 和 Linux(GNU sed)：macOS 的 -i 需要紧跟空字符串参数
if [[ "$(uname)" == "Darwin" ]]; then
    sed -i "" "s|pcd_name:.*|pcd_name: \"${PCD_FILE}\"|" ${OFFLINE_CFG}
else
    sed -i "s|pcd_name:.*|pcd_name: \"${PCD_FILE}\"|" ${OFFLINE_CFG}
fi
echo "[start_offline] rog_map pcd_name → ${PCD_FILE}"

xdotool key ctrl+shift+P
sleep 0.2
echo "[start_offline] [6/7] 启动 super_planner 路径规划（离线配置）..."
xdotool type "source ${IROS_WS} && roslaunch super_planner iros_real.launch config_name:=iros_offline.yaml"
xdotool key Return
sleep ${T_PLANNER}

# =============================================================================
# Pane 7：状态机（核心任务调度）
#   - 离线模式下 safeland 启动即有结果，状态机无需等待地图成熟
#   - 巡航完成（或提前触发地图成熟）后立即飞往最优降落点
#   - 提示：离线模式可将 map_mature_min_cells 设小（如 5），
#           让状态机在巡航初期就能快速拿到 safeland 结果跳出巡航
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_offline] [7/7] 启动状态机..."
xdotool type "source ${IROS_WS} && roslaunch state_machine iros_state_machine_simple_sim.launch"
xdotool key Return

echo ""
echo "============================================================"
echo "  [start_offline] 所有节点已启动（离线 PCD 模式）"
echo ""
echo "  定位与地图链路说明（离线模式）："
echo "    • PX4 飞控定位：Gazebo ground truth → mavros → PX4 EKF2"
echo "                    → /mavros/local_position/odom（始终可用）"
echo "    • FAST-LIO：不启动（离线模式无实时点云）"
echo "    • MPC z 轴融合：无 /Odometry，自动回退到 mavros 高度"
echo "    • 坐标系 TF：由 static_transform_publisher 广播恒等变换"
echo "                 camera_init = world = odom = map（仿真起飞点）"
echo "    • rog_map 障碍物地图：load_pcd_en=true，启动时从 PCD 文件"
echo "                         一次性加载，A* 和走廊生成器正常感知障碍物"
echo "      （在线模式 iros_real.yaml 的 load_pcd_en=false，两路互不干扰）"
echo ""
echo "  流程说明："
echo "    1. global_grid_map 加载 PCD 后立即发布完整高程地图"
echo "    2. safeland 一次性计算全图安全降落区域（无需等待建图）"
echo "    3. 状态机巡航过程中实时接收 safeland 结果"
echo "    4. 地图已成熟（PCD 加载完成）→ 快速进入降落流程"
echo "    5. 飞到最优降落点（距离+安全性加权）执行降落"
echo ""
echo "  当前 PCD 文件：${PCD_FILE}"
echo "============================================================"
