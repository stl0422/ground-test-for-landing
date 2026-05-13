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

# ---- 等待时间（秒） ----------------------------------------------------------
# 各节点启动之间的等待，确保依赖节点先就绪
T_SIM=8        # Gazebo/MAVROS 启动较慢，给充裕时间
T_LIO=5        # FAST-LIO 需要 IMU/LiDAR 数据流稳定后才开始建图
T_GRID=3       # global_grid_map 依赖 /cloud_registered，等 FAST-LIO 就绪
T_SAFE=3       # safeland 依赖 /global_grid_map
T_MPC=3        # MPC 控制器
T_PLANNER=3    # super_planner（路径规划）
T_SM=2         # 状态机最后启动

# =============================================================================
# 启动 Terminator 并依次创建 7 个分屏 pane
# 布局：垂直分 2 列，左列上下分 4 格，右列上下分 3 格
# =============================================================================
echo "[start_online] 启动 Terminator..."
terminator &
sleep 2

# 在当前 pane 基础上，水平切分（ctrl+shift+O=上下，ctrl+shift+E=左右）
# Pane 1 (当前) → Pane 2
xdotool key ctrl+shift+O
sleep 0.2
# Pane 1 → Pane 3
xdotool key ctrl+shift+O
sleep 0.2
# Pane 1 → Pane 4
xdotool key ctrl+shift+O
sleep 0.2
# Pane 1 → Pane 5
xdotool key ctrl+shift+O
sleep 0.2
# Pane 1 → Pane 6
xdotool key ctrl+shift+O
sleep 0.2
# Pane 1 → Pane 7
xdotool key ctrl+shift+O
sleep 0.4

# =============================================================================
# Pane 1：PX4 SITL + Gazebo + MAVROS + TF
#   - 启动仿真环境（nagetive_terrain.world）
#   - 发布 /livox/lidar、/livox/imu、/mavros/*
#   - 广播 body → base_link 静态 TF
# =============================================================================
echo "[start_online] [1/7] 启动 PX4 SITL + Gazebo..."
xdotool type "cd ${PX4_DIR} && source ${IROS_WS} && roslaunch px4 livox_custom.launch"
xdotool key Return
sleep ${T_SIM}

# =============================================================================
# Pane 2：FAST-LIO（在线建图）+ fastlio_px4（TF广播 & 视觉定位注入）
#
#   fast_lio mapping_mid360.launch：
#     - 订阅 /livox/lidar + /livox/imu
#     - 发布 /Odometry（camera_init系）、/cloud_registered（配准点云）
#     - 节点退出时自动保存 PCD 到 ~/iros_challenge/src/FAST_LIO/PCD/scans.pcd
#
#   fastlio_px4（rosrun fast_lio fastlio_px4）：
#     - 订阅 /Odometry，转发给 /mavros/vision_pose/pose（PX4 EKF2 视觉定位融合）
#     - 在主循环（30Hz）持续广播三个恒等 TF：
#         camera_init → world → odom → map
#       super_planner 的 rog_map 依赖 "world" 帧，缺少此 TF 会导致坐标变换失败
#
#   两者同 pane，fastlio_px4 用 & 后台运行，等 /Odometry 有数据后自动生效
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_online] [2/7] 启动 FAST-LIO 在线建图 + fastlio_px4 TF广播..."
xdotool type "source ${IROS_WS} && rosrun fast_lio fastlio_px4 & roslaunch fast_lio mapping_mid360.launch"
xdotool key Return
sleep ${T_LIO}

# =============================================================================
# Pane 3：global_grid_map（在线模式）
#   - 订阅 /cloud_registered，增量更新 2.5D 高程栅格地图
#   - 发布 /global_grid_map（grid_map_msgs，含 elevation/sum_z/count 图层）
#   - cell_count_threshold=0（全量累加取均值，适合坑洼检测场景，见 global_grid_map.yaml）
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_online] [3/7] 启动 global_grid_map（online 模式）..."
xdotool type "source ${COSTMAP_WS} && roslaunch fast_lio_global_grid_map global_grid_map.launch pcd_mode:=online"
xdotool key Return
sleep ${T_GRID}

# =============================================================================
# Pane 4：safeland（安全降落区检测）
#   - 订阅 /global_grid_map
#   - 计算坡度/粗糙度/阶梯/凹陷度，输出 /safeland/grid_map
#   - 含 landing_center（0/1二值）和 landing_score（综合安全得分）图层
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_online] [4/7] 启动 safeland 安全降落检测..."
xdotool type "source ${COSTMAP_WS} && roslaunch safeland safeland.launch open_rviz:=false"
xdotool key Return
sleep ${T_SAFE}

# =============================================================================
# Pane 5：MPC 轨迹跟踪控制器
#   - ~odom  ← /mavros/local_position/odom
#   - ~odom_lidar ← /Odometry（FAST-LIO，用于融合 z 轴高度）
#   - 发布 /drone_odometry（融合后里程计，供状态机使用）
#   - 发布 /mavros/setpoint_raw/attitude（姿态控制指令）
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_online] [5/7] 启动 MPC 控制器..."
xdotool type "source ${IROS_WS} && roslaunch mpc_control control_sim_iros2025.launch"
xdotool key Return
sleep ${T_MPC}

# =============================================================================
# Pane 6：super_planner（路径规划）
#   - 订阅 /goal（PoseStamped，camera_init 系），由状态机发布
#   - 订阅 /stop_super（Bool），状态机触发降落前停止规划
#   - 发布 /mpc_trajectory（ius_msgs/Trajectory），供 MPC 控制器跟踪
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_online] [6/7] 启动 super_planner 路径规划..."
xdotool type "source ${IROS_WS} && roslaunch super_planner iros_real.launch"
xdotool key Return
sleep ${T_PLANNER}

# =============================================================================
# Pane 7：状态机（核心任务调度）
#   - 订阅 /drone_odometry、/mavros/state、/safeland/grid_map
#   - 发布 /goal（航点指令）、/stop_super、/state_machine
#   - 流程：IDLE → CRUISE（巡航建图）→ WAIT_SAFELAND → FLY_TO_LAND → LAND
#   - 地图成熟度评估：landing_center 栅格数 ≥ 20 且 5s 内变化率 < 5% 时提前终止巡航
# =============================================================================
xdotool key ctrl+shift+P
sleep 0.2
echo "[start_online] [7/7] 启动状态机..."
xdotool type "source ${IROS_WS} && roslaunch state_machine iros_state_machine_simple_sim.launch"
xdotool key Return

echo ""
echo "============================================================"
echo "  [start_online] 所有节点已启动（在线建图模式）"
echo ""
echo "  流程说明："
echo "    1. 无人机按 WAYPOINTS 巡航，FAST-LIO 实时建图"
echo "    2. safeland 持续分析地图，输出安全降落区域"
echo "    3. 地图成熟（可降落面积足够 + 稳定）后自动进入降落流程"
echo "    4. 飞到最优降落点（距离+安全性加权）执行降落"
echo ""
echo "  PCD 保存路径（建图完成后）："
echo "    ~/iros_challenge/src/FAST_LIO/PCD/scans.pcd"
echo "    可供下次以离线模式（start_offline.sh）直接复用"
echo "============================================================"
