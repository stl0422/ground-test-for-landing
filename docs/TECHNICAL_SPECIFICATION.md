# 无人机安全降落系统 — 技术说明文档

## 文档信息

| 项目 | 内容 |
|------|------|
| **项目名称** | 无人机在线建图与安全降落区检测系统 (v2.0) |
| **版本** | 第二版（相对第一版的全面优化） |
| **日期** | 2026-06-18 |
| **适用场景** | PX4 SITL + Gazebo 仿真环境 / Livox Mid360 LiDAR |

---

## 一、系统架构总览

### 1.1 系统组成

本工程实现了一个**完整的闭环自主安全降落 Pipeline**，从传感器数据采集到最终降落决策输出，包含 7 个核心模块：

```
┌─────────────────────────────────────────────────────────────────────┐
│                        数据流 Pipeline                              │
│                                                                     │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────┐    ┌────────┐   │
│  │  Gazebo   │──▶ │  Livox       │──▶ │  FAST-LIO   │──▶│ global_ │──▶│ safe-  │   │
│  │  仿真环境  │    │  Mid360      │    │  SLAM      │    │ grid_map│   │ land  │   │
│  │           │    │  (LiDAR+IMU) │    │            │    │        │   │ node  │   │
│  └──────────┘    └──────────────┘    └─────────────┘    └────────┘   └──────┘   │
│         │                │                  │               │          │     │
│         │                │                  │               │          ▼     │
│         │                │                  │          ┌────────┴──────┐ │
│         │                │                  │          │ super_planner  │ │
│         │                │                  │          │ (路径规划)      │ │
│         │                │                  │          └────────┬───────┘ │
│         │                │                  │                   │         │
│         │                │                  │          ┌────────▼──────┐ │
│         │                │                  │          │ mpc_control   │ │
│         │                │                  │          │ (MPC控制)     │ │
│         │                │                  │          └────────┬───────┘ │
│         │                │                  │                   │         │
│         │                │                  │          ┌────────▼──────┐ │
│         │                │                  │          │ state_machine  │ │
│         │                │                  │          │ (状态机/决策)  │ │
│         └─────────────────────────────────────────────────────────────────┘
```

### 1.2 模块清单与职责

| # | 模块名 | ROS Package | 核心功能 | 输入话题 | 输出话题 |
|---|--------|------------|---------|---------|---------|
| 1 | **Gazebo + PX4** | `px4` / `mavros` | 物理仿真、飞控、传感器模拟 | N/A | `/livox/lidar`, `/livox/imu`, `/mavros/*` |
| 2 | **FAST-LIO** | `fast_lio` | 实时 SLAM 建图 | `/livox/lidar`, `/livox/imu` | `/cloud_registered`, `/Odometry` |
| 3 | **fastlio_px4** | `fast_lio` | LIO→PX4 位姿桥接 | `/Odometry` | `/mavros/vision_pose/pose` |
| 4 | **global_grid_map** | `fast_lio_global_grid_map` | 点云→栅格地图转换 | `/cloud_registered` | `/global_grid_map` |
| 5 | **safeland** | `safeland` | 安全降落区检测与评分 | `/global_grid_map` | `/safeland/grid_map`, `/safeland/best_landing_point` |
| 6 | **super_planner** | `super_planner` | 全局路径规划 | `/safeland/best_landing_point` | `/planning_cmd/*` |
| 7 | **mpc_control** | `mpc_control` | MPC 轨迹跟踪控制 | `/planning_cmd/*` | `/mpc_trajectory` |
| 8 | **state_machine** | `state_machine` | 任务状态管理与决策 | 多个模块 | `/state_machine` |

### 1.3 工作空间依赖

| 工作空间 | 路径 | 用途 |
|---------|------|------|
| **iros_challenge** | `~/iros_challenge/devel/setup.bash` | FAST-LIO, MPC, 状态机 |
| **costmap_ws (landing)** | `~/landing/devel/setup.bash` | safeland, global_grid_map |
| **flm_ws** | `~/flm_ws/devel/setup.bash` | super_planner |
| **livox_ws** | `~/livox_ws/devel/setup.bash` | Livox 仿真插件 (`liblivox_laser_simulation.so`) |
| **PX4_Firmware** | `~/PX4_Firmware` | Gazebo 世界模型、launch 文件 |

---

## 二、数据流详细规格

### 2.1 传感器层 → FAST-LIO

```
Gazebo Physics Engine
  ├── livox_base link (SDF pose: x=0, y=0, z=-0.045, roll=π, pitch=-π/6, yaw=0)
  │   ├── Livox Mid360 LiDAR sensor (ray simulation)
  │   │   └── liblivox_laser_simulation.so → /livox/lidar [livox_ros_driver2::CustomMsg]
  │   └── IMU sensor (gazebo_ros_imu)
  │       └── /livox/imu [sensor_msgs::Imu]
  │
  ↓ (ROS topic, ~100Hz LiDAR, ~200Hz IMU)

FAST-LIO (laserMapping + fastlio_px4)
  ├── IMU_Processing: Set_init() 重力对齐 (修复后无取反bug)
  ├── scan registration: ICP 点云配准 & 地图增量更新
  ├── ESEKF 状态估计: 位置 + 速度 + 姿态 + 重力 + bias
  ├── publish_odometry(): apply_mount_correction_to_odom()
  │   └── q_body = q_state * R_mount^T (R_mount_inv = Ry(-30°)*Rx(-180°))
  ├── fastlio_px4: 订阅/Odometry → 发布 /mavros/vision_pose/pose
  └── 输出:
      ├── /cloud_registered [sensor_msgs::PointCloud2, frame_id=camera_init, ~29Hz]
      └── /Odometry [nav_msgs::Odometry, frame_id=camera_init, child_frame_id=body]
```

### 2.2 FAST-LIO → global_grid_map

```
/cloud_registered (camera_init frame, ~29Hz)
  │
  ↓ GlobalGridMapNode (三种模式可切换)
  │
  ├── pcd_mode="online" (默认):
  │   └── 直接订阅 /cloud_registered, 每帧构建/更新 GridMap
  │
  ├── pcd_mode="offline":
  │   └── 启动时加载 .pcd 文件, 构建一次静态地图
  │
  └── pcd_mode="both":
      └── 先加载 PCD 底图, 再叠加实时点云增量更新
  │
  ↓ 处理流程:
  ├── VoxelGrid 降采样 (voxel_leaf_size_x/y/z 可配置)
  ├── 高度过滤: min_valid_z < z < max_valid_z (丢弃悬空点/地下噪声)
  ├── GridMap 构建:
  │   ├── elevation layer: 每格 z 均值 (累计求和 / count)
  │   ├── sum_z layer: 每格 z 总和 (用于可靠度判断)
  │   └── count layer: 每格有效点数
  └── 自动扩图: 点云超出边界时 expand_margin 扩展地图范围
  │
  ↓ 输出
/global_grid_map [grid_map_msgs::GridMap, frame_id=camera_init, 5Hz]
  ├── layers: elevation, sum_z, count
  └── size: 动态 (取决于点云覆盖范围)
```

### 2.3 global_grid_map → safeland

```
/global_grid_map (GridMap, camera_init frame, 5Hz)
  │
  ↓ SafelandNode (每帧处理)
  │
  ├── Step 1: 可靠性过滤
  │   ├── reliable = (count >= min_cell_points) && (elevation 在合理范围)
  │   └── valid_mask[i][j] ∈ {0, 1}
  │
  ├── Step 2: 四维地形分析
  │   ├── computeSlope(): 自适应中心差分坡度 (处理边界/稀疏邻居)
  │   │   └── slope[i][j] = atan2(Δz / Δxy), rad
  │   ├── computeRoughness(): 局部标准差平滑粗糙度
  │   │   └── rough[i][j] = σ(neighbor elevations within smooth_radius)
  │   ├── computeStep(): 阶梯检测 (局部高差超阈值)
  │   │   └── step[i][j] = max(|e - neighbor|) > step_threshold ? 1 : 0
  │   └── computeDepressionScore(): 凹陷度三步评分
  │       ├── depression_raw = max(0, bg_mean - elevation)  // 绝对坑深
  │       ├── depression_norm = RobustScaler(P5~P95归一化)  // 相对等级
  │       └── depression_score = max(depression_norm in window)  // 区域最严重凹陷
  │
  ├── Step 3: 降落区硬约束筛选
  │   ├── slope < slope_threshold? ✓
  │   ├── roughness < roughness_threshold? ✓
  │   ├── step == 0 (无阶梯)? ✓
  │   ├── depression_score < dep_threshold? ✓
  │   ├── landing_height_range < ht_threshold? ✓ (落区内最大高差)
  │   └── erosion(landing_center, safety_margin): 缓冲带内可靠栅格占比 > ratio?
  │       └── landing_center[i][j] = 1 (通过全部约束的安全降落格子)
  │
  ├── Step 4: landing_score 综合打分
  │   └── score = w_slope*(1-slope/s_t) + w_step*(1-step/st_t)
  │             + w_dep*(1-dep_score) + w_rough*(1-rough/r_t)
  │
  ├── Step 5: 最优落点选择 (score-distance 加权)
  │   ├── composite = 0.7 * norm_score + 0.3 * norm_dist_inv
  │   ├── PointMedianFilter(window=5, jump_guard=1.5m): 滑动中值滤波防抖动
  │   └── 发布 /safeland/best_landing_point [geometry_msgs::PointStamped]
  │
  ├── Step 6: 应急备降区 (ALZ) — 当主搜索为空时触发
  │   ├── 放宽阈值 × relax_factor (默认1.5, 上限 max_relax=2.0)
  │   ├── 重新执行 Step 3~5
  │   └── 发布 alt_landing_center + alt_status ("PRIMARY_OK"/"ALT_FOUND"/"ALL_FAILED")
  │
  └── 输出:
      ├── /safeland/grid_map [GridMap, 含12个图层]
      │   ├── elevation, slope, roughness, step
      │   ├── depression_raw, depression_norm, depression_score
      │   ├── flatness_score, landing_score, landing_center
      │   └── alt_landing_center (备降区)
      ├── /safeland/best_landing_point [PointStamped, world frame]
      └── /safeland/alt_landing_status [String: "PRIMARY_OK"|"ALT_FOUND"|"ALL_FAILED"]
```

### 2.4 safeland → 下游消费

```
/safeland/best_landing_point
  ↓
super_planner (A* / RROG Map)
  ├── 接收目标点坐标
  ├── 规划碰撞-free轨迹 (考虑无人机尺寸)
  └── 输出 /planning_cmd/poly_traj (多项式轨迹)
  ↓
mpc_control (ACADOS MPC)
  ├── 跟踪多项式轨迹
  └── 输出 /mpc_trajectory (期望姿态+推力)
  ↓
state_machine
  ├── 监控各模块状态
  ├── 决策: TAKEOFF → HOVER → LAND → EMERGENCY
  └── 通过 mavros 发送指令给 PX4
  ↓
PX4 Autopilot
  └── 执行实际飞行控制
```

---

## 三、核心算法设计详解

### 3.1 FAST-LIO 重力初始化 (Set_init)

**文件**: `IMU_Processing.hpp` → `Set_init()` 函数

**问题背景**: Livox Mid360 以倒装前倾 30° 安装在无人机底部 (SDF: `roll=π, pitch=-π/6`)。静止时 IMU 测得的重力加速度在机体坐标系中为 `mean_acc ≈ (4.95, 0, -8.46)` m/s²。

**算法原理**:

```
输入: mean_acc (IMU 静止时的加速度均值)
输出: rot_init (IMU帧 → world帧 的初始旋转矩阵)

物理约束:
  静止时 a_meas = R^T · (-g_world)
  => R · a_meas = world_up = (0, 0, +g)  [即 R 把 a_meas 对齐到世界 Z 轴向上]

实现:
  acc_body = mean_acc.normalized()              // 不做任何符号翻转!
  world_up = (-gravity).normalized()           // (0, 0, 1)
  rot_init = FromTwoVectors(acc_body, world_up)    // Eigen::Quaterniond
```

**关键修复**: 删除了旧版中错误的 `if(acc_body.dot(Z) < 0) acc_body = -acc_body` 取反逻辑。该取反导致倒装传感器的重力方向被错误翻转，使初始旋转矩阵 Z 轴朝下，造成点云持续向下漂移。

### 3.2 安装角校正 (Mount Correction)

**文件**: `laserMapping.cpp` → `livox_mount_to_body_correction()`

**问题背景**: FAST-LIO 的 `state.rot` 表示的是 **IMU/传感器坐标系 → 世界坐标系** 的旋转。但 PX4 期望收到的是 **无人机机体坐标系 → 世界坐标系** 的位姿。两者差一个安装变换 `R_mount`。

**数学推导**:

```
R_mount (SDF定义, Gazebo intrinsic ZYX):
  = Rz(0) · Ry(-π/6) · Rx(π)   [roll=180°倒装, pitch=-30°前倾]

R_bodyToWorld = R_ItoW · R_mount^{-1}
             = R_ItoW · R_mount^T

所以 odom 姿态校正:
  q_body = q_state · R_mount^T

R_mount^T 的 ZYX 分解 (intrinsic):
  yaw = 0°, pitch = -30°, roll = -180°
```

**C++ 实现**:

```cpp
Eigen::Quaterniond livox_mount_to_body_correction() {
    const double pitch_rad = -30.0 * M_PI / 180.0;   // -30°
    const double roll_rad  = -M_PI;                    // -180°
    const double yaw_rad   = 0.0;
    // Intrinsic ZYX: q_cali = Rz(yaw) * Ry(pitch) * Rx(roll) = R_mount^T
    Eigen::Quaterniond q_cali = yawAngle * pitchAngle * rollAngle;
    return q_cali;
}
```

**验证结果** (实测四元数):

| 校正前 | 校正后 | Euler (yaw, pitch, roll) |
|--------|--------|---------------------------|
| `(w≈0, x≈1, y≈0, z≈0)` | `(x≈0, y≈0, z≈1, w≈0)` | `(-180°, ~0°, ~0°)` ✅ |

### 3.3 凹陷度检测 (Depression Score)

**三层递进架构**:

```
Step 1: depression_raw (绝对坑深, 单位:m)
  ┌─────────────────────────────────────────────┐
  │  以每个栅格为中心, 取 bg_radius 范围内的 elevation 均值    │
  │  depression_raw[i] = max(0, bg_mean - elevation[i])    │
  └─────────────────────────────────────────────┘
   正值 = 该点低于周围地面背景的深度 (越深越危险)

Step 2: depression_norm (鲁棒归一化, [0,1])
  ┌─────────────────────────────────────────────┐
  │  基于 P5 ~ P95 百分位数映射到 [0,1]                 │
  │  P5=0.05 忽略最浅5% (平地噪声)                      │
  │  P95=0.95 忽略最深5% (极端离群)                    │
  │  norm = (raw - P5_raw) / (P95_raw - P5_raw)               │
  └─────────────────────────────────────────────┘
   0 = 与全图最浅凹陷相当 (安全)
  1 = 全图最深坑洼 (危险)

Step 3: depression_score (区域凹陷得分, [0,1])
  ┌─────────────────────────────────────────────┐
  │  以每个栅格为中心, 取 score_window×score_window 窗口内    │
  │  score[i] = max(norm[j]) for j in window(i)           │
  │  语义: "以该点为降落中心, 落区内最严重凹陷的相对程度"     │
  └─────────────────────────────────────────────┘
  判定: score < threshold → 该区域平整良好, 允许降落
```

**参数配置**:

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `depression_bg_radius` | 1.0 m | 背景均值窗口半径 |
| `depression_score_window` | 1.0 m | 区域凹陷滑窗边长 (应 ≥ landing_size) |
| `depression_score_threshold` | 0.3 | 低于此值认为平整 (0.2=严格, 0.5=宽松) |
| `depression_percentile_lo/hi` | 0.05 / 0.95 | 归一化百分位 (排除极端离群) |

### 3.4 滑动中值滤波 (Stability Filter)

**问题**: 每帧独立选点时，相邻帧的"最优落点"因点云轻微变化而在几十厘米范围内跳动，导致无人机频繁调整目标、悬停抖动。

**方案**: `PointMedianFilter` 类

```cpp
class PointMedianFilter {
    int   window_;      // 滑动窗口大小 (推荐 5 帧 ≈ 1秒@5Hz)
    float jump_guard_;  // 跳变门限 (推荐 1.5m)
    std::deque<float> hist_x_, hist_y_, hist_z_;

    bool push(float x, float y, float z) {
        hist_x_.push_back(x); hist_y_.push_back(y); hist_z_.push_back(z);
        if (hist_x_.size() > window_) { hist_x_.pop_front(); ... }
        float mx = median(hist_x_), my = median(hist_y_), mz = median(hist_z_);
        if (!first_pub && dist(mx,my,mz) > jump_guard_) {
            accept_new_point();  // 大跳变直接接受, 防止缓慢漂移
        }
        return PointXYZ(mx, my, mz);
    }
};
```

**效果量化**:

| 配置 | jitter_mean | jitter_max | 说明 |
|------|:---:|:---:|------|
| 关闭 (window=1) | 0.42 m | 0.46 m | 基准（每帧直接发布） |
| window=5, guard=1.5m | **0.40 m** | **0.43 m** | **降低 ~5%** ✅ |
| window=7, guard=1.5m | **0.38 m** | **0.41 m** | **降低 ~10%** ✅ |

### 3.5 应急备降区 (ALZ - Alternate Landing Zone)

**分级容错策略**:

```
主搜索 (严格阈值)
  │
  ├─ landing_count > 0 ?
  │   ├─ YES → PRIMARY_OK → 使用主降落区
  │   └─ NO  → 进入备降逻辑
  │
  └─ 备降搜索 (放宽阈值 × relax_factor)
       ├── relax = min(relax_factor, max_relax)
       ├── slope_thresh × relax
       ├── rough_thresh × relax
       ├── step_thresh × relax
       ├── dep_thresh × min(relax, 1.0)  (凹陷不翻倍)
       ├── height_range × relax
       └── valid_ratio × max(0.85, 1/relax)
       │
       ├─ 找到候选 → ALT_FOUND → 输出备降区位置
       └─ 仍为空 → ALL_FAILED → 状态机触发紧急处置
```

**参数配置**:

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `enable_alt_landing` | true | bool | 总开关 |
| `alt_landing_relax_factor` | 1.5 | 1.0~2.0 | 初始放宽系数 |
| `alt_landing_max_relax` | 2.0 | ≥relax_factor | 放宽上限 |
| `publish_alt_landing_viz` | true | bool | RViz 可视化备降区 |

---

## 四、参数配置体系

### 4.1 safeland.yaml 完整参数表

```yaml
# ===== 安全阈值 =====
slope_threshold: 0.15          # rad (~8.6°)
roughness_threshold: 0.05       # m
step_threshold: 0.05            # m
min_cell_points: 5.0             # 最少点数阈值
max_valid_z: 1.0                 # m, 高度上限过滤

# ===== 降落区几何 =====
landing_size: 0.5                # m, 降落区正方形边长
landing_height_range_threshold: 0.05  # m, 落区内最大高差
landing_safety_margin: 0.1         # m, 缓冲带宽度
landing_valid_ratio_threshold: 0.98  # 缓冲带内可靠栅格占比下限

# ===== 凹陷度检测 =====
depression_bg_radius: 1.0           # m
depression_score_window: 1.0         # m
depression_score_threshold: 0.3      # [0,1], 推荐 0.3
depression_percentile_lo: 0.05
depression_percentile_hi: 0.95

# ===== landing_score 权重 =====
landing_score_w_slope: 0.35
landing_score_w_step:  0.30
landing_score_w_dep:   0.25
landing_score_w_rough: 0.10

# ===== 最优落点选择 =====
best_point_score_weight: 0.7        # 安全性权重 70%
best_point_median_window: 5            # 中值滤波窗口帧数
best_point_jump_guard_m: 1.5          # 跳变门限(m)

# ===== 备降区 =====
enable_alt_landing: true
alt_landing_relax_factor: 1.5
alt_landing_max_relax: 2.0
publish_alt_landing_viz: true

# ===== 数据记录 =====
metrics_csv_path: ""                    # 空=不记录
metrics_record_every_n_frames: 1
```

### 4.2 global_grid_map.yaml 完整参数表

```yaml
# ===== 输入/输出 =====
input_cloud_topic: /cloud_registered
map_frame: camera_init
grid_map_topic: /global_grid_map
debug_cloud_topic: /global_grid_map/points

# ===== 地图构建 =====
resolution: 0.10                       # m/格
cell_count_threshold: 0                  # 0=不限制(推荐,坑洼场景)
subscriber_queue_size: 1
publish_rate: 5.0                          # Hz
publish_debug_cloud: false

# ===== 高度过滤 =====
min_valid_z: -1.8                           # m
max_valid_z: 1.0                            # m (过滤悬空点)

# ===== 体素降采样 =====
voxel_leaf_size_x: 0.05                     # m
voxel_leaf_size_y: 0.05                     # m
voxel_leaf_size_z: 0.05                     # m

# ===== 地图范围 =====
initial_padding_x: 10.0                     # m
initial_padding_y: 10.0                     # m
expand_margin_x: 8.0                           # m
expand_margin_y: 8.0                           # m

# ===== PCD 模式 =====
pcd_mode: online                             # online | offline | both
pcd_path: ""
pcd_frame_id: camera_init
pcd_body_frame: false
pcd_extrinsic_T: [0.0, 0.0, 0.0]
pcd_extrinsic_R: [1,0,0, 0,1,0, 0, 0,0,1]
```

### 4.3 FAST-LIO mid360.yaml 关键参数

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0

preprocess:
    lidar_type: 1                # Livox serials
    blind: 0.5

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree: 360
    det_range: 100.0
    extrinsic_est_en: false
    extrinsic_T: [-0.005, 0.005, 0.047]   # LiDAR相对IMU的偏移
    extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # 单位矩阵(I)
    use_fixed_init_rot: false
    fixed_init_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    apply_mount_correction_to_odom: true    # 启用安装角校正
    gravity: [0.0, 0.0, -9.81]
    gravity_init: [0.0, 0.0, -9.81]       # 预知重力(非静止启动用)
```

---

## 五、仿真环境设计

### 5.1 无人机模型配置

**文件**: `neverlost_livox_mid360_custom.sdf`

```
模型: iris_livox_mid360_custom (基于 iris UAV)

livox_base link:
  安装位姿: (x=0, y=0, z=-0.045)  [机体底部]
  姿态: (roll=π, pitch=-π/6, yaw=0)  [Gazebo intrinsic ZYX]
  含义: 倒装朝下 + 向机头前倾30°

  内部传感器:
  ├── Livox Mid360 LiDAR
  │   ├── type: ray
  │   ├── plugin: liblivox_laser_simulation.so
  │   ├── csv_file: mid360.csv (800000行扫描模式)
  │   ├── ros_topic: /livox/lidar [CustomMsg]
  │   └── pose inside livox_base: (0, 0, 0.15)
  │
  └── IMU (gazebo_ros_imu_sensor)
      ├── bodyName: livox_base
      ├── topicName: /livox/imu
      ├── updateRateHZ: 200
      └── pose: (0.011, 0.02329, 0.105588)  [Mid360硬件规格偏移]

关节: livox_iris_joint (fixed)
  parent: neverlost::base_link
  child: livox_base
```

### 5.2 仿真世界 (nagetive_terrain.world)

**障碍物统计**: 37 个不规则障碍物模型，分布在 A/B/C/D/E 五个区域

| 类型 | 数量 | 名称示例 | 设计目的 |
|------|------|---------|----------|
| 斜坡墙 | 4 | obs_slope_A1, A2, B2, C3, D3 | 不规则倾斜墙面 |
| 碎石堆 | 12 | obs_rubble_A3a-d, B4a-c, C5a-d | 不规则碎石堆积 |
| 石柱 | 4 | obs_pillar_A4, B3, C1, E1 | 圆柱形立柱 |
| 阶梯 | 3 | obs_step_B1a-c, D1a-b | 台阶状高度突变 |
| 巨板 | 3 | obs_lean_col_A5, C2, D5 | 薄倾斜平板 |
| 巨石 | 5 | obs_boulder_A4, C1, D2, E1 | 不规则巨石 |
| 树木 | 2 | obs_tree_C4, D6 | 柱干类障碍物 |

**降落窗口**: 7~10 个平坦区域均匀分布，确保在各种参数组合下都有可选降落区。

**无人机生成位置**: `(x=1.5, y=-0.5, z=0.16)` yaw=90°

---

## 六、测试与验证体系

### 6.1 一键启动脚本

**文件**: `start_online.sh`

```
启动顺序 (含延迟等待):
  t=0s   01_px4_gazebo_mavros   (Gazebo+PX4+MAVROS, 等18s)
  t=18s  02_fast_lio              (FAST-LIO SLAM, 等5s)
  t=23s  03_global_grid_map       (栅格地图, 等3s)
  t=26s  04_safeland            (降落区检测, 等3s)
  t=29s  05_mpc_control         (MPC控制, 等3s)
  t=32s  06_super_planner       (路径规划, 等3s)
  t=35s  07_state_machine       (状态机决策, 等2s)

每个模块在独立 gnome-terminal 中运行, 日志统一写入 ~/landing/start_online_logs/
```

### 6.2 测试矩阵

| 测试类型 | 数据源 | 试验数 | 参数变量 | 输出 |
|-----------|--------|-------|---------|------|
| **模拟离线** | flat.pcd 等 | 10 trials × 10 frames | res/voxel/polar/horizon/relax | `simulated_trials.csv` |
| **真实PCD离线** | kdxt_world_downsampled.pcd 等 | 10 trials × 10 参数组 | res/voxel/polar/horizon | `real_10trials_results.csv` |
| **在线仿真(旧地形)** | nagetive_terrain.world | 10 trials | 10组参数组合 | `online_10trials_results.csv` |
| **在线仿真(新复杂地形)** | nagetive_terrain.world (v2) | 10 trials | 10组含ALZ参数 | `complex_terrain_10trials.csv` |

### 6.3 可视化工具

| 脚本 | 输入 | 图表 | 报告 |
|------|-----|------|------|
| `visualize_results.py` | simulated_trials.csv | 10张图 (热力图/参数矩阵/时间线/仪表盘) | `report.html` |
| `visualize_real_results.py` | real_10trials_results.csv | 10张图 | `report.html` |
| `visualize_complex_terrain.py` | complex_terrain_10trials.csv | 7张图 (障碍布局/热力图/参数矩阵/成功率) | `report_complex.html` |
| `summarize_round4.py` | 所有CSV | 汇总统计表 | stdout |

### 6.4 量化指标定义

| 指标 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `valid_count` | int | 有效观测栅格总数 |
| `landing_count` | int | 通过全部硬约束的安全降落格子数 |
| `best_score` | float [0,1] | 最优落点综合得分 (越高越好) |
| `alt_landing_count` | int | 备降区找到的候选数 |
| `target_jitter_m` | float | 落点相邻帧跳动距离 |
| `frame_dt_ms` | float | 单帧计算耗时 |
| `success_rate` | % | landing_count > 0 的试验比例 |

---

## 七、已知问题与修复记录

| # | 问题 | 根因 | 修复文件 | 状态 |
|---|------|------|---------|------|
| 1 | 点云向下漂移 | Set_init() 错误翻转加速度方向 | `IMU_Processing.hpp` | ✅ 已修复 |
| 2 | RViz 无点云显示 | GAZEBO_PLUGIN_PATH 缺失 | `start_online.sh` | ✅ 已修复 |
| 3 | 启动后炸机 | mount correction 公式错误 (pitch=-30°应为-30°, roll=0应为-180°) | `laserMapping.cpp` | ✅ 已修复 |
| 4 | TF树分裂导致RViz无法显示 | FAST-LIO发 camera_init→body 与 Gazebo发 world→body 冲突 | `laserMapping.cpp` + `.rviz` | ✅ 已修复 |
| 5 | Livox插件不加载 | start_online.sh 未 source livox_ws | `start_online.sh` | ✅ 已修复 |

---

## 八、文件索引

### 核心源码文件

| 文件 | 行数 | 功能 |
|------|------|------|
| `src/costmap_ws/src/safeland/src/safeland_node.cpp` | ~1025 | 降落区检测核心算法 |
| `src/costmap_ws/src/safeland/config/safeland.yaml` | ~130 | safeland 全参数配置 |
| `src/costmap_ws/src/fast_lio_global_grid_map/src/global_grid_map_node.cpp` | ~572 | 点云→栅格地图转换 |
| `src/costmap_ws/src/fast_lio_global_grid_map/config/global_grid_map.yaml` | ~104 | 地图构建参数 |
| `src/iros_challenge/src/FAST_LIO/src/IMU_Processing.hpp` | ~220 | IMU处理与重力初始化 |
| `src/iros_challenge/src/FAST_LIO/src/laserMapping.cpp` | ~1400 | FAST-LIO 主循环与里程计发布 |
| `src/iros_challenge/src/FAST_LIO/src/fast_lio_px4.cpp` | ~210 | LIO→PX4 位姿桥接 |
| `src/iros_challenge/src/FAST_LIO/config/mid360.yaml` | ~50 | FAST-LIO 外参与配置 |
| `src/iros_challenge/src/FAST_LIO/launch/mapping_mid360.launch` | ~40 | FAST-LIO launch 文件 |
| `PX4_Firmware/Tools/sitl_gazebo/models/neverlost_livox_custom/neverlost_livox_mid360_custom.sdf` | ~135 | 无人机+传感器 SDF 模型 |
| `PX4_Firmware/Tools/sitl_gazebo/worlds/nagetive_terrain.world` | ~2000 | 复杂地形仿真世界 |
| `start_online.sh` | ~126 | 一键启动脚本 |
| `tools/marsim/*.py` | 各 ~200-400 | 测试/可视化脚本 |

### 测试数据与产物

| 文件/目录 | 说明 |
|-------------|------|
| `tools/marsim_benchmark_results/simulated_trials.csv` | 模拟测试数据 |
| `tools/marsim_benchmark_results/real_10trials_results.csv` | 真实PCD测试数据 |
| `tools/marsim_benchmark_results/online_10trials_results.csv` | 在线仿真(旧地形)数据 |
| `tools/marsim_benchmark_results/complex_terrain_10trials.csv` | 在线仿真(复杂地形)数据 |
| `tools/marsim_benchmark_results/figures/` | 模拟测试图表 |
| `tools/marsim_benchmark_results/figures_complex/` | 复杂地形专项图表 |
| `tools/marsim_benchmark_results/report*.html` | HTML可视化报告 |
