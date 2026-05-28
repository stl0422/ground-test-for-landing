# MARSIM 离线地图集成方案

## 目标

把当前体系里的离线点云地图直接接到 `MARSIM -> global_grid_map -> safeland`，用于：

- 用固定 `pcd` 重复生成局部扫描，做参数对比
- 不依赖 Gazebo/PX4，专注评估 `safeland` 的坡度/粗糙度/凹陷阈值
- 以后再把 `super_planner` 或状态机挂上去做更完整的离线回放

## 当前接口对齐

当前新增的离线链路是：

```text
FAST-LIO 世界系 PCD
  -> MARSIM(perfect_drone_sim / marsim_render)
  -> /global_pc              # 当前默认，已验证稳定
  -> global_grid_map_node
  -> /global_grid_map
  -> safeland_node
  -> /safeland/grid_map
```

辅助接口：

- `/lidar_slam/odom` 由 `perfect_drone_sim` 发布
- launch 中已自动 relay 到 `/drone_odometry`
- `world -> camera_init` 提供了恒等静态 TF，便于复用现有 RViz 配置

## 为什么这样集成

有两条离线路径，职责不同：

1. `global_grid_map` 自带 `pcd_mode:=offline`
   适合“直接对静态整图做高程分析”，最快，但不模拟扫描稀疏性。

2. MARSIM 读 PCD 后重新发点云
   当前稳定入口是 `/global_pc`：适合参数 benchmark，保证地图一次性完整导入。
   可选入口是 `/cloud_registered`：更接近扫描过程，但对地面朝下场景还需要继续调 MARSIM 的局部扫描建模。

你这次要的数据增强和 benchmark，应该以第 2 条为主。

## 已完成的环境与改动

- 本机依赖已具备：`ROS Noetic / PCL / Eigen / GLFW / GLEW / libdw / mavros`
- `marsim_render` 已支持绝对路径 `pcd_name`
  文件：[config.hpp](/home/fractal/super_ws/src/SUPER/mars_uav_sim/marsim_render/include/marsim_render/config.hpp:57)
- 新增离线 launch：
  [marsim_safeland_offline.launch](/home/fractal/landing/src/costmap_ws/src/safeland/launch/marsim_safeland_offline.launch:1)
- 新增 MARSIM Mid-360 模板：
  [marsim_mid360_template.yaml](/home/fractal/landing/src/costmap_ws/src/safeland/config/marsim_mid360_template.yaml:1)
- 新增结果采集脚本：
  [collect_safeland_metrics.py](/home/fractal/landing/src/costmap_ws/src/safeland/scripts/collect_safeland_metrics.py:1)
- 新增启动与批量脚本：
  [make_marsim_offline_config.sh](/home/fractal/landing/tools/marsim/make_marsim_offline_config.sh:1)
  [run_marsim_safeland_offline.sh](/home/fractal/landing/tools/marsim/run_marsim_safeland_offline.sh:1)
  [batch_eval_marsim_safeland.sh](/home/fractal/landing/tools/marsim/batch_eval_marsim_safeland.sh:1)

## 默认姿态与初始位姿

- `make_marsim_offline_config.sh` 会优先读取 `pcd` 头里的 `VIEWPOINT`
  作为默认 `x/y`
- 若 `VIEWPOINT` 存在，默认 `z = viewpoint_z + 1.5`
- 默认姿态按你在线仿真的 Mid-360 安装方式设置：
  `roll=pi`，`pitch=-pi/6`，`yaw=0`

这一步是为了让离线 benchmark 尽量贴近你现在的机载雷达朝下安装方式。

## 单图离线可视化

```bash
cd /home/fractal/landing
./tools/marsim/run_marsim_safeland_offline.sh \
  --pcd /home/fractal/iros_challenge/src/FAST_LIO/PCD/scans.pcd \
  --open-rviz true
```

这会做三件事：

- 用 MARSIM 读取 `scans.pcd`
- 发布 `/global_pc` 作为离线 benchmark 默认输入
- 建立 `/global_grid_map` 并运行 `safeland`

如果后面你想试局部扫描模式，可以显式改为：

```bash
roslaunch safeland marsim_safeland_offline.launch \
  marsim_config:=/tmp/landing_marsim_offline.yaml \
  input_cloud_topic:=/cloud_registered \
  open_rviz:=false
```

## 单图离线统计

先启动离线链路：

```bash
cd /home/fractal/landing
./tools/marsim/run_marsim_safeland_offline.sh \
  --pcd /home/fractal/iros_challenge/src/FAST_LIO/PCD/scans.pcd \
  --open-rviz false
```

另开一个终端采集指标：

```bash
source /opt/ros/noetic/setup.bash
source /home/fractal/super_ws/devel/setup.bash
source /home/fractal/landing/devel/setup.bash --extend
rosrun safeland collect_safeland_metrics.py \
  --csv /home/fractal/landing/marsim_benchmark_results/single_run.csv \
  --tag baseline \
  --pcd /home/fractal/iros_challenge/src/FAST_LIO/PCD/scans.pcd
```

输出字段包括：

- `valid_count`
- `landing_count`
- `best_score`
- `frame_id`
- `layers`

## 批量参数扫描

先准备一个地图列表文件，比如：

```text
/home/fractal/iros_challenge/src/FAST_LIO/PCD/scans.pcd
/home/fractal/landing/src/costmap_ws/src/grid_map/grid_map_pcl/data/input_cloud.pcd
```

然后运行：

```bash
cd /home/fractal/landing
./tools/marsim/batch_eval_marsim_safeland.sh \
  --maps /path/to/maps.txt \
  --csv /home/fractal/landing/marsim_benchmark_results/results.csv \
  --slope-values 0.12,0.15,0.18 \
  --dep-values 0.2,0.3,0.4
```

默认会扫描：

- `slope_threshold`
- `depression_score_threshold`

结果会写入同一个 CSV，后续你可以继续扩展到：

- `roughness_threshold`
- `step_threshold`
- `landing_height_range_threshold`
- `landing_safety_margin`

## 建议的 benchmark 维度

优先先扫这四类：

- 平整地图：验证不会误检坑洼
- 纯凹陷地图：验证 `depression_score_threshold`
- 凸起+坑洼混合地图：验证 `step/slope/roughness` 和凹陷约束是否冲突
- 稀疏覆盖地图：验证 `min_cell_points` 和 `landing_valid_ratio_threshold`

## 在线与离线如何分工

推荐分工：

- 在线：继续用 Gazebo/PX4 验证整条飞控与降落闭环
- 离线静态整图：用 `pcd_mode:=offline` 快速做高程图分析
- 离线扫描模拟：用 MARSIM 做参数扫描和数据增强

这样三条链路互补，不冲突，也不会把在线调试复杂度继续抬高。

## 第二轮 benchmark 结果

`scans.pcd` 第二轮固定以下参数：

- `slope_threshold = 0.15`
- `depression_score_threshold = 0.30`
- `landing_valid_ratio_threshold = 0.98`

批量扫描这四个硬约束：

- `landing_height_range_threshold = {0.05, 0.08, 0.10}`
- `step_threshold = {0.05, 0.08, 0.10}`
- `landing_size = {0.5, 0.4, 0.3}`
- `landing_safety_margin = {0.10, 0.05, 0.00}`

结果文件：

- `/home/fractal/landing/marsim_benchmark_results/scans_pcd_round2.csv`

结论：

- 总组合数：`81`
- 有正例的组合数：`9`
- 所有正例都满足：
  - `landing_size = 0.3`
  - `landing_safety_margin = 0.0`
- `landing_height_range_threshold` 在 `0.05/0.08/0.10` 三档上结果完全一致，不是主瓶颈。
- `step_threshold` 放宽会提升 `best_score`，但单独放宽并不能救活 `0.4m` 或 `0.5m` 落区。

最佳组合：

- `landing_height_range_threshold = 0.05/0.08/0.10`
- `step_threshold = 0.10`
- `landing_size = 0.3`
- `landing_safety_margin = 0.0`
- `landing_count = 2`
- `best_score = 0.709`

参数维度总结：

- `landing_size`
  - `0.5`: `0/27` 正例
  - `0.4`: `0/27` 正例
  - `0.3`: `9/27` 正例
- `landing_safety_margin`
  - `0.10`: `0/27` 正例
  - `0.05`: `0/27` 正例
  - `0.00`: `9/27` 正例
- `step_threshold`
  - `0.05`: `3/27` 正例, `max_best_score = 0.605`
  - `0.08`: `3/27` 正例, `max_best_score = 0.683`
  - `0.10`: `3/27` 正例, `max_best_score = 0.709`
- `landing_height_range_threshold`
  - `0.05`: `3/27` 正例
  - `0.08`: `3/27` 正例
  - `0.10`: `3/27` 正例

因此，对这张离线 `scans.pcd`，先卡死的不是凹陷分数、坡度，也不是落区内部高差，而是：

1. `landing_size`
2. `landing_safety_margin`
3. `step_threshold`

推荐离线 benchmark 配置：

- `landing_size = 0.3`
- `landing_safety_margin = 0.0`
- `step_threshold = 0.08 ~ 0.10`
- `landing_height_range_threshold = 0.05` 保持不动

这套推荐值适合离线稀疏点云地图评估，不建议直接回灌到在线飞行参数；在线模式仍应保持更保守的落区尺寸和安全边界。

## 第三轮 benchmark 结果

目标：在第二轮最佳降落参数基础上，继续扫描地图侧参数，让离线 `scans.pcd` 更接近在线局部图，稳定算出可降落区域。

固定降落参数：

- `slope_threshold = 0.15`
- `depression_score_threshold = 0.30`
- `landing_valid_ratio_threshold = 0.98`
- `landing_height_range_threshold = 0.05`
- `step_threshold = 0.10`
- `landing_size = 0.3`
- `landing_safety_margin = 0.0`

扫描参数：

- `grid_resolution = {0.10, 0.12, 0.15}`
- `voxel_leaf_size = {0.05, 0.08, 0.10}`
- `min_cell_points = {5, 3, 2}`

结果文件：

- `/home/fractal/landing/marsim_benchmark_results/scans_pcd_round3.csv`

最佳组合：

- `grid_resolution = 0.10`
- `voxel_leaf_size = 0.05`
- `min_cell_points = 2`
- `landing_count = 180`
- `best_score = 0.978`
- `valid_count = 2158`

次优保守组合：

- `grid_resolution = 0.10`
- `voxel_leaf_size = 0.05`
- `min_cell_points = 3`
- `landing_count = 153`
- `best_score = 0.977`
- `valid_count = 2158`

维度结论：

- `voxel_leaf_size` 是最敏感的地图参数
  - `0.05`: `9/9` 正例，效果最好
  - `0.08`: `6/9` 正例，明显退化
  - `0.10`: `2/9` 正例，基本不可用
- `min_cell_points` 放宽能显著提升离线候选点数量
  - `2`: `7/9` 正例，`max_landing_count = 180`
  - `3`: `6/9` 正例，`max_landing_count = 153`
  - `5`: `4/9` 正例，`max_landing_count = 51`
- `grid_resolution` 存在 tradeoff
  - `0.10`: 候选点最多，保留细节最好
  - `0.12`: 候选点下降，但仍可用
  - `0.15`: 正例比例更高，但 `valid_count` 明显下降，地图过粗

推荐离线默认值：

- `grid_resolution = 0.10`
- `voxel_leaf_size = 0.05`
- `min_cell_points = 3`

如果目标是“尽量多找出候选降落区”，可进一步放到：

- `min_cell_points = 2`

这套参数已经能让离线模式稳定算出可降落区域，不再出现“全图无落点”的情况。
