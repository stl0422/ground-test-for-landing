# Ground Bump Detection Land

This repository contains the landing detection and landing mission workflow for the UAV ground-test environment.

## What It Does

- Runs PX4, Gazebo, MAVROS, FAST-LIO, `global_grid_map`, `safeland`, planner, controller, and state machine together.
- Builds a terrain cost / safety map from LiDAR mapping results.
- Detects safe landing regions with slope, roughness, step, and depression constraints.
- Selects a landing target from valid safe cells and executes the landing workflow.

## Main Data Flow

```text
PX4 + Gazebo + MAVROS
  -> FAST-LIO
  -> global_grid_map
  -> safeland
  -> /safeland/grid_map
  -> state_machine
  -> landing goal / landing command
```

## Key Entry Points

- `start_online.sh`: start the full online simulation and landing pipeline.
- `stop_online.sh`: stop the online pipeline and clean up related processes.
- `start_offline.sh`: offline workflow entry point.
- `docs/marsim_offline_integration.md`: offline MARSIM integration and benchmark notes.

## Repository Layout

- `src/iros_challenge`: flight logic, state machine, planner, and control launch files.
- `src/costmap_ws`: `global_grid_map` and `safeland` terrain evaluation stack.
- `PX4_Firmware`: PX4 launch and simulator integration files used by `start_online.sh`.
- `tools/marsim`: offline configuration generation, batch evaluation, and metrics scripts.

### Tree

```text
.
├── README.md
├── start_online.sh
├── start_offline.sh
├── stop_online.sh
├── docs/
│   └── marsim_offline_integration.md
├── PX4_Firmware/
│   ├── launch/
│   │   └── livox_custom.launch
│   └── Tools/
│       └── sitl_gazebo/
│           ├── worlds/
│           │   └── nagetive_terrain.world
│           └── models/
│               └── neverlost_livox_custom/
│                   └── neverlost_livox_mid360_custom.sdf
├── src/
│   ├── iros_challenge/
│   │   └── src/
│   │       ├── FAST_LIO/
│   │       ├── mpc_control/
│   │       ├── state_machine/
│   │       └── super_planner/
│   └── costmap_ws/
│       └── src/
│           ├── fast_lio_global_grid_map/
│           ├── grid_map/
│           └── safeland/
└── tools/
    └── marsim/
```

## Online Start

```bash
chmod +x start_online.sh stop_online.sh
./start_online.sh
```

This script opens each module in its own terminal and writes logs to `start_online_logs/`.

### PX4 assets used by `start_online.sh`

The online launcher depends on these repository-local PX4 assets:

- `PX4_Firmware/launch/livox_custom.launch`
- `PX4_Firmware/Tools/sitl_gazebo/worlds/nagetive_terrain.world`
- `PX4_Firmware/Tools/sitl_gazebo/models/neverlost_livox_custom/neverlost_livox_mid360_custom.sdf`

The launch chain is:

```text
start_online.sh
  -> PX4_Firmware/launch/livox_custom.launch
  -> PX4 SITL + Gazebo
  -> Tools/sitl_gazebo/worlds/nagetive_terrain.world
  -> Tools/sitl_gazebo/models/neverlost_livox_custom/neverlost_livox_mid360_custom.sdf
  -> FAST-LIO / grid map / safeland / planner / state machine
```

### Expected environment

- ROS Noetic
- PX4 build tree available in the local workspace
- Gazebo and MAVROS installed
- `gnome-terminal`, `terminator`, or `xterm`

## Online vs Offline

| Mode | Main Input | Main Output | Purpose | Typical Entry |
| --- | --- | --- | --- | --- |
| Online simulation | PX4 + Gazebo + MAVROS + live LiDAR mapping | `landing_center`, `landing_score`, landing command | Validate the full closed loop in simulation | `./start_online.sh` |
| Offline benchmark | Static PCD / MARSIM replay | Safety metrics and landing map evaluation | Tune `safeland` thresholds and compare parameter sets | `./tools/marsim/run_marsim_safeland_offline.sh` |
| Batch evaluation | Map list + parameter sweep | CSV benchmark results | Compare thresholds across many scans | `./tools/marsim/batch_eval_marsim_safeland.sh` |

## Optimization Roadmap

Detailed plan: `docs/optimization_plan.md`

### 1. Environment complexity

The current Gazebo terrain should be extended with randomized tree and cylinder obstacles in the world file.

Placement rules:

- Obstacles should be added directly in `PX4_Firmware/Tools/sitl_gazebo/worlds/nagetive_terrain.world`.
- Trees and cylinders should not be placed on pits, bumps, or steep local terrain.
- Obstacle density should stay moderate: not clustered enough to block the whole map, and not sparse enough to make avoidance trivial.
- Keep multiple open landing footprints of at least `0.5 m x 0.5 m`.
- Keep enough free space around candidate landing areas so `landing_safety_margin` remains meaningful.

Recommended generation policy:

- Predefine allowed placement zones on locally flat terrain.
- Randomly sample obstacle positions only inside those zones.
- Enforce a minimum distance between obstacles.
- Enforce a minimum distance between obstacles and known safe landing patches.

### 2. Stability and efficiency metrics

The landing detector should be evaluated with both stability and runtime metrics.

Key metrics:

- Grid resolution versus landing accuracy.
- Point cloud downsampling voxel size versus terrain detail preservation.
- `safeland` runtime per frame.
- Number of valid landing cells over time.
- Landing target jitter between consecutive frames.
- False rejection rate on valid landing areas.
- False acceptance rate on unsafe terrain.

Tuning direction:

- Smaller grid cells improve terrain detail but increase map size and filtering cost.
- Larger grid cells reduce computation but can hide small bumps, pits, and steps.
- Stronger point cloud downsampling improves runtime but may remove terrain features needed by `slope`, `roughness`, `step`, and `depression_score`.
- The practical target is to keep the detector stable across frames while maintaining enough terrain detail for a `0.5 m x 0.5 m` landing footprint.

### 3. Emergency safety behavior

The state machine should support a fallback landing strategy when no valid landing region is found.

Expected behavior:

- Continue searching during the configured `safeland_timeout` window.
- If no `landing_center` candidate is available, switch to a backup landing policy.
- Prefer known backup zones that were prevalidated in the map or world file.
- If multiple backup zones exist, select the nearest reachable one with planner support.
- If no backup zone is reachable, hover or return to a safe loiter point instead of descending onto unknown terrain.

Recommended implementation path:

- Add a backup landing zone list to the state machine config.
- Add a backup-zone selector that considers distance, reachability, and minimum terrain safety.
- Log the reason for fallback activation for later benchmark analysis.
- Record whether the final landing target came from `safeland` or from the backup policy.

## Offline Benchmark

See:

- `docs/marsim_offline_integration.md`
- `tools/marsim/run_marsim_safeland_offline.sh`
- `tools/marsim/batch_eval_marsim_safeland.sh`

## Notes

- The repo is organized for ground-test landing experiments, not as a minimal code sample.
- Large generated artifacts such as `build/`, `devel/`, logs, and benchmark outputs are intentionally ignored.
- `start_online.sh` still assumes your local PX4 workspace paths are available under the same home-directory layout as described in the script comments.
