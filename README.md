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

## Offline Benchmark

See:

- `docs/marsim_offline_integration.md`
- `tools/marsim/run_marsim_safeland_offline.sh`
- `tools/marsim/batch_eval_marsim_safeland.sh`

## Notes

- The repo is organized for ground-test landing experiments, not as a minimal code sample.
- Large generated artifacts such as `build/`, `devel/`, logs, and benchmark outputs are intentionally ignored.
- `start_online.sh` still assumes your local PX4 workspace paths are available under the same home-directory layout as described in the script comments.
