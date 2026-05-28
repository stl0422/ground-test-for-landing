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
- `PX4_Firmware`: PX4 launch and simulator integration files.
- `tools/marsim`: offline configuration generation, batch evaluation, and metrics scripts.

## Online Start

```bash
chmod +x start_online.sh stop_online.sh
./start_online.sh
```

This script opens each module in its own terminal and writes logs to `start_online_logs/`.

## Offline Benchmark

See:

- `docs/marsim_offline_integration.md`
- `tools/marsim/run_marsim_safeland_offline.sh`
- `tools/marsim/batch_eval_marsim_safeland.sh`

## Notes

- The repo is organized for ground-test landing experiments, not as a minimal code sample.
- Large generated artifacts such as `build/`, `devel/`, logs, and benchmark outputs are intentionally ignored.
