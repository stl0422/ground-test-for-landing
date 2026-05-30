# Optimization Plan

This document turns the current roadmap into concrete engineering work for the landing detection project.

## 1. Environment Complexity

The online Gazebo scene should include additional trees and cylinder obstacles. These obstacles should be edited directly in:

```text
PX4_Firmware/Tools/sitl_gazebo/worlds/nagetive_terrain.world
```

### Placement Requirements

- Do not place obstacles on pits, bumps, steep slopes, or sharp height transitions.
- Keep obstacle density moderate. The scene should require path planning, but it should not become a blocked maze.
- Preserve several open landing patches of at least `0.5 m x 0.5 m`.
- Keep obstacle clearance around candidate landing patches so `landing_safety_margin` can still reject risky edges.
- Keep the takeoff area clear enough for PX4 arming and stable initial hover.

### Recommended World Editing Strategy

Use deterministic groups of obstacles instead of fully random world edits committed to the repository.

Recommended approach:

- Define several flat placement zones in the world file.
- Within each zone, manually place a small randomized-looking set of trees and cylinders.
- Keep minimum spacing between obstacles, for example `1.0 m` to `1.5 m`.
- Keep a larger exclusion radius around intended backup landing zones, for example `0.8 m` to `1.2 m`.
- Use different obstacle radii and heights so the LiDAR map is more realistic.

### Candidate Obstacle Types

Trees:

- Use thin trunks with larger visual canopies if needed.
- Collision should stay simple, preferably cylinder or box collision.
- Do not make the canopy collision block large landing areas unless that is intentional.

Cylinders:

- Use several radii, for example `0.10 m`, `0.15 m`, and `0.20 m`.
- Use heights between `0.5 m` and `1.5 m`.
- Place them as sparse clutter, not as a wall.

### Acceptance Criteria

- PX4 can still spawn and arm reliably.
- FAST-LIO can build a stable map without immediate failure.
- `safeland` can still find at least one valid landing area in normal scenes.
- At least one scene should intentionally force fallback behavior for emergency testing.

## 2. Stability And Efficiency Quantification

The detector should be measured with repeatable metrics instead of relying only on RViz inspection.

### Metrics To Record

Runtime:

- `global_grid_map` update time per frame.
- `safeland` update time per frame.
- End-to-end delay from point cloud update to `/safeland/grid_map`.

Map quality:

- Number of valid grid cells.
- Number of reliable grid cells after `min_cell_points`.
- Number of `landing_center=1` cells.
- Best `landing_score`.
- Landing target displacement between consecutive frames.

Decision quality:

- False acceptance on pits, bumps, steps, and sparse regions.
- False rejection on flat terrain.
- Time until the first valid landing area appears.
- Time until the landing target becomes stable.

### Grid Resolution Tradeoff

Smaller grid cells:

- Preserve small terrain details better.
- Improve sensitivity to pits, bumps, and local steps.
- Increase grid size and sliding-window cost.
- Require enough point density per cell to avoid sparse-cell rejection.

Larger grid cells:

- Reduce CPU cost and memory use.
- Make the output smoother and usually more stable.
- Can hide small unsafe features.
- Can make a `0.5 m x 0.5 m` footprint too coarse to evaluate.

Practical direction:

- Keep the grid resolution fine enough that a `0.5 m x 0.5 m` landing footprint spans multiple cells.
- Treat `0.10 m` as a reasonable starting point for footprint-level evaluation.
- Benchmark nearby values instead of picking by inspection, for example `0.08 m`, `0.10 m`, and `0.15 m`.

### Point Cloud Downsampling Tradeoff

More aggressive downsampling:

- Reduces `global_grid_map` cost.
- Reduces duplicate points and noise.
- Can erase narrow bumps, pit rims, and step edges.
- Can lower per-cell point count and cause reliable cells to be rejected.

Less aggressive downsampling:

- Preserves local terrain geometry.
- Improves slope, roughness, step, and depression calculation.
- Increases CPU cost and map update latency.

Recommended benchmark:

- Sweep voxel sizes together with grid resolution.
- For each pair, record runtime, `landing_count`, best score, and target jitter.
- Reject settings that make the target unstable even if runtime is good.

### Suggested Sweep Matrix

```text
grid_resolution: 0.08, 0.10, 0.15
downsample_voxel: 0.05, 0.08, 0.10, 0.15
min_cell_points: 2, 3, 5
```

The goal is not maximum `landing_count`. The target is stable detection, low false acceptance, and runtime that remains compatible with online flight.

## 3. Emergency Safety Mechanism

When `safeland` cannot find a valid landing region, the state machine should not descend blindly. It should switch to a controlled fallback policy.

### Fallback Priority

1. Use a valid `safeland` target if available.
2. If no target appears before `safeland_timeout`, fly to the nearest configured backup landing zone.
3. If the nearest backup zone is unreachable, try the next backup zone.
4. If no backup zone is reachable, hover at a safe loiter point or return to a known safe point.
5. Only perform forced landing as the final emergency behavior.

### Backup Zone Configuration

Add backup zones to the state machine config:

```yaml
backup_landing_zones:
  - [1.5, -0.5, 0.0, 1.0]
  - [0.0,  0.0, 0.0, 1.0]
```

Suggested format:

```text
[x, y, z, radius]
```

The `radius` marks the minimum clear area around the backup zone center.

### State Machine Changes

Recommended behavior:

- Track whether the final target comes from `safeland` or backup selection.
- Publish a clear log when fallback is activated.
- Stop updating the selected target after entering the final landing approach.
- Keep `land_timeout_secs` as the last-stage protection.

Useful states:

```text
ST_WAIT_SAFELAND
  -> ST_FLY_TO_LAND          when safeland target exists
  -> ST_FLY_TO_BACKUP_LAND   when timeout occurs and backup exists
  -> ST_HOVER_FAILSAFE       when no target is available
```

### Acceptance Criteria

- In normal scenes, the vehicle still lands on a `safeland` target.
- In blocked or over-constrained scenes, the vehicle switches to a configured backup zone.
- The log clearly records why fallback was used.
- The final landing target source is available for offline analysis.
