#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys
import time

import rospy
from grid_map_msgs.msg import GridMap


def index_layer(msg, name):
    try:
        return msg.layers.index(name)
    except ValueError:
        return -1


def finite_count(values):
    count = 0
    for value in values:
        if math.isfinite(value):
            count += 1
    return count


class Collector:
    def __init__(self, args):
        self.args = args
        self.best = None
        self.last_update = None
        self.prev_update = None
        self.prev_best_xy = None
        self.dt_sum = 0.0
        self.dt_max = 0.0
        self.jitter_sum = 0.0
        self.jitter_max = 0.0
        self.jitter_samples = 0
        self.samples = 0
        rospy.Subscriber(args.topic, GridMap, self.callback, queue_size=1)

    @staticmethod
    def grid_xy(msg, row, col, rows, cols):
        res = msg.info.resolution
        origin = msg.info.pose.position
        x = origin.x + (rows * 0.5 - 0.5 - float(row)) * res
        y = origin.y + (cols * 0.5 - 0.5 - float(col)) * res
        return x, y

    def callback(self, msg):
        landing_idx = index_layer(msg, self.args.landing_layer)
        if landing_idx < 0 or landing_idx >= len(msg.data):
            return

        rows = int(round(msg.info.length_x / msg.info.resolution)) if msg.info.resolution > 0 else 0
        cols = int(round(msg.info.length_y / msg.info.resolution)) if msg.info.resolution > 0 else 0
        landing_values = msg.data[landing_idx].data
        landing_count = 0
        landing_row_sum = 0.0
        landing_col_sum = 0.0
        for index, value in enumerate(landing_values):
            if math.isfinite(value) and value > 0.5:
                landing_count += 1
                if rows > 0:
                    landing_row_sum += index % rows
                    landing_col_sum += index // rows

        best_score = float("nan")
        best_xy = (float("nan"), float("nan"))
        score_idx = index_layer(msg, self.args.score_layer)
        if score_idx >= 0 and score_idx < len(msg.data):
            score_values = msg.data[score_idx].data
            best_index = -1
            for index, value in enumerate(score_values):
                if math.isfinite(value) and (best_index < 0 or value > best_score):
                    best_score = value
                    best_index = index
            if best_index >= 0 and rows > 0 and cols > 0:
                best_xy = self.grid_xy(msg, best_index % rows, best_index // rows, rows, cols)

        centroid_xy = (float("nan"), float("nan"))
        if landing_count > 0 and rows > 0 and cols > 0:
            centroid_xy = self.grid_xy(
                msg,
                landing_row_sum / float(landing_count),
                landing_col_sum / float(landing_count),
                rows,
                cols,
            )

        target_xy = best_xy if math.isfinite(best_xy[0]) else centroid_xy
        jitter = float("nan")
        if self.prev_best_xy is not None and math.isfinite(target_xy[0]):
            jitter = math.hypot(target_xy[0] - self.prev_best_xy[0],
                                target_xy[1] - self.prev_best_xy[1])
            self.jitter_sum += jitter
            self.jitter_max = max(self.jitter_max, jitter)
            self.jitter_samples += 1
        if math.isfinite(target_xy[0]):
            self.prev_best_xy = target_xy

        now = time.time()
        frame_dt = float("nan")
        if self.prev_update is not None:
            frame_dt = now - self.prev_update
            self.dt_sum += frame_dt
            self.dt_max = max(self.dt_max, frame_dt)
        self.prev_update = now

        valid_count = 0
        elevation_idx = index_layer(msg, self.args.elevation_layer)
        if elevation_idx >= 0 and elevation_idx < len(msg.data):
            valid_count = finite_count(msg.data[elevation_idx].data)

        dt_samples = max(0, self.samples)
        mean_dt = self.dt_sum / dt_samples if dt_samples > 0 else float("nan")
        mean_jitter = (self.jitter_sum / self.jitter_samples
                       if self.jitter_samples > 0 else float("nan"))

        self.best = {
            "stamp": rospy.Time.now().to_sec(),
            "rows": rows,
            "cols": cols,
            "resolution": float(msg.info.resolution),
            "frame_id": msg.info.header.frame_id,
            "layers": ";".join(msg.layers),
            "valid_count": valid_count,
            "landing_count": landing_count,
            "best_score": best_score,
            "best_x": best_xy[0],
            "best_y": best_xy[1],
            "landing_centroid_x": centroid_xy[0],
            "landing_centroid_y": centroid_xy[1],
            "target_jitter_m": jitter,
            "target_jitter_mean_m": mean_jitter,
            "target_jitter_max_m": self.jitter_max if self.jitter_samples > 0 else float("nan"),
            "frame_dt": frame_dt,
            "frame_dt_mean": mean_dt,
            "frame_dt_max": self.dt_max if dt_samples > 0 else float("nan"),
        }
        self.last_update = now
        self.samples += 1

    def wait(self):
        deadline = time.time() + self.args.timeout
        stable_deadline = None

        while not rospy.is_shutdown() and time.time() < deadline:
            if self.best is not None:
                if stable_deadline is None:
                    stable_deadline = time.time() + self.args.stable_secs
                elif self.last_update is not None and time.time() - self.last_update >= self.args.stable_secs:
                    return True
            rospy.sleep(0.1)
        return self.best is not None


def write_csv(path, row):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    exists = os.path.exists(path)
    with open(path, "a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="Collect landing metrics from /safeland/grid_map.")
    parser.add_argument("--topic", default="/safeland/grid_map")
    parser.add_argument("--landing-layer", default="landing_center")
    parser.add_argument("--score-layer", default="landing_score")
    parser.add_argument("--elevation-layer", default="elevation")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--stable-secs", type=float, default=2.0)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--tag", default="")
    parser.add_argument("--pcd", default="")
    parser.add_argument("--slope-threshold", type=float, default=float("nan"))
    parser.add_argument("--depression-threshold", type=float, default=float("nan"))
    parser.add_argument("--landing-valid-ratio-threshold", type=float, default=float("nan"))
    parser.add_argument("--landing-height-range-threshold", type=float, default=float("nan"))
    parser.add_argument("--step-threshold", type=float, default=float("nan"))
    parser.add_argument("--landing-size", type=float, default=float("nan"))
    parser.add_argument("--landing-safety-margin", type=float, default=float("nan"))
    parser.add_argument("--grid-resolution", type=float, default=float("nan"))
    parser.add_argument("--voxel-leaf-size", type=float, default=float("nan"))
    parser.add_argument("--min-cell-points", type=float, default=float("nan"))
    args = parser.parse_args()

    rospy.init_node("collect_safeland_metrics", anonymous=True)
    collector = Collector(args)
    ok = collector.wait()
    if not ok:
      return 2

    row = dict(collector.best)
    row["tag"] = args.tag
    row["pcd"] = args.pcd
    row["slope_threshold"] = args.slope_threshold
    row["depression_score_threshold"] = args.depression_threshold
    row["landing_valid_ratio_threshold"] = args.landing_valid_ratio_threshold
    row["landing_height_range_threshold"] = args.landing_height_range_threshold
    row["step_threshold"] = args.step_threshold
    row["landing_size"] = args.landing_size
    row["landing_safety_margin"] = args.landing_safety_margin
    row["grid_resolution"] = args.grid_resolution
    row["voxel_leaf_size"] = args.voxel_leaf_size
    row["min_cell_points"] = args.min_cell_points
    row["samples"] = collector.samples
    write_csv(args.csv, row)

    score_text = "nan" if not math.isfinite(row["best_score"]) else f"{row['best_score']:.3f}"
    jitter_text = (
        "nan" if not math.isfinite(row["target_jitter_mean_m"])
        else f"{row['target_jitter_mean_m']:.3f}m"
    )
    dt_text = (
        "nan" if not math.isfinite(row["frame_dt_mean"])
        else f"{row['frame_dt_mean']:.3f}s"
    )
    print(
        f"landing_count={row['landing_count']} "
        f"best_score={score_text} "
        f"valid_count={row['valid_count']} "
        f"mean_jitter={jitter_text} "
        f"mean_dt={dt_text} "
        f"frame={row['frame_id']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
