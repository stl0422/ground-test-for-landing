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
        self.samples = 0
        rospy.Subscriber(args.topic, GridMap, self.callback, queue_size=1)

    def callback(self, msg):
        landing_idx = index_layer(msg, self.args.landing_layer)
        if landing_idx < 0 or landing_idx >= len(msg.data):
            return

        landing_values = msg.data[landing_idx].data
        landing_count = 0
        for value in landing_values:
            if math.isfinite(value) and value > 0.5:
                landing_count += 1

        best_score = float("nan")
        score_idx = index_layer(msg, self.args.score_layer)
        if score_idx >= 0 and score_idx < len(msg.data):
            score_values = msg.data[score_idx].data
            valid_scores = [value for value in score_values if math.isfinite(value)]
            if valid_scores:
                best_score = max(valid_scores)

        valid_count = 0
        elevation_idx = index_layer(msg, self.args.elevation_layer)
        if elevation_idx >= 0 and elevation_idx < len(msg.data):
            valid_count = finite_count(msg.data[elevation_idx].data)

        self.best = {
            "stamp": rospy.Time.now().to_sec(),
            "rows": int(msg.info.length_y / msg.info.resolution) if msg.info.resolution > 0 else 0,
            "cols": int(msg.info.length_x / msg.info.resolution) if msg.info.resolution > 0 else 0,
            "resolution": float(msg.info.resolution),
            "frame_id": msg.info.header.frame_id,
            "layers": ";".join(msg.layers),
            "valid_count": valid_count,
            "landing_count": landing_count,
            "best_score": best_score,
        }
        self.last_update = time.time()
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
    print(
        f"landing_count={row['landing_count']} "
        f"best_score={score_text} "
        f"valid_count={row['valid_count']} "
        f"frame={row['frame_id']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
