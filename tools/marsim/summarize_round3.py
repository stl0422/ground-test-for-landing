#!/usr/bin/env python3

import csv
import math
import sys


KEY_FIELDS = [
    "grid_resolution",
    "voxel_leaf_size",
    "min_cell_points",
]


def parse_float(text):
    if text is None or text == "":
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


def parse_int(text):
    if text is None or text == "":
        return 0
    return int(text)


def load_rows(path):
    rows = []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            row["landing_count"] = parse_int(row.get("landing_count"))
            row["valid_count"] = parse_int(row.get("valid_count"))
            row["best_score"] = parse_float(row.get("best_score"))
            for key in KEY_FIELDS:
                row[key] = parse_float(row.get(key))
            rows.append(row)
    deduped = {}
    for row in rows:
        key = tuple(row[key] for key in KEY_FIELDS)
        deduped[key] = row
    return list(deduped.values())


def score_key(row):
    score = row["best_score"]
    if not math.isfinite(score):
        score = -1.0
    return (row["landing_count"], score, row["valid_count"])


def summarize_dimension(rows, field):
    values = sorted(set(row[field] for row in rows))
    summary = []
    for value in values:
        subset = [row for row in rows if row[field] == value]
        positive = [row for row in subset if row["landing_count"] > 0]
        best = max((row["best_score"] for row in subset if math.isfinite(row["best_score"])), default=math.nan)
        summary.append(
            {
                "value": value,
                "positive_runs": len(positive),
                "total_runs": len(subset),
                "max_landing_count": max((row["landing_count"] for row in subset), default=0),
                "mean_landing_count": sum(row["landing_count"] for row in subset) / len(subset) if subset else 0.0,
                "max_valid_count": max((row["valid_count"] for row in subset), default=0),
                "max_best_score": best,
            }
        )
    return summary


def print_summary(title, stats):
    print(title)
    for item in stats:
        best_text = "nan" if not math.isfinite(item["max_best_score"]) else f"{item['max_best_score']:.3f}"
        print(
            f"  value={item['value']} positive_runs={item['positive_runs']}/{item['total_runs']} "
            f"max_landing_count={item['max_landing_count']} mean_landing_count={item['mean_landing_count']:.2f} "
            f"max_valid_count={item['max_valid_count']} max_best_score={best_text}"
        )


def main():
    if len(sys.argv) != 2:
      print(f"usage: {sys.argv[0]} RESULT_CSV", file=sys.stderr)
      return 2
    rows = load_rows(sys.argv[1])
    print(f"unique_rows={len(rows)}")
    print("top_combinations")
    for row in sorted(rows, key=score_key, reverse=True)[:10]:
        best_text = "nan" if not math.isfinite(row["best_score"]) else f"{row['best_score']:.3f}"
        print(
            f"  landing_count={row['landing_count']} best_score={best_text} valid_count={row['valid_count']} "
            f"res={row['grid_resolution']} voxel={row['voxel_leaf_size']} min_pts={row['min_cell_points']}"
        )
    print_summary("grid_resolution", summarize_dimension(rows, "grid_resolution"))
    print_summary("voxel_leaf_size", summarize_dimension(rows, "voxel_leaf_size"))
    print_summary("min_cell_points", summarize_dimension(rows, "min_cell_points"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
