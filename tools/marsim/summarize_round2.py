#!/usr/bin/env python3

import csv
import math
import statistics
import sys
from collections import OrderedDict, defaultdict


def parse_float(value):
    try:
        return float(value)
    except Exception:
        return float("nan")


def parse_int(value):
    try:
        return int(value)
    except Exception:
        return 0


def keep_last_rows(path):
    rows = list(csv.DictReader(open(path)))
    latest = OrderedDict()
    for row in rows:
        key = (
            row["landing_height_range_threshold"],
            row["step_threshold"],
            row["landing_size"],
            row["landing_safety_margin"],
        )
        latest[key] = row
    return list(latest.values())


def summarize_dimension(rows, field):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row[field]].append(row)
    summary = []
    for key in sorted(grouped, key=lambda x: float(x)):
        subset = grouped[key]
        landing_counts = [parse_int(r["landing_count"]) for r in subset]
        best_scores = [parse_float(r["best_score"]) for r in subset if math.isfinite(parse_float(r["best_score"]))]
        positive = sum(1 for count in landing_counts if count > 0)
        summary.append(
            {
                "value": key,
                "positive_runs": positive,
                "total_runs": len(subset),
                "max_landing_count": max(landing_counts) if landing_counts else 0,
                "mean_landing_count": statistics.mean(landing_counts) if landing_counts else 0.0,
                "max_best_score": max(best_scores) if best_scores else float("nan"),
            }
        )
    return summary


def print_summary(name, rows):
    print(name)
    for row in rows:
        score_text = "nan" if not math.isfinite(row["max_best_score"]) else f"{row['max_best_score']:.3f}"
        print(
            f"  value={row['value']} "
            f"positive_runs={row['positive_runs']}/{row['total_runs']} "
            f"max_landing_count={row['max_landing_count']} "
            f"mean_landing_count={row['mean_landing_count']:.2f} "
            f"max_best_score={score_text}"
        )


def print_best(rows):
    scored = []
    for row in rows:
        landing_count = parse_int(row["landing_count"])
        best_score = parse_float(row["best_score"])
        scored.append(
            (
                landing_count,
                best_score if math.isfinite(best_score) else -1.0,
                row,
            )
        )
    scored.sort(key=lambda item: (item[0], item[1]), reverse=True)
    print("top_combinations")
    for landing_count, best_score, row in scored[:10]:
        score_text = "nan" if best_score < 0 else f"{best_score:.3f}"
        print(
            f"  landing_count={landing_count} best_score={score_text} "
            f"dz={row['landing_height_range_threshold']} "
            f"step={row['step_threshold']} "
            f"size={row['landing_size']} "
            f"margin={row['landing_safety_margin']}"
        )


def main():
    if len(sys.argv) != 2:
        print("usage: summarize_round2.py <csv>", file=sys.stderr)
        return 2
    rows = keep_last_rows(sys.argv[1])
    print(f"unique_rows={len(rows)}")
    print_best(rows)
    print_summary("landing_height_range_threshold", summarize_dimension(rows, "landing_height_range_threshold"))
    print_summary("step_threshold", summarize_dimension(rows, "step_threshold"))
    print_summary("landing_size", summarize_dimension(rows, "landing_size"))
    print_summary("landing_safety_margin", summarize_dimension(rows, "landing_safety_margin"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
