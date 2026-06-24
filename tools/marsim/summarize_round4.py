#!/usr/bin/env python3
"""
summarize_round4.py — 第4轮批量评估结果汇总分析

分析维度：
  1. 网格分辨率 (grid_resolution) 对落区数量、最优得分、计算耗时的影响
  2. 体素降采样大小 (voxel_leaf_size) 对精度与效率的平衡
  3. 备降区放宽系数 (alt_relax_factor) 对备降命中率和得分的影响
  4. 是否启用备降区 (enable_alt_landing) 的对比
  5. 传感器参数 (polar_res, sensing_horizon) 模拟的环境复杂度影响

用法：
    python3 summarize_round4.py RESULT_CSV
    python3 summarize_round4.py RESULT_CSV --top 15

输出格式：
    - 各维度分析（正运行率、最大/均值落区数、最优得分）
    - 备降区专项分析（主区率、备降命中率、全失败率）
    - Top-N 最佳组合
"""
import csv
import math
import sys
from collections import defaultdict, OrderedDict
from typing import Dict, List, Optional


# ===========================================================================
# 字段定义（与 batch_eval 脚本保持一致）
# ===========================================================================
KEY_FIELDS = [
    "grid_resolution",
    "voxel_leaf_size",
    "alt_relax_factor",
    "enable_alt_landing",
    "polar_res",
    "sensing_horizon",
]


def parse_float(text: Optional[str]) -> float:
    if text is None or text == "":
        return math.nan
    try:
        return float(text)
    except ValueError:
        return math.nan


def parse_int(text: Optional[str]) -> int:
    if text is None or text == "":
        return 0
    try:
        return int(text)
    except ValueError:
        return 0


def parse_bool(text: Optional[str]) -> bool:
    if text is None:
        return False
    return text.strip().lower() in ("true", "1", "yes")


# ===========================================================================
# 数据加载
# ===========================================================================

def load_rows(path: str) -> List[Dict]:
    rows = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            # 数值化
            row["landing_count"]     = parse_int(row.get("landing_count"))
            row["alt_landing_count"] = parse_int(row.get("alt_landing_count", "0"))
            row["valid_count"]       = parse_int(row.get("valid_count"))
            row["best_score"]        = parse_float(row.get("best_score"))
            row["alt_best_score"]    = parse_float(row.get("alt_best_score", ""))
            row["dt_ms"]             = parse_float(row.get("dt_ms", ""))
            for key in KEY_FIELDS:
                v = row.get(key, "")
                if key == "enable_alt_landing":
                    row[key] = v.strip().lower() if v else "unknown"
                else:
                    row[key] = parse_float(v)
            rows.append(row)

    # 去重：同 tag 保留最后一条
    deduped: OrderedDict = OrderedDict()
    for row in rows:
        deduped[row.get("tag", row.get("pcd", ""))] = row
    return list(deduped.values())


# ===========================================================================
# 统计工具
# ===========================================================================

def alt_status(row: Dict) -> str:
    """判断该行的备降区状态"""
    if row["landing_count"] > 0:
        return "PRIMARY_OK"
    if row["alt_landing_count"] > 0:
        return "ALT_FOUND"
    return "ALL_FAILED"


def summarize_dimension(rows: List[Dict], field: str) -> List[Dict]:
    """按某一维度分组，统计正运行率、落区数、得分、耗时"""
    grouped: Dict[str, List] = defaultdict(list)
    for row in rows:
        grouped[str(row[field])].append(row)

    summary = []
    for key in sorted(grouped, key=lambda x: float(x) if x not in ("nan", "unknown", "true", "false") else 0):
        subset = grouped[key]
        landing_counts = [r["landing_count"] for r in subset]
        alt_counts     = [r["alt_landing_count"] for r in subset]
        best_scores    = [r["best_score"] for r in subset if math.isfinite(r["best_score"])]
        alt_scores     = [r["alt_best_score"] for r in subset if math.isfinite(r.get("alt_best_score", math.nan))]
        dt_vals        = [r["dt_ms"] for r in subset if math.isfinite(r.get("dt_ms", math.nan))]

        primary_ok  = sum(1 for r in subset if r["landing_count"] > 0)
        alt_found   = sum(1 for r in subset if r["landing_count"] == 0 and r["alt_landing_count"] > 0)
        all_failed  = sum(1 for r in subset if r["landing_count"] == 0 and r["alt_landing_count"] == 0)

        summary.append({
            "value":             key,
            "total_runs":        len(subset),
            "primary_ok":        primary_ok,
            "alt_found":         alt_found,
            "all_failed":        all_failed,
            "primary_rate":      primary_ok / len(subset) if subset else 0.0,
            "alt_hit_rate":      alt_found / max(len(subset) - primary_ok, 1),
            "max_landing":       max(landing_counts) if landing_counts else 0,
            "mean_landing":      sum(landing_counts) / len(landing_counts) if landing_counts else 0.0,
            "max_alt_landing":   max(alt_counts) if alt_counts else 0,
            "max_best_score":    max(best_scores) if best_scores else math.nan,
            "max_alt_score":     max(alt_scores) if alt_scores else math.nan,
            "mean_dt_ms":        sum(dt_vals) / len(dt_vals) if dt_vals else math.nan,
        })
    return summary


def print_summary(title: str, stats: List[Dict], show_alt: bool = True) -> None:
    print(f"\n{'='*70}")
    print(title)
    print(f"{'='*70}")
    for item in stats:
        best_txt = f"{item['max_best_score']:.3f}" if math.isfinite(item["max_best_score"]) else "nan"
        alt_txt  = f"{item['max_alt_score']:.3f}"  if math.isfinite(item["max_alt_score"])  else "nan"
        dt_txt   = f"{item['mean_dt_ms']:.1f}ms"   if math.isfinite(item["mean_dt_ms"])      else "nan"
        print(
            f"  {item['value']:>10} | "
            f"runs={item['total_runs']:>3} | "
            f"primary_ok={item['primary_ok']:>3}/{item['total_runs']:>3}"
            f"({item['primary_rate']*100:>5.1f}%) | "
            f"alt_found={item['alt_found']:>3} "
            f"alt_hit={item['alt_hit_rate']*100:>5.1f}% | "
            f"all_fail={item['all_failed']:>3} | "
            f"max_land={item['max_landing']:>5} mean_land={item['mean_landing']:>6.1f} | "
            f"best_score={best_txt} alt_score={alt_txt} | "
            f"dt={dt_txt}"
        )


def print_top_combinations(rows: List[Dict], top_n: int = 15) -> None:
    def score_key(r: Dict):
        s = r["best_score"] if math.isfinite(r["best_score"]) else -1.0
        s_alt = r.get("alt_best_score", math.nan)
        s_alt = s_alt if math.isfinite(s_alt) else -1.0
        return (r["landing_count"], s, r["alt_landing_count"], s_alt)

    ranked = sorted(rows, key=score_key, reverse=True)[:top_n]
    print(f"\n{'='*70}")
    print(f"Top {top_n} combinations (by primary landing_count then score, then alt)")
    print(f"{'='*70}")
    for r in ranked:
        bs = f"{r['best_score']:.3f}" if math.isfinite(r["best_score"]) else "nan"
        ab = f"{r.get('alt_best_score', math.nan):.3f}" if math.isfinite(r.get("alt_best_score", math.nan)) else "nan"
        dt = f"{r['dt_ms']:.1f}" if math.isfinite(r.get("dt_ms", math.nan)) else "nan"
        print(
            f"  land={r['landing_count']:>5} score={bs} | "
            f"alt_land={r['alt_landing_count']:>4} alt_score={ab} | "
            f"res={r['grid_resolution']} voxel={r['voxel_leaf_size']} "
            f"relax={r['alt_relax_factor']} alt_en={r['enable_alt_landing']} | "
            f"polar={r['polar_res']} horizon={r['sensing_horizon']} | "
            f"dt={dt}ms"
        )


def print_alt_comparison(rows: List[Dict]) -> None:
    """专项分析：enable_alt_landing=true vs false 的对比"""
    print(f"\n{'='*70}")
    print("备降区专项对比：enable_alt_landing = true vs false")
    print(f"{'='*70}")

    # 筛选出 enable=true 和 enable=false 的行，匹配相同的其他参数
    groups: Dict[tuple, Dict] = {}
    for r in rows:
        key = (r["grid_resolution"], r["voxel_leaf_size"],
               r["polar_res"], r["sensing_horizon"])
        en = r["enable_alt_landing"]
        if key not in groups:
            groups[key] = {}
        groups[key][en] = r

    paired = [(v["true"], v["false"])
              for v in groups.values()
              if "true" in v and "false" in v]

    if not paired:
        print("  (无配对数据，需要同时有 enable_alt=true 和 false 的运行记录)")
        return

    land_gain = 0
    alt_save  = 0
    for r_true, r_false in paired:
        if r_true["landing_count"] == r_false["landing_count"] and r_true["alt_landing_count"] > 0:
            alt_save += 1
        if r_true["landing_count"] > r_false["landing_count"]:
            land_gain += 1

    print(f"  配对数量: {len(paired)}")
    print(f"  主落区不变、备降区挽救运行数: {alt_save} ({alt_save/len(paired)*100:.1f}%)")
    print(f"  enable=true 时主落区更多的运行数: {land_gain} ({land_gain/len(paired)*100:.1f}%)")


def print_env_complexity_analysis(rows: List[Dict]) -> None:
    """分析传感器参数（环境复杂度）对算法的影响"""
    print(f"\n{'='*70}")
    print("环境复杂度分析（polar_res × sensing_horizon 组合）")
    print(f"{'='*70}")

    grouped: Dict[tuple, List] = defaultdict(list)
    for r in rows:
        grouped[(r["polar_res"], r["sensing_horizon"])].append(r)

    for (pr, sh), subset in sorted(grouped.items()):
        primary_ok = sum(1 for r in subset if r["landing_count"] > 0)
        alt_found  = sum(1 for r in subset if r["landing_count"] == 0 and r["alt_landing_count"] > 0)
        all_fail   = sum(1 for r in subset if r["landing_count"] == 0 and r["alt_landing_count"] == 0)
        mean_land  = sum(r["landing_count"] for r in subset) / len(subset) if subset else 0
        valid_vals = [r["valid_count"] for r in subset if r["valid_count"] > 0]
        mean_valid = sum(valid_vals) / len(valid_vals) if valid_vals else 0
        print(
            f"  polar_res={pr} sensing_horizon={sh} | "
            f"runs={len(subset):>3} | "
            f"primary_ok={primary_ok}({primary_ok/len(subset)*100:.0f}%) "
            f"alt_found={alt_found} all_fail={all_fail} | "
            f"mean_land={mean_land:.1f} mean_valid={mean_valid:.0f}"
        )


# ===========================================================================
# 主程序
# ===========================================================================

def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(
        description="summarize_round4.py — 第4轮批量评估结果汇总")
    parser.add_argument("csv", help="round4_results.csv 路径")
    parser.add_argument("--top", type=int, default=15, help="显示 Top-N 组合（默认15）")
    args = parser.parse_args()

    rows = load_rows(args.csv)
    if not rows:
        print(f"[ERROR] No data loaded from {args.csv}", file=sys.stderr)
        return 1

    print(f"\n[round4] 总共加载 {len(rows)} 条记录（去重后）")

    # ---- 各维度分析 ----
    print_summary("分辨率影响 (grid_resolution)",
                  summarize_dimension(rows, "grid_resolution"))
    print_summary("降采样大小影响 (voxel_leaf_size)",
                  summarize_dimension(rows, "voxel_leaf_size"))
    print_summary("备降区放宽系数 (alt_relax_factor)",
                  summarize_dimension(rows, "alt_relax_factor"))
    print_summary("备降区启用状态 (enable_alt_landing)",
                  summarize_dimension(rows, "enable_alt_landing"))
    print_summary("雷达角分辨率 (polar_res)",
                  summarize_dimension(rows, "polar_res"))
    print_summary("传感视距 (sensing_horizon)",
                  summarize_dimension(rows, "sensing_horizon"))

    # ---- 专项分析 ----
    print_env_complexity_analysis(rows)
    print_alt_comparison(rows)

    # ---- Top 组合 ----
    print_top_combinations(rows, top_n=args.top)

    return 0


if __name__ == "__main__":
    sys.exit(main())
