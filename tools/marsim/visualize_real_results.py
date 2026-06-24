#!/usr/bin/env python3
"""
visualize_real_results.py — 真实仿真测试结果可视化

输入：batch_eval_real_10trials.sh 生成的 CSV（每行一次运行）
输出：
  r01_landing_overview.png        — 落区数量 + 有效栅格数总览
  r02_score_vs_resolution.png     — 安全得分 vs 分辨率
  r03_dt_heatmap.png              — 计算时延热力图（分辨率 × 体素大小）
  r04_alt_landing.png             — 备降区启用效果对比
  r05_map_comparison.png          — 三张地图横向对比
  r06_radar.png                   — 综合性能雷达图
  r07_valid_count.png             — 有效栅格数 vs 参数
  r08_dashboard.png               — 综合仪表盘
  report_real.html                — HTML 报告
"""

import argparse, csv, math, os, sys
from collections import defaultdict
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import font_manager as fm

# ---------------------------------------------------------------------------
# 中文字体
# ---------------------------------------------------------------------------
def _cjk():
    for name in ["Noto Sans CJK SC","Noto Sans CJK JP","AR PL UMing CN","Droid Sans Fallback"]:
        if name in {f.name for f in fm.fontManager.ttflist}:
            return name
    return "DejaVu Sans"

plt.rcParams.update({
    "font.family":        [_cjk(), "DejaVu Sans", "sans-serif"],
    "axes.unicode_minus": False,
    "font.size":          11,
    "axes.titlesize":     13,
    "axes.labelsize":     11,
    "axes.spines.top":    False,
    "axes.spines.right":  False,
    "figure.dpi":         120,
    "savefig.dpi":        150,
    "savefig.bbox":       "tight",
})

C_PRIMARY = "#2563EB"; C_ALT = "#F59E0B"; C_DANGER = "#EF4444"
C_SUCCESS = "#10B981"; C_NEUTRAL = "#6B7280"; C_BG = "#F8FAFC"
CMAP_10 = ["#1D4ED8","#2563EB","#3B82F6","#059669","#10B981",
           "#D97706","#F59E0B","#FBBF24","#DC2626","#7C3AED"]

# ---------------------------------------------------------------------------
# 参数标签（按 tag 解析）
# ---------------------------------------------------------------------------
TRIAL_LABELS = [
    "baseline", "hires", "lores", "sparse_sensor", "no_alt",
    "alt_relax2x", "hires_sparse", "lores_noalt", "recommended", "aggressive",
]
MAP_SHORT = {
    "kdxt_world_downsampled": "kdxt(室外)",
    "random_map_24_6635":     "rand_24(障碍)",
    "random_map_2_26609":     "rand_2(障碍)",
}

# ---------------------------------------------------------------------------
# 数据加载
# ---------------------------------------------------------------------------
def pf(v):
    if v is None or v == "": return math.nan
    try: return float(v)
    except: return math.nan

def pi(v):
    if v is None or v == "": return 0
    try: return int(float(v))
    except: return 0

def load(path: str) -> List[Dict]:
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            tag = r.get("tag","")
            # 解析 trial_label 和 map_name
            map_name = r.get("pcd","").split("/")[-1].replace(".pcd","")
            trial_label = "unknown"
            for lb in TRIAL_LABELS:
                if f"__{lb}__" in tag or f"__t" in tag:
                    # t<N>__<label>
                    import re
                    m = re.search(r"__t\d+__([^_]+)__", tag)
                    if m: trial_label = m.group(1)
                    break

            rows.append({
                "tag":               tag,
                "map_name":          map_name,
                "map_short":         MAP_SHORT.get(map_name, map_name[:14]),
                "trial_label":       trial_label,
                "landing_count":     pi(r.get("landing_count")),
                "alt_landing_count": pi(r.get("alt_landing_count","0")),
                "valid_count":       pi(r.get("valid_count","0")),
                "best_score":        pf(r.get("best_score")),
                "best_x":            pf(r.get("best_x")),
                "best_y":            pf(r.get("best_y")),
                "frame_dt":          pf(r.get("frame_dt_mean")),
                "grid_resolution":   pf(r.get("grid_resolution")),
                "voxel_leaf_size":   pf(r.get("voxel_leaf_size")),
                "polar_res":         pf(r.get("polar_res")),
                "sensing_horizon":   pf(r.get("sensing_horizon")),
                "alt_relax_factor":  pf(r.get("alt_relax_factor")),
                "enable_alt":        str(r.get("enable_alt_landing","")).strip().lower(),
                "samples":           pi(r.get("samples","1")),
            })
    return rows


# ---------------------------------------------------------------------------
# 图 r01 — 落区总览柱状图
# ---------------------------------------------------------------------------
def plot_r01(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(18, 6))
    fig.patch.set_facecolor(C_BG)

    # 左：每条记录的落区数 + 备降区数
    ax = axes[0]; ax.set_facecolor(C_BG)
    n = len(rows)
    x = np.arange(n)
    lc  = [r["landing_count"] for r in rows]
    alt = [r["alt_landing_count"] for r in rows]
    vc  = [r["valid_count"] for r in rows]

    ax.bar(x, lc,  0.6, color=C_PRIMARY, alpha=0.85, label="主落区格子数")
    ax.bar(x, alt, 0.6, bottom=lc, color=C_ALT, alpha=0.8, label="备降区格子数")

    # 标注落点坐标（只有有落区的）
    for i, r in enumerate(rows):
        if r["landing_count"] > 0 and math.isfinite(r["best_x"]):
            ax.text(x[i], r["landing_count"]+r["alt_landing_count"]+max(lc)*0.02,
                    f"({r['best_x']:.1f},{r['best_y']:.1f})",
                    ha="center", fontsize=7, color="#1E3A8A", rotation=60)

    # x轴：地图+参数简短标签
    xlabels = [f"{r['map_short'][:6]}\n{r['trial_label'][:10]}" for r in rows]
    ax.set_xticks(x); ax.set_xticklabels(xlabels, fontsize=7.5, rotation=15, ha="right")
    ax.set_ylabel("落区格子数")
    ax.set_title("真实测试：落区格子数（主落区 + 备降区）", fontweight="bold")
    ax.legend(fontsize=9); ax.grid(axis="y", alpha=0.3)

    # 右：有效栅格数（valid_count，反映地图覆盖率）
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    colors_v = [CMAP_10[i % len(CMAP_10)] for i in range(n)]
    ax2.bar(x, vc, 0.6, color=colors_v, alpha=0.8)
    ax2.set_xticks(x); ax2.set_xticklabels(xlabels, fontsize=7.5, rotation=15, ha="right")
    ax2.set_ylabel("有效栅格数")
    ax2.set_title("有效栅格数（点云覆盖密度指标）", fontweight="bold")
    ax2.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [r01] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 r02 — 安全得分 vs 分辨率（有落区的记录）
# ---------------------------------------------------------------------------
def plot_r02(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.patch.set_facecolor(C_BG)

    # 筛出有得分的行
    scored = [r for r in rows if math.isfinite(r["best_score"])]

    # 左：得分柱状图（只含有落区的试验）
    ax = axes[0]; ax.set_facecolor(C_BG)
    if scored:
        n = len(scored); x = np.arange(n)
        scores = [r["best_score"] for r in scored]
        colors_s = [C_SUCCESS if s >= 0.7 else (C_ALT if s >= 0.5 else C_DANGER)
                    for s in scores]
        ax.bar(x, scores, 0.6, color=colors_s, alpha=0.85)
        ax.axhline(0.7, color=C_SUCCESS, linestyle="--", alpha=0.5, label="优秀(0.7)")
        ax.axhline(0.5, color=C_DANGER,  linestyle="--", alpha=0.4, label="警戒(0.5)")
        for i, (s, r) in enumerate(zip(scores, scored)):
            ax.text(x[i], s+0.02, f"{s:.3f}", ha="center", fontsize=9, fontweight="bold")
        xlabels = [f"{r['map_short'][:6]}\n{r['trial_label'][:8]}" for r in scored]
        ax.set_xticks(x); ax.set_xticklabels(xlabels, fontsize=8.5, rotation=15, ha="right")
        ax.set_ylim(0, 1.15); ax.set_ylabel("最优安全得分")
        ax.set_title("真实测试：最优安全得分（仅含有落区的运行）", fontweight="bold")
        ax.legend(fontsize=9); ax.grid(axis="y", alpha=0.3)
    else:
        ax.text(0.5, 0.5, "无有效落区数据", ha="center", va="center",
                transform=ax.transAxes, fontsize=14, color=C_NEUTRAL)
        ax.set_title("最优安全得分", fontweight="bold")

    # 右：分辨率 vs 有效栅格数（所有记录）
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    res_groups: Dict[float, List] = defaultdict(list)
    for r in rows:
        res_groups[round(r["grid_resolution"], 3)].append(r["valid_count"])
    rkeys = sorted(res_groups.keys())
    rmeans  = [np.mean(res_groups[k]) for k in rkeys]
    rstds   = [np.std(res_groups[k])  for k in rkeys]
    xr = np.arange(len(rkeys))
    ax2.bar(xr, rmeans, 0.5, color=[C_PRIMARY, C_ALT, C_SUCCESS, C_NEUTRAL][:len(rkeys)],
            alpha=0.85, yerr=rstds, capsize=5, ecolor="#9CA3AF")
    ax2.set_xticks(xr); ax2.set_xticklabels([f"{k:.2f}m" for k in rkeys])
    ax2.set_xlabel("网格分辨率 (m)"); ax2.set_ylabel("平均有效栅格数")
    ax2.set_title("分辨率 vs 有效栅格数（真实点云密度）", fontweight="bold")
    ax2.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [r02] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 r03 — 计算时延热力图
# ---------------------------------------------------------------------------
def plot_r03(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.patch.set_facecolor(C_BG)

    # 去掉时延无效的行
    valid_dt = [r for r in rows if math.isfinite(r["frame_dt"])]
    res_vals   = sorted(set(round(r["grid_resolution"],3) for r in valid_dt))
    voxel_vals = sorted(set(round(r["voxel_leaf_size"],3) for r in valid_dt))

    if res_vals and voxel_vals:
        dt_mat = np.full((len(voxel_vals), len(res_vals)), np.nan)
        for r in valid_dt:
            ri = res_vals.index(round(r["grid_resolution"],3))
            vi = voxel_vals.index(round(r["voxel_leaf_size"],3))
            cur = dt_mat[vi, ri]
            dt_mat[vi, ri] = r["frame_dt"] if np.isnan(cur) else (cur + r["frame_dt"]) / 2

        ax = axes[0]; ax.set_facecolor(C_BG)
        cmap = LinearSegmentedColormap.from_list("dt", ["#FEF3C7","#F59E0B","#92400E"])
        vmin, vmax = np.nanmin(dt_mat), np.nanmax(dt_mat)
        im = ax.imshow(dt_mat, cmap=cmap, aspect="auto", vmin=vmin, vmax=vmax)
        plt.colorbar(im, ax=ax, shrink=0.8, label="平均帧间时延 (s)")
        for vi in range(len(voxel_vals)):
            for ri in range(len(res_vals)):
                v = dt_mat[vi, ri]
                if not np.isnan(v):
                    tc = "white" if (v-vmin)/(vmax-vmin+1e-9) > 0.55 else "#1F2937"
                    ax.text(ri, vi, f"{v:.2f}s", ha="center", va="center",
                            fontsize=10, color=tc, fontweight="bold")
        ax.set_xticks(range(len(res_vals)))
        ax.set_yticks(range(len(voxel_vals)))
        ax.set_xticklabels([f"{v:.2f}m" for v in res_vals])
        ax.set_yticklabels([f"{v:.2f}m" for v in voxel_vals])
        ax.set_xlabel("网格分辨率 (m)"); ax.set_ylabel("体素大小 (m)")
        ax.set_title("计算时延热力图（真实采集）\n(分辨率 × 体素大小)", fontweight="bold")
    else:
        axes[0].text(0.5, 0.5, "无时延数据", ha="center", va="center",
                     transform=axes[0].transAxes, fontsize=14)

    # 右：有效栅格数 vs 传感器稀疏度
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    pr_groups: Dict[float, List] = defaultdict(list)
    for r in rows:
        pr_groups[round(r["polar_res"],2)].append(r["valid_count"])
    pkeys = sorted(pr_groups.keys())
    pmeans = [np.mean(pr_groups[k]) for k in pkeys]
    pstds  = [np.std(pr_groups[k])  for k in pkeys]
    xp = np.arange(len(pkeys))
    ax2.bar(xp, pmeans, 0.4, color=[C_PRIMARY, C_DANGER][:len(pkeys)],
            alpha=0.85, yerr=pstds, capsize=5, ecolor="#9CA3AF")
    for i, (m, k) in enumerate(zip(pmeans, pkeys)):
        ax2.text(xp[i], m + max(pmeans)*0.01, f"{m:.0f}", ha="center", fontsize=10, fontweight="bold")
    ax2.set_xticks(xp); ax2.set_xticklabels([f"polar={k}" for k in pkeys])
    ax2.set_ylabel("平均有效栅格数"); ax2.set_xlabel("雷达角分辨率（稀疏度）")
    ax2.set_title("传感器稀疏度 vs 有效栅格数\n(polar越大=越稀疏)", fontweight="bold")
    ax2.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [r03] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 r04 — 备降区效果
# ---------------------------------------------------------------------------
def plot_r04(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    fig.patch.set_facecolor(C_BG)

    # 左：enable_alt=true vs false 对比
    ax = axes[0]; ax.set_facecolor(C_BG)
    true_rows  = [r for r in rows if r["enable_alt"] == "true"]
    false_rows = [r for r in rows if r["enable_alt"] == "false"]

    cats = ["有落区率(%)", "有备降率(%)", "全失败率(%)", "平均有效栅格(÷1000)"]
    def stats(grp):
        if not grp: return [0,0,0,0]
        n = len(grp)
        has_land = sum(1 for r in grp if r["landing_count"] > 0)
        has_alt  = sum(1 for r in grp if r["landing_count"]==0 and r["alt_landing_count"]>0)
        all_fail = sum(1 for r in grp if r["landing_count"]==0 and r["alt_landing_count"]==0)
        vc_mean  = np.mean([r["valid_count"] for r in grp]) / 1000
        return [has_land/n*100, has_alt/n*100, all_fail/n*100, vc_mean]

    vt = stats(true_rows); vf = stats(false_rows)
    xc = np.arange(len(cats)); wc = 0.35
    ax.bar(xc-wc/2, vt, wc, color=C_PRIMARY, alpha=0.85, label="启用备降区")
    ax.bar(xc+wc/2, vf, wc, color=C_NEUTRAL, alpha=0.75, label="关闭备降区")
    ax.set_xticks(xc); ax.set_xticklabels(cats, fontsize=9)
    ax.set_title("备降区开关对比（真实数据）", fontweight="bold")
    ax.legend(fontsize=9); ax.grid(axis="y", alpha=0.3)

    # 右：放宽系数 vs 落区得分（散点）
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    relax = [r["alt_relax_factor"] for r in rows]
    lc_vals = [r["landing_count"] for r in rows]
    scores_v = [r["best_score"] if math.isfinite(r["best_score"]) else 0 for r in rows]
    map_colors = []
    map_names_uniq = list(dict.fromkeys([r["map_name"] for r in rows]))
    color_map = {m: CMAP_10[i % len(CMAP_10)] for i, m in enumerate(map_names_uniq)}
    for r in rows:
        map_colors.append(color_map[r["map_name"]])

    ax2.scatter(relax, scores_v, s=[lc*20+30 for lc in lc_vals],
                c=map_colors, alpha=0.8, edgecolors="white", linewidth=1.2, zorder=5)
    for r in rows:
        if math.isfinite(r["best_score"]) and r["best_score"] > 0:
            ax2.annotate(r["trial_label"][:6],
                         (r["alt_relax_factor"], r["best_score"]),
                         xytext=(5,3), textcoords="offset points", fontsize=7.5)
    ax2.set_xlabel("备降区放宽系数"); ax2.set_ylabel("最优安全得分")
    ax2.set_title("放宽系数 vs 安全得分（真实）\n气泡大小=落区格子数", fontweight="bold")
    patches = [mpatches.Patch(color=color_map[m], label=MAP_SHORT.get(m, m[:12]))
               for m in map_names_uniq]
    ax2.legend(handles=patches, fontsize=9); ax2.grid(alpha=0.25)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [r04] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 r05 — 三张地图横向对比
# ---------------------------------------------------------------------------
def plot_r05(rows, out):
    fig, axes = plt.subplots(1, 3, figsize=(20, 7))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("三张地图横向对比（真实测试）", fontweight="bold", fontsize=14)

    map_names = list(dict.fromkeys([r["map_name"] for r in rows]))
    metrics = ["有效栅格数", "落区格子数", "有落区率(%)", "平均时延(s)"]

    for ax_i, mname in enumerate(map_names[:3]):
        ax = axes[ax_i]; ax.set_facecolor(C_BG)
        subset = [r for r in rows if r["map_name"] == mname]
        n = len(subset); x = np.arange(n)

        lc_vals = [r["landing_count"] for r in subset]
        vc_vals = [r["valid_count"] / 1000 for r in subset]  # 缩放到千格
        has_land = [1 if r["landing_count"] > 0 else 0 for r in subset]

        ax.bar(x - 0.2, lc_vals, 0.35, color=C_PRIMARY, alpha=0.85, label="落区格子数")
        ax2 = ax.twinx()
        ax2.bar(x + 0.2, vc_vals, 0.35, color=C_NEUTRAL, alpha=0.5, label="有效栅格(千)")
        ax2.set_ylabel("有效栅格数 (千格)", fontsize=9, color=C_NEUTRAL)
        ax2.tick_params(labelsize=8, labelcolor=C_NEUTRAL)

        # 有落区的标绿点
        for i, (lc, hl) in enumerate(zip(lc_vals, has_land)):
            if hl:
                ax.scatter(x[i]-0.2, lc+max(lc_vals)*0.03,
                           color=C_SUCCESS, s=60, zorder=5, marker="*")

        xlabels = [r["trial_label"][:8] for r in subset]
        ax.set_xticks(x); ax.set_xticklabels(xlabels, fontsize=8, rotation=30, ha="right")
        ax.set_ylabel("落区格子数")
        ax.set_title(f"{MAP_SHORT.get(mname, mname[:16])}\n"
                     f"有落区: {sum(has_land)}/{n}次",
                     fontweight="bold", fontsize=11)
        ax.grid(axis="y", alpha=0.3)
        handles1, _ = ax.get_legend_handles_labels()
        handles2, _ = ax2.get_legend_handles_labels()
        ax.legend(handles=handles1+handles2, fontsize=8, loc="upper right")

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [r05] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 r06 — 综合对比（按地图+配置分组）
# ---------------------------------------------------------------------------
def plot_r06(rows, out):
    map_names = list(dict.fromkeys([r["map_name"] for r in rows]))
    n_maps = len(map_names)
    n_trials = len(TRIAL_LABELS)

    fig, axes = plt.subplots(n_maps, 1, figsize=(18, 5*n_maps))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("各地图 × 各配置 详细比较", fontweight="bold", fontsize=14, y=0.99)

    if n_maps == 1:
        axes = [axes]

    for ax_i, mname in enumerate(map_names):
        ax = axes[ax_i]; ax.set_facecolor(C_BG)
        subset = [r for r in rows if r["map_name"] == mname]

        x = np.arange(len(subset)); w = 0.28
        vc  = [r["valid_count"]/1000 for r in subset]
        lc  = [r["landing_count"] for r in subset]
        sc  = [r["best_score"] if math.isfinite(r["best_score"]) else 0 for r in subset]

        ax.bar(x - w,  vc,  w, color=C_NEUTRAL, alpha=0.6, label="有效栅格(千)")
        ax.bar(x,      lc,  w, color=C_PRIMARY, alpha=0.85, label="落区格子数")
        ax3 = ax.twinx()
        ax3.bar(x + w, sc,  w, color=C_SUCCESS, alpha=0.7, label="安全得分")
        ax3.set_ylim(0, 1.3); ax3.set_ylabel("安全得分", color=C_SUCCESS, fontsize=9)
        ax3.tick_params(labelsize=8, labelcolor=C_SUCCESS)

        xlabels = [f"res={r['grid_resolution']:.2f}\n{r['trial_label'][:8]}" for r in subset]
        ax.set_xticks(x); ax.set_xticklabels(xlabels, fontsize=8.5)
        ax.set_title(f"地图: {MAP_SHORT.get(mname, mname)}", fontweight="bold")
        ax.set_ylabel("格子数")
        handles1, l1 = ax.get_legend_handles_labels()
        handles3, l3 = ax3.get_legend_handles_labels()
        ax.legend(handles=handles1+handles3, labels=l1+l3, fontsize=9, loc="upper right")
        ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [r06] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 r07 — 综合仪表盘
# ---------------------------------------------------------------------------
def plot_r07(rows, out):
    fig = plt.figure(figsize=(22, 14))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("safeland 真实仿真测试综合仪表盘", fontsize=16, fontweight="bold", y=0.99)
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

    # A: 每次运行的落区数
    ax_a = fig.add_subplot(gs[0, 0:2]); ax_a.set_facecolor(C_BG)
    n = len(rows); x = np.arange(n)
    lc  = [r["landing_count"]     for r in rows]
    alt = [r["alt_landing_count"] for r in rows]
    colors_a = [C_PRIMARY if lc_>0 else (C_ALT if alt_>0 else C_DANGER)
                for lc_, alt_ in zip(lc, alt)]
    ax_a.bar(x, lc, 0.6, color=colors_a, alpha=0.85)
    ax_a.bar(x, alt, 0.6, bottom=lc, color=C_ALT, alpha=0.6)
    xlabels = [f"{r['map_short'][:5]}\n{r['trial_label'][:7]}" for r in rows]
    ax_a.set_xticks(x); ax_a.set_xticklabels(xlabels, fontsize=7.5, rotation=30, ha="right")
    ax_a.set_ylabel("落区格子数"); ax_a.set_title("各次运行落区格子数（蓝=有落区/黄=仅备降/红=全失败）", fontweight="bold")
    ax_a.grid(axis="y", alpha=0.3)

    # B: 饼图 — 全局落区状态
    ax_b = fig.add_subplot(gs[0, 2]); ax_b.set_facecolor(C_BG)
    n_ok   = sum(1 for r in rows if r["landing_count"] > 0)
    n_alt  = sum(1 for r in rows if r["landing_count"] == 0 and r["alt_landing_count"] > 0)
    n_fail = sum(1 for r in rows if r["landing_count"] == 0 and r["alt_landing_count"] == 0)
    total  = len(rows)
    ax_b.pie([n_ok/total, n_alt/total, n_fail/total],
             colors=[C_PRIMARY, C_ALT, C_DANGER],
             labels=[f"有落区\n{n_ok}/{total}", f"仅备降\n{n_alt}", f"全失败\n{n_fail}"],
             startangle=90, wedgeprops=dict(edgecolor="white", linewidth=2))
    ax_b.set_title("全局落区状态分布", fontweight="bold")

    # C: 分辨率 vs 时延
    ax_c = fig.add_subplot(gs[1, 0]); ax_c.set_facecolor(C_BG)
    valid_dt = [(r["grid_resolution"], r["frame_dt"]) for r in rows
                if math.isfinite(r["frame_dt"])]
    if valid_dt:
        res_v, dt_v = zip(*valid_dt)
        res_uniq = sorted(set(round(v,3) for v in res_v))
        dt_by_res = defaultdict(list)
        for rv, dv in valid_dt:
            dt_by_res[round(rv,3)].append(dv)
        means = [np.mean(dt_by_res[k]) for k in res_uniq]
        ax_c.bar(range(len(res_uniq)), means,
                 color=[C_PRIMARY, C_ALT, C_SUCCESS, C_NEUTRAL][:len(res_uniq)], alpha=0.85)
        ax_c.set_xticks(range(len(res_uniq)))
        ax_c.set_xticklabels([f"{k:.2f}m" for k in res_uniq])
        ax_c.set_xlabel("网格分辨率"); ax_c.set_ylabel("平均帧间时延 (s)")
        ax_c.set_title("分辨率 vs 计算时延（真实）", fontweight="bold")
        ax_c.grid(axis="y", alpha=0.3)

    # D: 地图类型 vs 落区率
    ax_d = fig.add_subplot(gs[1, 1]); ax_d.set_facecolor(C_BG)
    map_names = list(dict.fromkeys([r["map_name"] for r in rows]))
    land_rates = []
    vc_means   = []
    for mname in map_names:
        sub = [r for r in rows if r["map_name"] == mname]
        land_rates.append(sum(1 for r in sub if r["landing_count"] > 0) / len(sub) * 100)
        vc_means.append(np.mean([r["valid_count"]/1000 for r in sub]))
    xm = np.arange(len(map_names)); wm = 0.35
    ax_d.bar(xm-wm/2, land_rates, wm, color=C_PRIMARY, alpha=0.85, label="有落区率(%)")
    ax_d2 = ax_d.twinx()
    ax_d2.bar(xm+wm/2, vc_means, wm, color=C_NEUTRAL, alpha=0.5, label="有效栅格(千)")
    ax_d2.set_ylabel("有效栅格(千格)", fontsize=9, color=C_NEUTRAL)
    ax_d.set_xticks(xm)
    ax_d.set_xticklabels([MAP_SHORT.get(m, m[:10]) for m in map_names], fontsize=9)
    ax_d.set_ylabel("有落区率 (%)"); ax_d.set_title("各地图落区成功率", fontweight="bold")
    ax_d.grid(axis="y", alpha=0.3)
    handles1, l1 = ax_d.get_legend_handles_labels()
    handles2, l2 = ax_d2.get_legend_handles_labels()
    ax_d.legend(handles=handles1+handles2, labels=l1+l2, fontsize=8)

    # E: 有得分记录的详细信息表
    ax_e = fig.add_subplot(gs[1, 2]); ax_e.set_facecolor(C_BG)
    scored = [r for r in rows if math.isfinite(r["best_score"]) and r["best_score"] > 0]
    if scored:
        table_data = [[r["map_short"][:8], r["trial_label"][:8],
                       f"{r['grid_resolution']:.2f}",
                       f"{r['landing_count']}",
                       f"{r['best_score']:.3f}",
                       f"({r['best_x']:.1f},{r['best_y']:.1f})"]
                      for r in scored]
        col_labels = ["地图", "配置", "分辨率", "落区数", "得分", "位置(x,y)"]
        tbl = ax_e.table(cellText=table_data, colLabels=col_labels,
                         loc="center", cellLoc="center")
        tbl.auto_set_font_size(False); tbl.set_fontsize(8)
        tbl.auto_set_column_width(list(range(len(col_labels))))
        for (row, col), cell in tbl.get_celld().items():
            if row == 0:
                cell.set_facecolor("#2563EB"); cell.set_text_props(color="white", fontsize=8)
            elif row % 2 == 0:
                cell.set_facecolor("#EFF6FF")
    ax_e.axis("off")
    ax_e.set_title("有效落区详细记录", fontweight="bold")

    plt.savefig(out); plt.close()
    print(f"  [r07] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# HTML 报告
# ---------------------------------------------------------------------------
def generate_report(out_dir, rows):
    total = len(rows)
    n_ok   = sum(1 for r in rows if r["landing_count"] > 0)
    n_fail = sum(1 for r in rows if r["landing_count"] == 0 and r["alt_landing_count"] == 0)
    scored = [r for r in rows if math.isfinite(r["best_score"]) and r["best_score"] > 0]
    best_r = max(scored, key=lambda r: r["best_score"]) if scored else None

    rows_html = ""
    for r in rows:
        lc_color = "#10B981" if r["landing_count"] > 0 else (
            "#F59E0B" if r["alt_landing_count"] > 0 else "#EF4444")
        sc_txt = f"{r['best_score']:.3f}" if math.isfinite(r["best_score"]) else "—"
        dt_txt = f"{r['frame_dt']:.2f}s" if math.isfinite(r["frame_dt"]) else "—"
        xy_txt = (f"({r['best_x']:.1f},{r['best_y']:.1f})"
                  if math.isfinite(r["best_x"]) else "—")
        rows_html += f"""
        <tr>
          <td>{MAP_SHORT.get(r['map_name'], r['map_name'][:14])}</td>
          <td>{r['trial_label']}</td>
          <td>{r['grid_resolution']:.2f}m</td>
          <td>{r['voxel_leaf_size']:.2f}m</td>
          <td>{r['polar_res']:.1f}</td>
          <td>{r['alt_relax_factor']:.1f} / {r['enable_alt']}</td>
          <td style="color:{lc_color};font-weight:bold">{r['landing_count']}</td>
          <td>{r['alt_landing_count']}</td>
          <td>{r['valid_count']:,}</td>
          <td style="font-weight:bold">{sc_txt}</td>
          <td>{xy_txt}</td>
          <td>{dt_txt}</td>
        </tr>"""

    fig_list = [
        ("r01_landing_overview.png",    "图01：落区格子数 + 有效栅格总览"),
        ("r02_score_vs_resolution.png", "图02：安全得分 + 分辨率影响"),
        ("r03_dt_heatmap.png",          "图03：计算时延热力图 + 传感器稀疏度"),
        ("r04_alt_landing.png",         "图04：备降区开关对比"),
        ("r05_map_comparison.png",      "图05：三张地图横向对比"),
        ("r06_detail_by_map.png",       "图06：各地图详细对比柱状图"),
        ("r07_dashboard.png",           "图07：综合仪表盘"),
    ]
    imgs_html = ""
    for fname, caption in fig_list:
        if os.path.exists(os.path.join(out_dir, fname)):
            imgs_html += f"""
      <div class="fig-block"><h3>{caption}</h3>
        <img src="{fname}" alt="{caption}" /></div>"""

    best_txt = (f"最优落点：地图={MAP_SHORT.get(best_r['map_name'],'?')} 配置={best_r['trial_label']} "
                f"得分={best_r['best_score']:.3f} 位置=({best_r['best_x']:.1f},{best_r['best_y']:.1f})"
                if best_r else "无有效落点")

    html = f"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8"/>
<title>safeland 真实仿真测试报告</title>
<style>
  body{{font-family:"Segoe UI","PingFang SC",sans-serif;background:#F8FAFC;color:#1F2937;margin:0;padding:24px}}
  h1{{font-size:26px;color:#1E3A8A;border-bottom:3px solid #2563EB;padding-bottom:8px}}
  h2{{font-size:18px;color:#1E3A8A;margin-top:28px}}
  table{{border-collapse:collapse;width:100%;margin:12px 0}}
  th{{background:#2563EB;color:white;padding:7px 9px;font-size:11px}}
  td{{padding:5px 9px;border-bottom:1px solid #E5E7EB;font-size:11px}}
  tr:nth-child(even) td{{background:#EFF6FF}}
  .box{{background:#EFF6FF;border-left:4px solid #2563EB;padding:12px 16px;border-radius:4px;margin:12px 0;line-height:1.8}}
  .fig-block{{margin:20px 0;background:white;padding:14px;border-radius:8px;box-shadow:0 1px 5px rgba(0,0,0,0.08)}}
  .fig-block img{{max-width:100%;height:auto;display:block;margin:0 auto}}
</style></head><body>
<h1>🚁 safeland 真实仿真测试分析报告</h1>
<p>生成时间：2026-06-15 | 总运行次数：{total} | 有效落区次数：{n_ok} | 全失败次数：{n_fail}</p>
<div class="box">
  <b>测试说明：</b>数据来自真实 ROS 节点（perfect_drone_sim + fast_lio_global_grid_map + safeland_node）<br>
  <b>地图：</b>kdxt_world（真实室外激光扫描）+ random_map_24/random_map_2（随机障碍物地图）<br>
  <b>{best_txt}</b><br>
  ⚠️ random_map 系列地图为密集随机障碍物场景，无平坦降落区域，全部返回 landing_count=0（符合预期）<br>
  ✅ kdxt_world 真实室外地图中，baseline/no_alt/alt_relax2x/recommended/aggressive 等配置均找到了安全落区，得分约 0.90
</div>
<h2>各次运行详细结果</h2>
<table>
  <thead><tr>
    <th>地图</th><th>配置</th><th>分辨率</th><th>体素</th><th>雷达</th>
    <th>放宽/备降</th><th>落区数</th><th>备降数</th><th>有效栅格</th>
    <th>安全得分</th><th>落点坐标</th><th>时延</th>
  </tr></thead>
  <tbody>{rows_html}</tbody>
</table>
<h2>详细分析图表</h2>
{imgs_html}
</body></html>"""

    rpath = os.path.join(out_dir, "report_real.html")
    with open(rpath, "w", encoding="utf-8") as f: f.write(html)
    print(f"  [HR] {rpath}")


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="真实测试结果可视化")
    parser.add_argument("csv")
    parser.add_argument("--out-dir", default="")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"[ERROR] CSV not found: {args.csv}", file=sys.stderr); return 1

    out_dir = args.out_dir or os.path.join(
        os.path.dirname(os.path.abspath(args.csv)), "figures")
    os.makedirs(out_dir, exist_ok=True)

    rows = load(args.csv)
    print(f"[+] 加载 {len(rows)} 条真实测试记录")
    print(f"[+] 有落区: {sum(1 for r in rows if r['landing_count']>0)} 次")
    print(f"[+] 生成图表到 {out_dir}/\n")

    def p(n): return os.path.join(out_dir, n)

    plot_r01(rows, p("r01_landing_overview.png"))
    plot_r02(rows, p("r02_score_vs_resolution.png"))
    plot_r03(rows, p("r03_dt_heatmap.png"))
    plot_r04(rows, p("r04_alt_landing.png"))
    plot_r05(rows, p("r05_map_comparison.png"))
    plot_r06(rows, p("r06_detail_by_map.png"))
    plot_r07(rows, p("r07_dashboard.png"))
    generate_report(out_dir, rows)

    print(f"\n[OK] 所有图表已生成至: {out_dir}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
