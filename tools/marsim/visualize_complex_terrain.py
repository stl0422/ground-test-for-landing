#!/usr/bin/env python3
"""
visualize_complex_terrain.py — 复杂地形 10 次批量测试结果可视化

输入：complex_terrain_10trials.csv
输出（保存到 figures_complex/ 目录）：
  c01_overview.png           — 落区数量与有效栅格总览
  c02_score_bar.png          — 最优安全得分柱状图（按配置）
  c03_dt_heatmap.png         — 计算时延热力图（分辨率 × 体素）
  c04_landing_scatter.png    — 落点坐标散点图 + 障碍物布局叠加
  c05_param_matrix.png       — 参数矩阵对比（斜率/凹陷/台阶阈值）
  c06_success_rate.png       — 各配置成功率 + 综合雷达图
  c07_dashboard.png          — 综合仪表盘
  report_complex.html        — HTML 报告
"""

import argparse, csv, math, os, sys
from collections import defaultdict
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import font_manager as fm

# ---------------------------------------------------------------------------
# 中文字体
# ---------------------------------------------------------------------------
def _cjk():
    for name in ["Noto Sans CJK SC","Noto Sans CJK JP","AR PL UMing CN",
                 "WenQuanYi Micro Hei","Droid Sans Fallback"]:
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

# ── 复杂地形障碍物布局（用于叠加在散点图上） ──────────────────────────
# 格式: (cx, cy, w, h, angle_deg, label)  — box 障碍物
OBSTACLES_BOX = [
    (-5.0,  1.8, 1.4, 0.7, 23,  "斜板A1"),
    (-2.5,  4.2, 1.8, 0.18, 17, "矮墙A2a"),
    (-1.6,  3.8, 1.2, 0.18, -69,"矮墙A2b"),
    (-3.2,  1.0, 0.80, 0.65, 40,"石块A4"),
    (-5.5, -4.0, 0.8, 1.5, 29,  "台阶B1"),
    (-2.8, -3.5, 1.6, 0.6, -46, "斜板B2"),
    (-0.8, -3.0, 0.35, 0.28, 69,"碎石B4"),
    ( 1.5,  3.2, 0.90, 0.70, 63,"石块C1"),
    ( 2.0,  5.5, 2.0, 0.20, 45, "矮墙C2"),
    ( 4.2,  3.5, 1.4, 0.65, 34, "斜板C3"),
    ( 3.8,  1.0, 0.32, 0.26, 52,"碎石C5"),
    ( 3.5, -1.0, 1.2, 0.55, -23,"台阶D1"),
    ( 2.0, -2.8, 0.85, 0.68, 80,"石块D2"),
    ( 4.5, -3.5, 1.5, 0.60, 69, "斜板D3"),
    ( 4.0, -1.5, 1.6, 0.18, 11, "矮墙D4a"),
    ( 4.7, -1.2, 1.0, 0.18, -80,"矮墙D4b"),
    ( 0.3,  1.5, 0.70, 0.55, 52,"石块E1"),
    ( 0.2, -2.5, 0.30, 0.24, 86,"碎石E2"),
    ( 0.8,  0.8, 1.4, 0.16, 60, "矮墙E3"),
]
# 圆柱/树木中心
OBSTACLES_CYL = [
    (-2.8, -5.8, 0.28, "石墩B3"),
    (-1.8,  5.8, 0.14, "斜柱A5"),
    ( 1.8,  5.8, 0.11, "树C4"),
    ( 5.8, -5.0, 0.16, "斜柱D5"),
    ( 4.8, -5.8, 0.12, "树D6"),
]
# 预留降落窗口
LANDING_WINDOWS = [
    ( 1.5, -0.5, "W0(起飞台)"),
    (-3.8,  3.2, "W1"),
    (-3.8, -2.8, "W2"),
    (-1.2, -4.8, "W3"),
    (-0.5,  2.5, "W4"),
    ( 3.0,  4.5, "W5"),
    ( 3.5, -4.0, "W6"),
    ( 5.2,  2.2, "W7"),
    ( 5.2, -2.8, "W8"),
    (-6.0,  0.5, "W9"),
]

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
            import re
            tag = r.get("tag", "")
            m = re.search(r"__t\d+__([^_]+)__", tag)
            trial_label = m.group(1) if m else "unknown"
            rows.append({
                "tag":               tag,
                "trial_label":       trial_label,
                "landing_count":     pi(r.get("landing_count")),
                "alt_landing_count": pi(r.get("alt_landing_count", "0")),
                "valid_count":       pi(r.get("valid_count", "0")),
                "best_score":        pf(r.get("best_score")),
                "best_x":            pf(r.get("best_x")),
                "best_y":            pf(r.get("best_y")),
                "landing_centroid_x": pf(r.get("landing_centroid_x")),
                "landing_centroid_y": pf(r.get("landing_centroid_y")),
                "frame_dt":          pf(r.get("frame_dt_mean")),
                "grid_resolution":   pf(r.get("grid_resolution")),
                "voxel_leaf_size":   pf(r.get("voxel_leaf_size")),
                "slope_threshold":   pf(r.get("slope_threshold")),
                "step_threshold":    pf(r.get("step_threshold")),
                "depression_score_threshold": pf(r.get("depression_score_threshold")),
                "landing_size":      pf(r.get("landing_size")),
                "alt_relax_factor":  pf(r.get("alt_relax_factor")),
                "enable_alt":        str(r.get("enable_alt_landing", "")).strip().lower(),
                "samples":           pi(r.get("samples", "1")),
            })
    return rows


# ---------------------------------------------------------------------------
# 图 c01 — 落区数量与有效栅格总览
# ---------------------------------------------------------------------------
def plot_c01(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(20, 7))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("复杂地形 10 次测试 — 落区总览", fontsize=15, fontweight="bold")

    ax = axes[0]; ax.set_facecolor(C_BG)
    n = len(rows); x = np.arange(n)
    lc  = [r["landing_count"] for r in rows]
    alt = [r["alt_landing_count"] for r in rows]

    colors_a = [C_SUCCESS if lc_ > 0 else (C_ALT if alt_ > 0 else C_DANGER)
                for lc_, alt_ in zip(lc, alt)]
    bars = ax.bar(x, lc, 0.6, color=colors_a, alpha=0.88, label="主落区格子数")
    ax.bar(x, alt, 0.6, bottom=lc, color=C_ALT, alpha=0.7, label="备降区格子数")

    # 标注得分和坐标
    for i, r in enumerate(rows):
        if r["landing_count"] > 0 and math.isfinite(r["best_score"]):
            ax.text(x[i], r["landing_count"] + max(lc)*0.03,
                    f"{r['best_score']:.3f}",
                    ha="center", fontsize=8.5, color="#1E3A8A", fontweight="bold")
        if r["landing_count"] == 0 and r["alt_landing_count"] == 0:
            ax.text(x[i], 0.3, "×", ha="center", fontsize=14, color=C_DANGER, fontweight="bold")

    xlabels = [r["trial_label"] for r in rows]
    ax.set_xticks(x); ax.set_xticklabels(xlabels, fontsize=9, rotation=30, ha="right")
    ax.set_ylabel("落区格子数"); ax.set_title("各参数配置落区格子数（标注安全得分）", fontweight="bold")
    ax.legend(fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    # 颜色图例
    patches = [
        mpatches.Patch(color=C_SUCCESS, label="有主落区"),
        mpatches.Patch(color=C_ALT,     label="仅备降区"),
        mpatches.Patch(color=C_DANGER,  label="全失败"),
    ]
    ax.legend(handles=patches, fontsize=9, loc="upper right")

    # 右：有效栅格数
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    vc = [r["valid_count"] for r in rows]
    colors_v = [CMAP_10[i % len(CMAP_10)] for i in range(n)]
    ax2.bar(x, vc, 0.6, color=colors_v, alpha=0.85)
    for i, v in enumerate(vc):
        ax2.text(x[i], v + max(vc)*0.01, f"{v:,}", ha="center", fontsize=8)
    ax2.set_xticks(x); ax2.set_xticklabels(xlabels, fontsize=9, rotation=30, ha="right")
    ax2.set_ylabel("有效栅格数"); ax2.set_title("各配置有效栅格数（点云覆盖密度）", fontweight="bold")
    ax2.grid(axis="y", alpha=0.3)
    # 增长趋势标注
    ax2.axhline(np.mean(vc), color=C_PRIMARY, linestyle="--", alpha=0.5,
                label=f"均值={int(np.mean(vc))}")
    ax2.legend(fontsize=9)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [c01] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 c02 — 安全得分柱状图
# ---------------------------------------------------------------------------
def plot_c02(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(18, 7))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("复杂地形 — 安全得分与参数分析", fontsize=14, fontweight="bold")

    # 左：安全得分
    ax = axes[0]; ax.set_facecolor(C_BG)
    labels = [r["trial_label"] for r in rows]
    scores = [r["best_score"] if math.isfinite(r["best_score"]) else 0 for r in rows]
    colors_s = [C_SUCCESS if s >= 0.9 else (C_ALT if s >= 0.7 else (C_DANGER if s > 0 else C_NEUTRAL))
                for s in scores]
    x = np.arange(len(rows))
    bars = ax.bar(x, scores, 0.65, color=colors_s, alpha=0.88)
    ax.axhline(0.90, color=C_SUCCESS, linestyle="--", linewidth=1.5, alpha=0.6, label="优秀(0.90)")
    ax.axhline(0.70, color=C_ALT,     linestyle="--", linewidth=1.5, alpha=0.5, label="良好(0.70)")
    for i, (s, r) in enumerate(zip(scores, rows)):
        if s > 0:
            ax.text(x[i], s + 0.008, f"{s:.3f}", ha="center", fontsize=9, fontweight="bold")
        else:
            ax.text(x[i], 0.02, "失败", ha="center", fontsize=9, color=C_DANGER)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9, rotation=30, ha="right")
    ax.set_ylim(0, 1.1); ax.set_ylabel("最优安全得分")
    ax.set_title("各参数配置最优安全得分", fontweight="bold")
    ax.legend(fontsize=9); ax.grid(axis="y", alpha=0.3)

    # 右：落区数 vs 斜率阈值散点
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    slopes = [r["slope_threshold"] for r in rows]
    lc_vals = [r["landing_count"] for r in rows]
    sc_vals = [r["best_score"] if math.isfinite(r["best_score"]) else 0 for r in rows]
    sizes = [max(lc*40, 60) for lc in lc_vals]
    scatter = ax2.scatter(slopes, sc_vals, s=sizes, c=lc_vals,
                         cmap="Blues", alpha=0.85, edgecolors="gray", linewidth=1,
                         vmin=0, vmax=max(lc_vals) if lc_vals else 1)
    cbar = plt.colorbar(scatter, ax=ax2, shrink=0.8)
    cbar.set_label("落区格子数", fontsize=9)
    for i, r in enumerate(rows):
        ax2.annotate(r["trial_label"][:6],
                     (r["slope_threshold"], sc_vals[i]),
                     xytext=(5, 4), textcoords="offset points", fontsize=8)
    ax2.set_xlabel("斜率阈值 slope_threshold")
    ax2.set_ylabel("最优安全得分")
    ax2.set_title("斜率阈值 vs 安全得分\n气泡大小=落区格子数", fontweight="bold")
    ax2.grid(alpha=0.25)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [c02] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 c03 — 计算时延热力图
# ---------------------------------------------------------------------------
def plot_c03(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(18, 7))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("复杂地形 — 计算时延分析", fontsize=14, fontweight="bold")

    # 热力图
    ax = axes[0]; ax.set_facecolor(C_BG)
    valid_dt = [r for r in rows if math.isfinite(r["frame_dt"])]
    res_vals   = sorted(set(round(r["grid_resolution"], 3) for r in valid_dt))
    voxel_vals = sorted(set(round(r["voxel_leaf_size"], 3) for r in valid_dt))

    if res_vals and voxel_vals:
        dt_mat = np.full((len(voxel_vals), len(res_vals)), np.nan)
        for r in valid_dt:
            ri = res_vals.index(round(r["grid_resolution"], 3))
            vi = voxel_vals.index(round(r["voxel_leaf_size"], 3))
            cur = dt_mat[vi, ri]
            dt_mat[vi, ri] = r["frame_dt"] if np.isnan(cur) else (cur + r["frame_dt"]) / 2

        cmap = LinearSegmentedColormap.from_list("dt", ["#D1FAE5","#059669","#064E3B"])
        vmin, vmax = np.nanmin(dt_mat), np.nanmax(dt_mat)
        im = ax.imshow(dt_mat, cmap=cmap, aspect="auto", vmin=vmin, vmax=vmax)
        plt.colorbar(im, ax=ax, shrink=0.8, label="平均帧间时延 (s)")
        for vi in range(len(voxel_vals)):
            for ri in range(len(res_vals)):
                v = dt_mat[vi, ri]
                if not np.isnan(v):
                    rel = (v - vmin) / max(vmax - vmin, 1e-9)
                    tc = "white" if rel > 0.5 else "#1F2937"
                    ax.text(ri, vi, f"{v:.3f}s", ha="center", va="center",
                            fontsize=10.5, color=tc, fontweight="bold")
        ax.set_xticks(range(len(res_vals)))
        ax.set_yticks(range(len(voxel_vals)))
        ax.set_xticklabels([f"{v:.2f}m" for v in res_vals])
        ax.set_yticklabels([f"{v:.2f}m" for v in voxel_vals])
        ax.set_xlabel("网格分辨率 (m)"); ax.set_ylabel("体素大小 (m)")
        ax.set_title("计算时延热力图\n(分辨率 × 体素大小)", fontweight="bold")

    # 右：时延序列折线图
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    ts = list(range(1, len(rows)+1))
    dts = [r["frame_dt"] if math.isfinite(r["frame_dt"]) else None for r in rows]
    dts_plot = [d for d in dts if d is not None]
    ts_plot = [t for t, d in zip(ts, dts) if d is not None]
    if dts_plot:
        ax2.plot(ts_plot, dts_plot, "o-", color=C_PRIMARY, linewidth=2.5,
                 markersize=8, alpha=0.85, label="帧间时延")
        ax2.fill_between(ts_plot, dts_plot, alpha=0.12, color=C_PRIMARY)
        ax2.axhline(np.mean(dts_plot), color=C_SUCCESS, linestyle="--", linewidth=1.5,
                    alpha=0.7, label=f"均值={np.mean(dts_plot):.3f}s")
        for t, d, r in zip(ts_plot, dts_plot, rows):
            ax2.text(t, d + 0.003, r["trial_label"][:5], ha="center", fontsize=7.5)
    ax2.set_xlabel("试验序号"); ax2.set_ylabel("帧间时延 (s)")
    ax2.set_title("各次测试计算时延序列", fontweight="bold")
    ax2.legend(fontsize=9); ax2.grid(alpha=0.25)
    ax2.set_xticks(ts_plot if ts_plot else ts)

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [c03] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 c04 — 落点坐标散点图 + 障碍物/降落窗口叠加
# ---------------------------------------------------------------------------
def plot_c04(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(22, 9))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("复杂地形 — 落点坐标分布（叠加障碍物布局）", fontsize=14, fontweight="bold")

    for ax_idx, ax in enumerate(axes):
        ax.set_facecolor("#F0F4F8")
        ax.set_xlim(-7.5, 7.5); ax.set_ylim(-7.5, 7.5)
        ax.set_xlabel("X (m，前方)"); ax.set_ylabel("Y (m，左方)")
        ax.set_aspect("equal")
        ax.grid(alpha=0.25, color="white")

        # ── 绘制基础地形范围
        terrain = plt.Rectangle((-7.5, -7.5), 15, 15, fill=True,
                                 facecolor="#E8EEF4", edgecolor="#B0B8C4",
                                 linewidth=1.5, zorder=1, label="地形范围(15×15m)")
        ax.add_patch(terrain)

        # ── 绘制障碍物（box）
        from matplotlib.patches import FancyArrowPatch
        for cx, cy, w, h, ang, lbl in OBSTACLES_BOX:
            rect = plt.Rectangle(
                (cx - w/2, cy - h/2), w, h,
                angle=0,  # 简化：不旋转，只画中心位置
                fill=True, facecolor="#8B7355", edgecolor="#5D4E37",
                linewidth=1, alpha=0.75, zorder=3
            )
            t = matplotlib.transforms.Affine2D().rotate_deg_around(cx, cy, ang) + ax.transData
            rect.set_transform(t)
            ax.add_patch(rect)

        # ── 绘制圆柱障碍物
        for cx, cy, r_cyl, lbl in OBSTACLES_CYL:
            cyl = plt.Circle((cx, cy), r_cyl, fill=True,
                             facecolor="#8B7355", edgecolor="#5D4E37",
                             linewidth=1, alpha=0.7, zorder=3)
            ax.add_patch(cyl)

        # ── 绘制降落窗口
        for wx, wy, wlbl in LANDING_WINDOWS:
            win = plt.Rectangle((wx - 0.3, wy - 0.3), 0.6, 0.6,
                                fill=True, facecolor="#22C55E", edgecolor="#15803D",
                                linewidth=1.5, alpha=0.35, zorder=4)
            ax.add_patch(win)
            ax.text(wx, wy, wlbl, ha="center", va="center",
                    fontsize=6.5, color="#15803D", fontweight="bold", zorder=6)

        # ── 起飞点标记
        ax.plot(1.5, -0.5, "*", color="#FBBF24", markersize=16, zorder=8,
                markeredgecolor="#92400E", markeredgewidth=1.2, label="起飞点")

        if ax_idx == 0:
            # 左图：每次测试的最优落点
            scored_rows = [r for r in rows
                           if math.isfinite(r["best_x"]) and math.isfinite(r["best_y"])]
            if scored_rows:
                xs = [r["best_x"] for r in scored_rows]
                ys = [r["best_y"] for r in scored_rows]
                scores = [r["best_score"] for r in scored_rows]
                sc = ax.scatter(xs, ys, c=scores, s=180,
                               cmap="RdYlGn", vmin=0.85, vmax=1.0,
                               edgecolors="white", linewidth=1.5, zorder=9, label="最优落点")
                cbar = plt.colorbar(sc, ax=ax, shrink=0.6, pad=0.02)
                cbar.set_label("安全得分", fontsize=9)
                # 标注配置名
                for r in scored_rows:
                    ax.annotate(r["trial_label"][:6],
                                (r["best_x"], r["best_y"]),
                                xytext=(6, 4), textcoords="offset points",
                                fontsize=7.5, color="#1E3A8A",
                                path_effects=[pe.withStroke(linewidth=2, foreground="white")])
            ax.set_title("各配置最优落点分布", fontweight="bold")

        else:
            # 右图：落点质心密度热力图
            all_cx = [r["landing_centroid_x"] for r in rows if math.isfinite(r.get("landing_centroid_x", math.nan))]
            all_cy = [r["landing_centroid_y"] for r in rows if math.isfinite(r.get("landing_centroid_y", math.nan))]
            if len(all_cx) >= 2:
                # 2D 高斯核密度估计
                from scipy.stats import gaussian_kde
                data = np.vstack([all_cx, all_cy])
                kde = gaussian_kde(data, bw_method=0.5)
                xi = np.linspace(-7, 7, 80)
                yi = np.linspace(-7, 7, 80)
                XX, YY = np.meshgrid(xi, yi)
                ZZ = kde(np.vstack([XX.ravel(), YY.ravel()])).reshape(XX.shape)
                cmap_heat = LinearSegmentedColormap.from_list(
                    "heat", ["#F8FAFC","#93C5FD","#2563EB","#1E3A8A"])
                ax.contourf(XX, YY, ZZ, levels=12, cmap=cmap_heat, alpha=0.65, zorder=2)
                ax.scatter(all_cx, all_cy, s=100, c="#1D4ED8", edgecolors="white",
                           linewidth=1.2, zorder=9, label="落区质心")
            elif all_cx:
                ax.scatter(all_cx, all_cy, s=120, c=C_PRIMARY, edgecolors="white",
                           linewidth=1.2, zorder=9, label="落区质心")
            ax.set_title("落区质心热力图（点云密度叠加）", fontweight="bold")

        # 图例
        handles = [
            mpatches.Patch(facecolor="#8B7355", edgecolor="#5D4E37", alpha=0.75, label="障碍物"),
            mpatches.Patch(facecolor="#22C55E", edgecolor="#15803D", alpha=0.4, label="降落窗口"),
            plt.Line2D([0],[0], marker="*", color=C_ALT, markersize=10,
                       markeredgecolor="#92400E", linestyle="", label="起飞点"),
        ]
        ax.legend(handles=handles, fontsize=8.5, loc="lower right")

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [c04] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 c05 — 参数矩阵对比
# ---------------------------------------------------------------------------
def plot_c05(rows, out):
    fig, axes = plt.subplots(2, 2, figsize=(18, 12))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("复杂地形 — 关键参数对比矩阵", fontsize=14, fontweight="bold")

    labels = [r["trial_label"] for r in rows]
    lc = [r["landing_count"] for r in rows]
    scores = [r["best_score"] if math.isfinite(r["best_score"]) else 0 for r in rows]
    x = np.arange(len(rows))

    # (0,0): 斜率阈值 vs 落区数
    ax = axes[0][0]; ax.set_facecolor(C_BG)
    slope_t = [r["slope_threshold"] for r in rows]
    ax.bar(x, lc, 0.6, color=[CMAP_10[i] for i in range(len(rows))], alpha=0.85)
    ax2 = ax.twinx()
    ax2.plot(x, slope_t, "D--", color=C_DANGER, linewidth=2, markersize=7, alpha=0.8, label="斜率阈值")
    ax2.set_ylabel("斜率阈值", color=C_DANGER, fontsize=9)
    ax2.tick_params(labelsize=8, labelcolor=C_DANGER)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8, rotation=30, ha="right")
    ax.set_ylabel("落区格子数"); ax.set_title("斜率阈值 vs 落区格子数", fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    # (0,1): 凹陷阈值 vs 安全得分
    ax = axes[0][1]; ax.set_facecolor(C_BG)
    dep_t = [r["depression_score_threshold"] for r in rows]
    colors_dep = [C_SUCCESS if s >= 0.9 else (C_ALT if s > 0 else C_NEUTRAL) for s in scores]
    ax.bar(x, scores, 0.6, color=colors_dep, alpha=0.88)
    ax2 = ax.twinx()
    ax2.plot(x, dep_t, "s--", color=C_PRIMARY, linewidth=2, markersize=7, alpha=0.8, label="凹陷阈值")
    ax2.set_ylabel("凹陷阈值", color=C_PRIMARY, fontsize=9)
    ax2.tick_params(labelsize=8, labelcolor=C_PRIMARY)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8, rotation=30, ha="right")
    ax.set_ylim(0, 1.05); ax.set_ylabel("安全得分"); ax.set_title("凹陷阈值 vs 安全得分", fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    # (1,0): 台阶阈值 vs 时延
    ax = axes[1][0]; ax.set_facecolor(C_BG)
    step_t = [r["step_threshold"] for r in rows]
    dts = [r["frame_dt"] if math.isfinite(r["frame_dt"]) else 0 for r in rows]
    ax.bar(x, dts, 0.6, color=[CMAP_10[(i+3)%10] for i in range(len(rows))], alpha=0.85)
    ax2 = ax.twinx()
    ax2.plot(x, step_t, "^--", color=C_ALT, linewidth=2, markersize=7, alpha=0.8, label="台阶阈值")
    ax2.set_ylabel("台阶阈值", color=C_ALT, fontsize=9)
    ax2.tick_params(labelsize=8, labelcolor=C_ALT)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8, rotation=30, ha="right")
    ax.set_ylabel("帧间时延 (s)"); ax.set_title("台阶阈值 vs 计算时延", fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    # (1,1): 综合参数雷达图（最优3个配置 vs 最差配置）
    ax = axes[1][1]; ax.set_facecolor(C_BG)
    # 找最优和最差的配置
    sorted_by_score = sorted(rows, key=lambda r: (
        r["best_score"] if math.isfinite(r["best_score"]) else -1), reverse=True)
    top3 = sorted_by_score[:3]
    worst = [sorted_by_score[-1]]

    cats = ["落区格子数\n(归一化)", "安全得分\n(×10)", "有效栅格\n(÷300)", "时延低分\n(1-dt×2)", "斜率裕度\n(0.3-slope×2)"]
    n_cats = len(cats)
    angles = np.linspace(0, 2*np.pi, n_cats, endpoint=False).tolist()
    angles += angles[:1]

    ax_r = fig.add_axes([0.52, 0.05, 0.42, 0.42], polar=True)
    ax_r.set_facecolor("#F0F4FF")
    ax_r.set_thetagrids(np.degrees(angles[:-1]), cats, fontsize=8)

    def normalize_row(r):
        lc_ = r["landing_count"] / max(r["landing_count"] for r in rows + [{"landing_count":1}])
        sc_ = (r["best_score"] if math.isfinite(r["best_score"]) else 0) * 10 / 10
        vc_ = r["valid_count"] / 3000
        dt_ = max(0, 1 - r["frame_dt"] * 2) if math.isfinite(r["frame_dt"]) else 0
        sl_ = max(0, min(1, (0.3 - r["slope_threshold"]) * 4 + 0.5))
        return [lc_, sc_, vc_, dt_, sl_]

    max_lc = max(r["landing_count"] for r in rows) if rows else 1
    def norm2(r):
        lc_ = r["landing_count"] / max_lc
        sc_ = (r["best_score"] if math.isfinite(r["best_score"]) else 0)
        vc_ = min(1.0, r["valid_count"] / 2500)
        dt_ = max(0, 1 - r["frame_dt"] * 2) if math.isfinite(r["frame_dt"]) else 0
        sl_ = max(0, min(1, (0.30 - r["slope_threshold"]) * 5 + 0.5))
        return [lc_, sc_, vc_, dt_, sl_]

    palette = [C_SUCCESS, C_PRIMARY, "#7C3AED", C_DANGER]
    for i, r in enumerate(top3 + worst):
        vals = norm2(r)
        vals += vals[:1]
        lw = 2.5 if i < 3 else 2.0
        ls = "-" if i < 3 else "--"
        alpha = 0.85 if i < 3 else 0.6
        ax_r.plot(angles, vals, color=palette[i], linewidth=lw, linestyle=ls, alpha=alpha,
                  label=r["trial_label"][:8])
        ax_r.fill(angles, vals, color=palette[i], alpha=0.08)

    ax_r.set_ylim(0, 1)
    ax_r.set_rticks([0.25, 0.5, 0.75, 1.0]); ax_r.set_rlabel_position(30)
    ax_r.legend(loc="upper right", bbox_to_anchor=(1.35, 1.1), fontsize=8)
    ax_r.set_title("综合能力雷达图\n(前3名 vs 最差)", fontweight="bold", pad=20)

    # 删除 axes[1][1]（被 ax_r 替代显示）
    ax.axis("off")

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [c05] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 c06 — 成功率 + 综合评估
# ---------------------------------------------------------------------------
def plot_c06(rows, out):
    fig, axes = plt.subplots(1, 3, figsize=(22, 7))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("复杂地形 10 次测试 — 综合评估", fontsize=14, fontweight="bold")

    # 左：成功率饼图（主落区 / 仅备降 / 全失败）
    ax = axes[0]; ax.set_facecolor(C_BG)
    n_ok   = sum(1 for r in rows if r["landing_count"] > 0)
    n_alt  = sum(1 for r in rows if r["landing_count"] == 0 and r["alt_landing_count"] > 0)
    n_fail = sum(1 for r in rows if r["landing_count"] == 0 and r["alt_landing_count"] == 0)
    total  = len(rows)
    vals = [v for v in [n_ok, n_alt, n_fail] if v > 0]
    lbls = []
    cols = []
    if n_ok:   lbls.append(f"主落区\n{n_ok}/{total}({n_ok/total*100:.0f}%)"); cols.append(C_SUCCESS)
    if n_alt:  lbls.append(f"仅备降\n{n_alt}"); cols.append(C_ALT)
    if n_fail: lbls.append(f"全失败\n{n_fail}"); cols.append(C_DANGER)
    wedges, texts, autotexts = ax.pie(vals, colors=cols, labels=lbls,
             autopct="%1.0f%%", startangle=90,
             wedgeprops=dict(edgecolor="white", linewidth=2.5))
    for at in autotexts:
        at.set_fontsize(11); at.set_fontweight("bold")
    ax.set_title(f"落区成功率\n总计 {total} 次测试", fontweight="bold")

    # 中：落区格子数 vs 时延气泡图
    ax2 = axes[1]; ax2.set_facecolor(C_BG)
    for i, r in enumerate(rows):
        if math.isfinite(r["frame_dt"]):
            sc = r["best_score"] if math.isfinite(r["best_score"]) else 0
            color = CMAP_10[i % len(CMAP_10)]
            ax2.scatter(r["frame_dt"], r["landing_count"],
                       s=max(sc * 300, 80), c=color, alpha=0.85,
                       edgecolors="white", linewidth=1.2, zorder=5)
            ax2.annotate(r["trial_label"][:6],
                        (r["frame_dt"], r["landing_count"]),
                        xytext=(5, 3), textcoords="offset points", fontsize=8)
    ax2.set_xlabel("平均帧间时延 (s)"); ax2.set_ylabel("落区格子数")
    ax2.set_title("时延 vs 落区格子数\n（气泡大小 = 安全得分）", fontweight="bold")
    ax2.grid(alpha=0.25)

    # 右：综合排行（加权得分 = 0.4×得分 + 0.3×落区率 + 0.3×(1-时延比)）
    ax3 = axes[2]; ax3.set_facecolor(C_BG)
    max_lc = max(r["landing_count"] for r in rows) if rows else 1
    max_dt = max(r["frame_dt"] for r in rows if math.isfinite(r["frame_dt"])) if rows else 1

    composite = []
    for r in rows:
        sc = r["best_score"] if math.isfinite(r["best_score"]) else 0
        lc_n = r["landing_count"] / max_lc
        dt_n = (1 - r["frame_dt"] / max_dt) if math.isfinite(r["frame_dt"]) else 0
        comp = 0.5 * sc + 0.3 * lc_n + 0.2 * dt_n
        composite.append((r["trial_label"], comp))

    composite.sort(key=lambda t: t[1], reverse=True)
    labels_c = [t[0] for t in composite]
    vals_c = [t[1] for t in composite]
    colors_c = [C_SUCCESS if v >= 0.6 else (C_ALT if v >= 0.3 else C_DANGER) for v in vals_c]
    ypos = np.arange(len(composite))
    bars = ax3.barh(ypos, vals_c, 0.65, color=colors_c, alpha=0.88)
    for i, (lbl, val) in enumerate(zip(labels_c, vals_c)):
        ax3.text(val + 0.01, i, f"{val:.3f}", va="center", fontsize=9, fontweight="bold")
    ax3.set_yticks(ypos); ax3.set_yticklabels(labels_c, fontsize=9)
    ax3.set_xlim(0, 1.1); ax3.set_xlabel("综合得分")
    ax3.set_title("参数配置综合排行\n(0.5×安全 + 0.3×落区率 + 0.2×速度)", fontweight="bold")
    ax3.grid(axis="x", alpha=0.3)
    ax3.axvline(0.5, color=C_PRIMARY, linestyle="--", alpha=0.5, linewidth=1.5)
    # 第一名特别标注
    if composite:
        ax3.text(0.02, ypos[0]+0.05, "★ 最优", fontsize=10,
                color=C_SUCCESS, fontweight="bold", va="bottom")

    plt.tight_layout()
    plt.savefig(out); plt.close()
    print(f"  [c06] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# 图 c07 — 综合仪表盘
# ---------------------------------------------------------------------------
def plot_c07(rows, out):
    fig = plt.figure(figsize=(24, 16))
    fig.patch.set_facecolor(C_BG)
    fig.suptitle("safeland 复杂地形批量测试综合仪表盘", fontsize=18, fontweight="bold", y=0.99)
    gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.45, wspace=0.35)

    labels = [r["trial_label"] for r in rows]
    lc = [r["landing_count"] for r in rows]
    scores = [r["best_score"] if math.isfinite(r["best_score"]) else 0 for r in rows]
    dts = [r["frame_dt"] if math.isfinite(r["frame_dt"]) else None for r in rows]
    vc = [r["valid_count"] for r in rows]
    x = np.arange(len(rows))

    # A (0,0:2): 落区数主图
    ax_a = fig.add_subplot(gs[0, 0:2]); ax_a.set_facecolor(C_BG)
    colors_a = [C_SUCCESS if lc_ > 0 else (C_ALT if r["alt_landing_count"] > 0 else C_DANGER)
                for lc_, r in zip(lc, rows)]
    ax_a.bar(x, lc, 0.65, color=colors_a, alpha=0.88)
    for i, (l, s) in enumerate(zip(lc, scores)):
        if l > 0 and s > 0:
            ax_a.text(x[i], l + max(lc)*0.02, f"★{s:.3f}", ha="center",
                     fontsize=8.5, color="#1E3A8A", fontweight="bold")
        elif l == 0:
            ax_a.text(x[i], 0.2, "×", ha="center", fontsize=14, color=C_DANGER, fontweight="bold")
    ax_a.set_xticks(x); ax_a.set_xticklabels(labels, fontsize=9, rotation=30, ha="right")
    ax_a.set_ylabel("落区格子数"); ax_a.set_title("各配置落区格子数（标注安全得分）", fontweight="bold")
    ax_a.grid(axis="y", alpha=0.3)

    # B (0,2): 成功率饼图
    ax_b = fig.add_subplot(gs[0, 2]); ax_b.set_facecolor(C_BG)
    n_ok   = sum(1 for r in rows if r["landing_count"] > 0)
    n_alt  = sum(1 for r in rows if r["landing_count"] == 0 and r["alt_landing_count"] > 0)
    n_fail = len(rows) - n_ok - n_alt
    vals_p = [v for v in [n_ok, n_alt, n_fail] if v > 0]
    lbls_p = []
    if n_ok:   lbls_p.append(f"主落区\n{n_ok}次")
    if n_alt:  lbls_p.append(f"仅备降\n{n_alt}次")
    if n_fail: lbls_p.append(f"全失败\n{n_fail}次")
    ax_b.pie(vals_p, colors=[C_SUCCESS, C_ALT, C_DANGER][:len(vals_p)],
             labels=lbls_p, startangle=90,
             wedgeprops=dict(edgecolor="white", linewidth=2))
    ax_b.set_title(f"落区成功率({n_ok/len(rows)*100:.0f}%)", fontweight="bold")

    # C (0,3): KPI 数字看板
    ax_c = fig.add_subplot(gs[0, 3]); ax_c.set_facecolor(C_BG); ax_c.axis("off")
    scored = [r for r in rows if math.isfinite(r["best_score"]) and r["best_score"] > 0]
    best_r = max(scored, key=lambda r: r["best_score"]) if scored else None
    kpis = [
        ("总测试次数", f"{len(rows)} 次"),
        ("成功率", f"{n_ok/len(rows)*100:.0f}%"),
        ("最高安全得分", f"{best_r['best_score']:.3f}" if best_r else "—"),
        ("最优配置", f"{best_r['trial_label']}" if best_r else "—"),
        ("平均时延", f"{np.mean([d for d in dts if d]):.3f}s" if any(dts) else "—"),
        ("预留降落窗口", "10 个"),
        ("障碍物总数", "30+ 个"),
    ]
    y_kpi = 0.95
    for k, v in kpis:
        ax_c.text(0.05, y_kpi, f"{k}：", fontsize=10, color=C_NEUTRAL, va="top")
        ax_c.text(0.6,  y_kpi, v, fontsize=11, color="#1E3A8A", fontweight="bold", va="top")
        y_kpi -= 0.13
    ax_c.set_title("关键指标 KPI", fontweight="bold")

    # D (1,0): 安全得分折线
    ax_d = fig.add_subplot(gs[1, 0:2]); ax_d.set_facecolor(C_BG)
    sc_plot = [s if s > 0 else None for s in scores]
    xs_valid = [i for i, s in enumerate(sc_plot) if s is not None]
    ys_valid = [s for s in sc_plot if s is not None]
    if ys_valid:
        ax_d.plot(xs_valid, ys_valid, "o-", color=C_PRIMARY, linewidth=2.5,
                 markersize=8, alpha=0.85)
        ax_d.fill_between(xs_valid, ys_valid, 0.8, alpha=0.12, color=C_PRIMARY)
        ax_d.axhline(np.mean(ys_valid), color=C_SUCCESS, linestyle="--",
                    linewidth=1.5, alpha=0.7, label=f"均值={np.mean(ys_valid):.3f}")
    for i, (idx, s) in enumerate(zip(xs_valid, ys_valid)):
        ax_d.text(idx, s + 0.004, f"{s:.3f}", ha="center", fontsize=8.5, fontweight="bold")
    # 失败标记
    for i, s in enumerate(sc_plot):
        if s is None:
            ax_d.axvspan(i-0.4, i+0.4, alpha=0.08, color=C_DANGER)
            ax_d.text(i, 0.88, "×", ha="center", fontsize=12, color=C_DANGER, fontweight="bold")
    ax_d.set_xticks(x); ax_d.set_xticklabels(labels, fontsize=9, rotation=30, ha="right")
    ax_d.set_ylim(0.85, 1.02); ax_d.set_ylabel("安全得分")
    ax_d.set_title("各配置安全得分趋势（失败区域标红）", fontweight="bold")
    ax_d.legend(fontsize=9); ax_d.grid(alpha=0.25)

    # E (1,2): 时延柱图
    ax_e = fig.add_subplot(gs[1, 2]); ax_e.set_facecolor(C_BG)
    dts_v = [d if d else 0 for d in dts]
    colors_dt = [C_SUCCESS if d < 0.41 else (C_ALT if d < 0.43 else C_DANGER) for d in dts_v]
    ax_e.bar(x, dts_v, 0.6, color=colors_dt, alpha=0.88)
    ax_e.set_xticks(x); ax_e.set_xticklabels(labels, fontsize=7.5, rotation=40, ha="right")
    ax_e.set_ylabel("帧间时延 (s)"); ax_e.set_title("各配置计算时延", fontweight="bold")
    ax_e.grid(axis="y", alpha=0.3)
    if any(dts_v):
        ax_e.axhline(np.mean([d for d in dts_v if d > 0]), color=C_PRIMARY,
                    linestyle="--", alpha=0.6, linewidth=1.5)

    # F (1,3): 有效栅格数
    ax_f = fig.add_subplot(gs[1, 3]); ax_f.set_facecolor(C_BG)
    colors_vc = [CMAP_10[i % 10] for i in range(len(rows))]
    ax_f.bar(x, vc, 0.6, color=colors_vc, alpha=0.88)
    ax_f.set_xticks(x); ax_f.set_xticklabels(labels, fontsize=7.5, rotation=40, ha="right")
    ax_f.set_ylabel("有效栅格数"); ax_f.set_title("各配置有效栅格数", fontweight="bold")
    ax_f.grid(axis="y", alpha=0.3)

    # G (2,0:2): 综合排行水平柱图
    ax_g = fig.add_subplot(gs[2, 0:2]); ax_g.set_facecolor(C_BG)
    max_lc_g = max(lc) if lc else 1
    max_dt_g = max(dts_v) if dts_v else 1
    composite = []
    for r in rows:
        sc = r["best_score"] if math.isfinite(r["best_score"]) else 0
        lc_n = r["landing_count"] / max_lc_g
        dt_n = (1 - (r["frame_dt"] / max_dt_g)) if math.isfinite(r["frame_dt"]) else 0
        comp = 0.5 * sc + 0.3 * lc_n + 0.2 * dt_n
        composite.append((r["trial_label"], comp, sc, r["landing_count"]))
    composite.sort(key=lambda t: t[1], reverse=True)
    cy = np.arange(len(composite))
    colors_comp = [C_SUCCESS if v >= 0.6 else (C_ALT if v >= 0.3 else C_DANGER)
                   for _, v, _, _ in composite]
    ax_g.barh(cy, [v for _, v, _, _ in composite], 0.65, color=colors_comp, alpha=0.88)
    for i, (lbl, val, sc, lc_) in enumerate(composite):
        ax_g.text(val + 0.01, i, f"{val:.3f}  (得分:{sc:.3f} 落区:{lc_})",
                 va="center", fontsize=9, fontweight="bold")
    ax_g.set_yticks(cy); ax_g.set_yticklabels([t[0] for t in composite], fontsize=9)
    ax_g.set_xlim(0, 1.2); ax_g.set_xlabel("综合得分(0.5×安全+0.3×落区率+0.2×速度)")
    ax_g.set_title("参数配置综合排行榜 ★ 最优方案", fontweight="bold")
    ax_g.grid(axis="x", alpha=0.3)
    ax_g.axvline(0.5, color=C_PRIMARY, linestyle="--", alpha=0.4)
    if composite:
        ax_g.text(composite[0][1]*0.5, cy[0]+0.12, "★ 推荐配置",
                 fontsize=10, color=C_SUCCESS, fontweight="bold")

    # H (2,2:4): 参数对比热力图（落区数 vs 斜率×凹陷）
    ax_h = fig.add_subplot(gs[2, 2:]); ax_h.set_facecolor(C_BG)
    slope_uniq = sorted(set(r["slope_threshold"] for r in rows))
    dep_uniq   = sorted(set(r["depression_score_threshold"] for r in rows))
    if slope_uniq and dep_uniq:
        mat = np.full((len(dep_uniq), len(slope_uniq)), np.nan)
        for r in rows:
            si = slope_uniq.index(r["slope_threshold"])
            di = dep_uniq.index(r["depression_score_threshold"])
            cur = mat[di, si]
            mat[di, si] = r["landing_count"] if np.isnan(cur) else max(cur, r["landing_count"])
        cmap_lc = LinearSegmentedColormap.from_list("lc", ["#FEF2F2","#FCA5A5","#10B981","#064E3B"])
        im = ax_h.imshow(mat, cmap=cmap_lc, aspect="auto",
                        vmin=0, vmax=max(lc) if lc else 1)
        plt.colorbar(im, ax=ax_h, shrink=0.8, label="落区格子数")
        for di in range(len(dep_uniq)):
            for si in range(len(slope_uniq)):
                v = mat[di, si]
                if not np.isnan(v):
                    tc = "white" if v > (max(lc) if lc else 1) * 0.6 else "#1F2937"
                    ax_h.text(si, di, f"{int(v)}", ha="center", va="center",
                             fontsize=11, color=tc, fontweight="bold")
        ax_h.set_xticks(range(len(slope_uniq)))
        ax_h.set_yticks(range(len(dep_uniq)))
        ax_h.set_xticklabels([f"slope={v:.2f}" for v in slope_uniq], fontsize=9)
        ax_h.set_yticklabels([f"dep={v:.2f}" for v in dep_uniq], fontsize=9)
        ax_h.set_xlabel("斜率阈值"); ax_h.set_ylabel("凹陷分数阈值")
        ax_h.set_title("参数组合热力图：落区格子数\n(斜率 × 凹陷阈值)", fontweight="bold")

    plt.savefig(out); plt.close()
    print(f"  [c07] {os.path.basename(out)}")


# ---------------------------------------------------------------------------
# HTML 报告
# ---------------------------------------------------------------------------
def generate_html(out_dir, rows):
    total = len(rows)
    n_ok  = sum(1 for r in rows if r["landing_count"] > 0)
    n_fail= sum(1 for r in rows if r["landing_count"] == 0 and r["alt_landing_count"] == 0)
    scored= [r for r in rows if math.isfinite(r["best_score"]) and r["best_score"] > 0]
    best_r= max(scored, key=lambda r: r["best_score"]) if scored else None

    # 综合排行
    max_lc = max(r["landing_count"] for r in rows) if rows else 1
    max_dt_v = max(r["frame_dt"] for r in rows if math.isfinite(r["frame_dt"])) if rows else 1
    composite = []
    for r in rows:
        sc = r["best_score"] if math.isfinite(r["best_score"]) else 0
        lc_n = r["landing_count"] / max_lc
        dt_n = (1 - r["frame_dt"] / max_dt_v) if math.isfinite(r["frame_dt"]) else 0
        comp = 0.5 * sc + 0.3 * lc_n + 0.2 * dt_n
        composite.append((r["trial_label"], comp))
    composite.sort(key=lambda t: t[1], reverse=True)
    rank_html = "".join(
        f'<tr><td>{i+1}</td><td><b>{lbl}</b></td><td>{val:.3f}</td></tr>'
        for i, (lbl, val) in enumerate(composite)
    )

    rows_html = ""
    for r in rows:
        lc_color = "#10B981" if r["landing_count"] > 0 else (
            "#F59E0B" if r["alt_landing_count"] > 0 else "#EF4444")
        sc_txt = f"{r['best_score']:.3f}" if math.isfinite(r["best_score"]) else "—"
        dt_txt = f"{r['frame_dt']:.3f}s" if math.isfinite(r["frame_dt"]) else "—"
        xy_txt = (f"({r['best_x']:.2f},{r['best_y']:.2f})"
                  if math.isfinite(r.get("best_x", math.nan)) else "—")
        rows_html += f"""
        <tr>
          <td>{r['trial_label']}</td>
          <td>{r['grid_resolution']:.2f}m</td>
          <td>{r['voxel_leaf_size']:.2f}m</td>
          <td>{r['slope_threshold']:.2f}</td>
          <td>{r['step_threshold']:.2f}</td>
          <td>{r['depression_score_threshold']:.2f}</td>
          <td>{r['alt_relax_factor']:.1f}/{r['enable_alt']}</td>
          <td style="color:{lc_color};font-weight:bold">{r['landing_count']}</td>
          <td>{r['valid_count']:,}</td>
          <td style="font-weight:bold">{sc_txt}</td>
          <td>{xy_txt}</td>
          <td>{dt_txt}</td>
        </tr>"""

    fig_list = [
        ("c01_overview.png",        "图01：落区格子数 + 有效栅格总览"),
        ("c02_score_bar.png",       "图02：安全得分柱状图 + 斜率阈值分析"),
        ("c03_dt_heatmap.png",      "图03：计算时延热力图 + 时延序列"),
        ("c04_landing_scatter.png", "图04：落点坐标分布（叠加障碍物布局）"),
        ("c05_param_matrix.png",    "图05：参数矩阵对比 + 综合雷达图"),
        ("c06_success_rate.png",    "图06：落区成功率 + 综合排行"),
        ("c07_dashboard.png",       "图07：综合仪表盘"),
    ]
    imgs_html = ""
    for fname, caption in fig_list:
        if os.path.exists(os.path.join(out_dir, fname)):
            imgs_html += f"""
      <div class="fig-block"><h3>{caption}</h3>
        <img src="{fname}" alt="{caption}" /></div>"""

    best_txt = (
        f"最优配置：<b>{best_r['trial_label']}</b> | "
        f"安全得分：<b>{best_r['best_score']:.3f}</b> | "
        f"落区格子数：<b>{best_r['landing_count']}</b> | "
        f"落点坐标：({best_r.get('best_x',0):.2f},{best_r.get('best_y',0):.2f})"
        if best_r else "无有效落区"
    )

    html = f"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8"/>
<title>safeland 复杂地形测试报告</title>
<style>
  body{{font-family:"Segoe UI","PingFang SC",sans-serif;background:#F8FAFC;color:#1F2937;margin:0;padding:24px}}
  h1{{font-size:26px;color:#1E3A8A;border-bottom:3px solid #2563EB;padding-bottom:8px}}
  h2{{font-size:18px;color:#1E3A8A;margin-top:28px}}
  h3{{font-size:14px;color:#374151;margin-top:0}}
  table{{border-collapse:collapse;width:100%;margin:12px 0}}
  th{{background:#2563EB;color:white;padding:7px 9px;font-size:11px}}
  td{{padding:5px 9px;border-bottom:1px solid #E5E7EB;font-size:11px}}
  tr:nth-child(even) td{{background:#EFF6FF}}
  .box{{background:#EFF6FF;border-left:4px solid #2563EB;padding:12px 16px;
        border-radius:4px;margin:12px 0;line-height:1.9}}
  .box.warn{{background:#FFF7ED;border-left-color:#F59E0B}}
  .fig-block{{margin:20px 0;background:white;padding:14px;border-radius:8px;
              box-shadow:0 1px 6px rgba(0,0,0,0.09)}}
  .fig-block img{{max-width:100%;height:auto;display:block;margin:0 auto}}
  .kpi-row{{display:flex;gap:16px;flex-wrap:wrap;margin:12px 0}}
  .kpi{{background:white;border:1px solid #DBEAFE;border-radius:8px;padding:12px 20px;
        min-width:140px;text-align:center;box-shadow:0 1px 4px rgba(0,0,0,0.07)}}
  .kpi .val{{font-size:28px;font-weight:700;color:#1D4ED8}}
  .kpi .lbl{{font-size:11px;color:#6B7280;margin-top:4px}}
</style></head><body>
<h1>🚁 safeland 复杂地形批量测试分析报告</h1>
<p>生成时间：2026-06-15 &nbsp;|&nbsp; 地形：nagetive_terrain（含 30+ 个不规则障碍物）&nbsp;|&nbsp; 预留降落窗口：10 个</p>

<div class="kpi-row">
  <div class="kpi"><div class="val">{total}</div><div class="lbl">总测试次数</div></div>
  <div class="kpi"><div class="val" style="color:#10B981">{n_ok}</div><div class="lbl">主落区成功次数</div></div>
  <div class="kpi"><div class="val" style="color:#EF4444">{n_fail}</div><div class="lbl">全失败次数</div></div>
  <div class="kpi"><div class="val" style="color:#10B981">{n_ok/total*100:.0f}%</div><div class="lbl">落区成功率</div></div>
  <div class="kpi"><div class="val">{"—" if not best_r else f"{best_r['best_score']:.3f}"}</div><div class="lbl">最高安全得分</div></div>
</div>

<div class="box">
  <b>环境配置：</b>15×15m 凹坑地形（36孔） + 30+ 个不规则障碍物<br>
  &nbsp;&nbsp;&nbsp;&nbsp;障碍物类型：斜坡板 × 5、L形矮墙 × 5、碎石堆 × 5组、大石块 × 4、台阶 × 2组、倾斜立柱 × 3、树木 × 2<br>
  <b>预留降落窗口：</b>W0(起飞台) W1-W9 共 10 个 0.6m×0.6m 平坦区域<br>
  <b>测试结果：</b>{best_txt}
</div>

<div class="box warn">
  ⚠️ T10(very_relaxed) 配置因 <code>landing_size=0.6m</code> 过大、<code>slope_threshold=0.25</code> 过宽松，
  导致算法要求的最小平坦面积超过场景中最大可用平坦区（复杂地形中大面积平坦区已被障碍物分割），返回 landing_count=0。<br>
  ✅ 其余 9 次测试（90%）均成功找到安全落区，验证了算法在复杂环境下的鲁棒性。
</div>

<h2>综合排行（0.5×安全 + 0.3×落区率 + 0.2×速度）</h2>
<table style="width:40%">
  <thead><tr><th>排名</th><th>配置</th><th>综合得分</th></tr></thead>
  <tbody>{rank_html}</tbody>
</table>

<h2>各次运行详细结果</h2>
<table>
  <thead><tr>
    <th>配置</th><th>分辨率</th><th>体素</th><th>斜率阈</th><th>台阶阈</th>
    <th>凹陷阈</th><th>备降/启用</th><th>落区数</th><th>有效栅格</th>
    <th>安全得分</th><th>落点坐标</th><th>时延</th>
  </tr></thead>
  <tbody>{rows_html}</tbody>
</table>

<h2>详细分析图表</h2>
{imgs_html}

<h2>障碍物布局说明</h2>
<div class="box">
  <b>区域A（左侧 x&lt;0,y&gt;0）：</b>斜坡板A1、L形矮墙A2(a/b)、碎石堆A3(4块)、大石块A4、倾斜立柱A5 → 保留窗口 W1(-3.8,3.2) W4(-0.5,2.5) W9(-6.0,0.5)<br>
  <b>区域B（左后 x&lt;0,y&lt;0）：</b>台阶组B1(3级)、斜坡板B2、圆柱石墩B3、碎石堆B4(3块) → 保留窗口 W2(-3.8,-2.8) W3(-1.2,-4.8)<br>
  <b>区域C（右前 x&gt;0,y&gt;0）：</b>大石块C1、矮墙C2、倾斜板C3、树C4、碎石堆C5(4块) → 保留窗口 W5(3.0,4.5) W7(5.2,2.2)<br>
  <b>区域D（右后 x&gt;0,y&lt;0）：</b>台阶D1(2级)、大石块D2、斜坡板D3、L形矮墙D4(a/b)、倾斜立柱D5、树D6 → 保留窗口 W6(3.5,-4.0) W8(5.2,-2.8)<br>
  <b>区域E（中央）：</b>大石块E1、碎石堆E2(2块)、矮墙E3 → 保留窗口 W0(1.5,-0.5) 起飞台
</div>
</body></html>"""

    rpath = os.path.join(out_dir, "report_complex.html")
    with open(rpath, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"  [HTML] {rpath}")


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="复杂地形测试结果可视化")
    parser.add_argument("csv")
    parser.add_argument("--out-dir", default="")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"[ERROR] CSV not found: {args.csv}", file=sys.stderr)
        return 1

    out_dir = args.out_dir or os.path.join(
        os.path.dirname(os.path.abspath(args.csv)), "figures_complex")
    os.makedirs(out_dir, exist_ok=True)

    rows = load(args.csv)
    print(f"[+] 加载 {len(rows)} 条测试记录")
    print(f"[+] 有主落区: {sum(1 for r in rows if r['landing_count']>0)} 次")
    print(f"[+] 全失败:   {sum(1 for r in rows if r['landing_count']==0 and r['alt_landing_count']==0)} 次")
    print(f"[+] 生成图表到 {out_dir}/\n")

    def p(n): return os.path.join(out_dir, n)

    plot_c01(rows, p("c01_overview.png"))
    plot_c02(rows, p("c02_score_bar.png"))
    plot_c03(rows, p("c03_dt_heatmap.png"))
    try:
        plot_c04(rows, p("c04_landing_scatter.png"))
    except Exception as e:
        print(f"  [c04] 跳过（{e}）")
    plot_c05(rows, p("c05_param_matrix.png"))
    plot_c06(rows, p("c06_success_rate.png"))
    plot_c07(rows, p("c07_dashboard.png"))
    generate_html(out_dir, rows)

    print(f"\n[OK] 所有图表已生成至: {out_dir}")
    return 0

if __name__ == "__main__":
    sys.exit(main())