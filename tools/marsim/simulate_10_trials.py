#!/usr/bin/env python3
"""
simulate_10_trials.py  — 基于算法参数空间生成 10 次（或更多）模拟试验数据

设计原则
---------
本脚本不调用 ROS，而是根据 safeland 算法的数学模型直接生成
与真实批量测试结构完全一致的 CSV 数据，供可视化分析脚本使用。

数据模型
---------
  landing_count  ~ f(slope_thresh, step_thresh, dep_thresh,
                     grid_resolution, voxel_leaf, polar_res, sensing_horizon)

  算法核心：
    1. 有效格子数  valid_count = 理论地图面积 / resolution² × density(polar_res, horizon)
    2. 可靠格子数  reliable_count = valid_count × reliability(voxel_leaf)
    3. 安全格子数  safe_count = reliable_count × safe_rate(slope_t, step_t, dep_t, polar_res)
    4. 落区格子数  landing_count = max(0, safe_count × landing_efficiency(resolution))
    5. 最优得分    best_score = sigmoid(slope_t + step_t - dep_t × 0.5)
    6. 帧间计算时延 dt_ms = base_dt / (resolution² × speedup(voxel_leaf))
    7. 落点抖动   jitter_m = base_jitter × (1 - stability(median_window, polar_res))
    8. 备降区      alt_landing_count = 备降搜索扩展后的候选点数量

输出 CSV 字段与 collect_safeland_metrics.py 完全一致，额外添加
  trial_id     — 试验编号 1~N
  terrain_tag  — 地形场景标签 (flat / gentle_slope / rough / mixed / obstacle_rich)
"""

import argparse
import csv
import math
import os
import random
import sys

# ===========================================================================
# 试验场景定义（10 组，覆盖不同地形与传感器条件）
# ===========================================================================

TRIALS = [
    # trial 1：开阔平坦地形，基线参数
    dict(trial_id=1, terrain_tag="flat",
         slope_base=0.04, rough_base=0.01, step_base=0.02, dep_base=0.05,
         grid_resolution=0.10, voxel_leaf_size=0.05, min_cell_points=3,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.5, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 2：平坦地形，更细分辨率
    dict(trial_id=2, terrain_tag="flat_hires",
         slope_base=0.04, rough_base=0.01, step_base=0.02, dep_base=0.05,
         grid_resolution=0.08, voxel_leaf_size=0.03, min_cell_points=3,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.5, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 3：平坦地形，粗分辨率
    dict(trial_id=3, terrain_tag="flat_lores",
         slope_base=0.04, rough_base=0.01, step_base=0.02, dep_base=0.05,
         grid_resolution=0.12, voxel_leaf_size=0.08, min_cell_points=3,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.5, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 4：轻微坡地，基线
    dict(trial_id=4, terrain_tag="gentle_slope",
         slope_base=0.10, rough_base=0.025, step_base=0.04, dep_base=0.10,
         grid_resolution=0.10, voxel_leaf_size=0.05, min_cell_points=3,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.5, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 5：复杂粗糙地形（乱石/碎石）
    dict(trial_id=5, terrain_tag="rough_terrain",
         slope_base=0.18, rough_base=0.06, step_base=0.08, dep_base=0.20,
         grid_resolution=0.10, voxel_leaf_size=0.05, min_cell_points=5,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.5, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 6：复杂粗糙地形 + 更大放宽系数
    dict(trial_id=6, terrain_tag="rough_relax2x",
         slope_base=0.18, rough_base=0.06, step_base=0.08, dep_base=0.20,
         grid_resolution=0.10, voxel_leaf_size=0.05, min_cell_points=5,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=2.0, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 7：复杂粗糙地形 + 关闭备降区
    dict(trial_id=7, terrain_tag="rough_no_alt",
         slope_base=0.18, rough_base=0.06, step_base=0.08, dep_base=0.20,
         grid_resolution=0.10, voxel_leaf_size=0.05, min_cell_points=5,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.5, enable_alt_landing=False,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 8：混合地形 + 稀疏传感器（高 polar_res）
    dict(trial_id=8, terrain_tag="mixed_sparse_sensor",
         slope_base=0.12, rough_base=0.04, step_base=0.05, dep_base=0.15,
         grid_resolution=0.10, voxel_leaf_size=0.05, min_cell_points=3,
         polar_res=0.4, sensing_horizon=20,
         alt_relax_factor=1.5, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 9：障碍物丰富场景（建筑、树木旁降落）
    dict(trial_id=9, terrain_tag="obstacle_rich",
         slope_base=0.08, rough_base=0.03, step_base=0.06, dep_base=0.25,
         grid_resolution=0.10, voxel_leaf_size=0.05, min_cell_points=5,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.5, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),

    # trial 10：最佳优化配置（round4 推荐组合）
    dict(trial_id=10, terrain_tag="optimized_full",
         slope_base=0.06, rough_base=0.02, step_base=0.03, dep_base=0.08,
         grid_resolution=0.08, voxel_leaf_size=0.04, min_cell_points=3,
         polar_res=0.2, sensing_horizon=30,
         alt_relax_factor=1.8, enable_alt_landing=True,
         slope_threshold=0.15, step_threshold=0.10,
         depression_score_threshold=0.30),
]

# ===========================================================================
# 地图模拟参数
# ===========================================================================
MAP_WIDTH_M  = 10.0   # 地图宽度 m
MAP_HEIGHT_M = 10.0   # 地图高度 m
RANDOM_SEED  = 42
FRAMES_PER_TRIAL = 20  # 每次试验模拟帧数（用于统计稳定性）


# ===========================================================================
# 物理模型函数
# ===========================================================================

def density(polar_res: float, sensing_horizon: float) -> float:
    """点云密度因子 [0,1]：polar_res 越小、视距越远，密度越高"""
    base_res = 0.2
    base_hor = 30.0
    density_factor = (base_res / polar_res) * (sensing_horizon / base_hor)
    return min(1.0, density_factor * 0.85)


def reliability(voxel_leaf: float, min_cell_points: float) -> float:
    """
    可靠格子比例：
    - voxel 太大 → 稀疏点 → 少格子通过 min_cell_points 检测
    - voxel 太小 → 点密集但耗时，可靠性略降（噪声增多）
    """
    ideal_voxel = 0.04
    penalty = abs(voxel_leaf - ideal_voxel) * 2.5
    return max(0.3, 1.0 - penalty)


def safe_rate(slope_base: float, step_base: float, dep_base: float,
              slope_thresh: float, step_thresh: float, dep_thresh: float,
              polar_res: float) -> float:
    """
    安全格子比例：
    通过坡度/阶梯/凹陷三重约束的概率。
    地形越复杂（base 值越高）、阈值越严格，安全率越低。
    """
    # 各指标与阈值的裕量比例
    slope_margin = max(0.0, (slope_thresh - slope_base) / slope_thresh)
    step_margin  = max(0.0, (step_thresh  - step_base)  / step_thresh)
    dep_margin   = max(0.0, (dep_thresh   - dep_base)   / dep_thresh)

    # 传感器稀疏度惩罚（稀疏传感器让地形看起来更不规则）
    sensor_penalty = max(0.0, (polar_res - 0.2) * 0.8)

    # 三维约束综合通过率（独立假设简化模型）
    combined = slope_margin * step_margin * dep_margin
    combined = max(0.0, combined - sensor_penalty)
    return combined


def landing_efficiency(resolution: float) -> float:
    """
    分辨率对落区检测效率的影响：
    - 太粗 (0.12) → 无法精确定位 2m×2m 落区的细节 → 效率低
    - 太细 (0.08) → 点密集，落区更精确 → 效率高
    - 中间 (0.10) → 基准
    """
    ideal_res = 0.09
    penalty = abs(resolution - ideal_res) * 3.0
    return max(0.5, 1.0 - penalty)


def compute_dt_ms(resolution: float, voxel_leaf: float,
                  grid_w: float, grid_h: float) -> float:
    """
    计算帧处理时延（ms）。
    分辨率越细 → 格子越多 → 越慢
    voxel 越大 → 点云稀疏 → 略快
    """
    num_cells = (grid_w / resolution) * (grid_h / resolution)
    base_per_cell_us = 0.8  # μs/格子（基准）
    voxel_speedup = 1.0 + (voxel_leaf - 0.05) * 4.0  # voxel 越大越快
    dt_ms = (num_cells * base_per_cell_us / 1000.0) / max(0.5, voxel_speedup)
    return max(5.0, dt_ms + random.gauss(0, 1.5))


def compute_jitter(resolution: float, polar_res: float,
                   median_window: int = 5) -> float:
    """
    落点帧间抖动（m）：
    - 传感器越稀疏 → 抖动越大
    - 分辨率越细 → 抖动略小（精确定位）
    - median_window 越大 → 滤波后抖动越小
    """
    base_jitter = 0.15 + polar_res * 0.4 + (0.10 - resolution) * 2.0
    filter_reduction = 1.0 - (median_window - 1) * 0.08
    jitter = base_jitter * max(0.3, filter_reduction)
    return max(0.02, jitter + random.gauss(0, 0.02))


def simulate_trial(t: dict, rng: random.Random, n_frames: int = FRAMES_PER_TRIAL) -> list:
    """
    模拟一次试验，生成 FRAMES_PER_TRIAL 帧数据行。

    返回 list of dict，每个 dict 对应 CSV 一行。
    """
    rows = []

    res   = t["grid_resolution"]
    voxel = t["voxel_leaf_size"]
    mcp   = t["min_cell_points"]
    pr    = t["polar_res"]
    sh    = t["sensing_horizon"]
    s_t   = t["slope_threshold"]
    st_t  = t["step_threshold"]
    dep_t = t["depression_score_threshold"]
    relax = t["alt_relax_factor"]
    en_alt = t["enable_alt_landing"]

    # ---- 地形基础参数（带噪声）----
    slope_b = t["slope_base"]
    step_b  = t["step_base"]
    dep_b   = t["dep_base"]

    # ---- 地图格子数 ----
    cols_n = int(MAP_WIDTH_M  / res)
    rows_n = int(MAP_HEIGHT_M / res)
    total_cells = rows_n * cols_n

    # ---- 稳定统计 ----
    prev_best_xy = None
    jitter_sum = 0.0
    jitter_max = 0.0
    jitter_samples = 0

    dt_sum = 0.0
    dt_max = 0.0

    for frame in range(n_frames):
        # -- 本帧地形噪声 --
        slope_now = slope_b + rng.gauss(0, slope_b * 0.1)
        step_now  = step_b  + rng.gauss(0, step_b  * 0.1)
        dep_now   = dep_b   + rng.gauss(0, dep_b   * 0.1)

        # -- 有效格子 --
        den = density(pr, sh)
        valid_count = int(total_cells * den * rng.uniform(0.85, 1.0))

        # -- 可靠格子 --
        rel = reliability(voxel, mcp)
        reliable_count = int(valid_count * rel)

        # -- 安全格子 --
        sr = safe_rate(slope_now, step_now, dep_now,
                       s_t, st_t, dep_t, pr)
        safe_count = int(reliable_count * sr)

        # -- 主落区 --
        le = landing_efficiency(res)
        landing_count = int(safe_count * le * 0.25)  # 落区约占安全区 25%
        landing_count = max(0, landing_count + rng.randint(-5, 5))

        # -- 最优得分 --
        best_score = math.nan
        if landing_count > 0:
            raw_score = ((s_t - slope_now) / s_t * 0.35 +
                         (st_t - step_now) / st_t * 0.30 +
                         (1.0 - dep_now / dep_t) * 0.25 +
                         0.10)
            best_score = max(0.0, min(1.0,
                raw_score + rng.gauss(0, 0.02)))

        # -- 备降区 --
        alt_landing_count = 0
        alt_best_score    = math.nan

        if landing_count == 0 and en_alt:
            s_t_rel   = s_t  * relax
            st_t_rel  = st_t * relax
            dep_t_rel = min(dep_t * relax, 1.0)
            sr_alt = safe_rate(slope_now, step_now, dep_now,
                               s_t_rel, st_t_rel, dep_t_rel, pr)
            safe_alt = int(reliable_count * sr_alt)
            alt_landing_count = int(safe_alt * le * 0.20)
            alt_landing_count = max(0, alt_landing_count + rng.randint(-3, 3))
            if alt_landing_count > 0:
                raw_alt = ((s_t_rel - slope_now) / s_t_rel * 0.35 +
                           (st_t_rel - step_now) / st_t_rel * 0.30 +
                           (1.0 - dep_now / dep_t_rel) * 0.25 + 0.10)
                alt_best_score = max(0.0, min(1.0,
                    raw_alt * 0.85 + rng.gauss(0, 0.02)))  # 备降区得分略低

        # -- 计算时延 --
        dt_ms = compute_dt_ms(res, voxel, MAP_WIDTH_M, MAP_HEIGHT_M)
        dt_sum += dt_ms
        dt_max = max(dt_max, dt_ms)

        # -- 最优落点 (XY) --
        best_x = best_y = math.nan
        if landing_count > 0:
            best_x = rng.uniform(-MAP_WIDTH_M / 2 * 0.7, MAP_WIDTH_M / 2 * 0.7)
            best_y = rng.uniform(-MAP_HEIGHT_M / 2 * 0.7, MAP_HEIGHT_M / 2 * 0.7)

        # -- 抖动 --
        jitter = math.nan
        if math.isfinite(best_x) and prev_best_xy is not None:
            jitter = math.hypot(best_x - prev_best_xy[0],
                                best_y - prev_best_xy[1])
            # 中值滤波效果（5帧窗口）简化为单帧随机衰减
            jitter *= rng.uniform(0.35, 0.55)
            jitter_sum += jitter
            jitter_max = max(jitter_max, jitter)
            jitter_samples += 1
        if math.isfinite(best_x):
            prev_best_xy = (best_x, best_y)

        mean_jitter = jitter_sum / jitter_samples if jitter_samples > 0 else math.nan
        mean_dt     = dt_sum / (frame + 1)

        row = {
            # -- 试验标识 --
            "trial_id":    t["trial_id"],
            "terrain_tag": t["terrain_tag"],
            "frame":       frame + 1,
            "tag":         f"{t['terrain_tag']}__res{res}__voxel{voxel}"
                           f"__polar{pr}__horizon{sh}__relax{relax}__alt{en_alt}",
            "pcd":         f"/data/terrain_{t['terrain_tag']}.pcd",

            # -- 地图信息 --
            "rows":        rows_n,
            "cols":        cols_n,
            "resolution":  res,
            "frame_id":    "world",
            "layers":      "elevation;slope;roughness;step;depression_raw;"
                           "depression_norm;depression_score;landing_center;"
                           "landing_score;alt_landing_center",
            "valid_count":    valid_count,
            "reliable_count": reliable_count,
            "safe_count":     safe_count,

            # -- 主落区 --
            "landing_count":    landing_count,
            "best_score":       best_score,
            "best_x":           best_x,
            "best_y":           best_y,
            "landing_centroid_x": best_x,
            "landing_centroid_y": best_y,

            # -- 备降区 --
            "alt_landing_count": alt_landing_count,
            "alt_best_score":    alt_best_score,

            # -- 稳定性指标 --
            "target_jitter_m":      jitter,
            "target_jitter_mean_m": mean_jitter,
            "target_jitter_max_m":  jitter_max if jitter_samples > 0 else math.nan,
            "frame_dt":             dt_ms,
            "frame_dt_mean":        mean_dt,
            "frame_dt_max":         dt_max if frame > 0 else math.nan,

            # -- 算法参数 --
            "slope_threshold":                t["slope_threshold"],
            "depression_score_threshold":     t["depression_score_threshold"],
            "landing_valid_ratio_threshold":  0.98,
            "landing_height_range_threshold": 0.05,
            "step_threshold":                 t["step_threshold"],
            "landing_size":                   0.3,
            "landing_safety_margin":          0.0,
            "grid_resolution":                res,
            "voxel_leaf_size":                voxel,
            "min_cell_points":                mcp,
            "samples":                        frame + 1,

            # -- round4 新增 --
            "alt_relax_factor":   relax,
            "enable_alt_landing": str(en_alt).lower(),
            "polar_res":          pr,
            "sensing_horizon":    sh,
            "metrics_csv":        "",

            # -- 用于热力图的地形坐标（随机模拟落点在地图上的位置）--
            "slope_local":   slope_now,
            "step_local":    step_now,
            "dep_local":     dep_now,
        }
        rows.append(row)

    return rows


def main():
    parser = argparse.ArgumentParser(
        description="生成 safeland 10次试验模拟数据 CSV")
    parser.add_argument("--out", default="",
                        help="输出 CSV 路径（默认：<root>/marsim_benchmark_results/simulated_trials.csv）")
    parser.add_argument("--seed", type=int, default=RANDOM_SEED)
    parser.add_argument("--frames", type=int, default=FRAMES_PER_TRIAL,
                        help="每次试验模拟帧数")
    args = parser.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_csv = args.out or os.path.join(root, "marsim_benchmark_results", "simulated_trials.csv")
    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)

    rng = random.Random(args.seed)
    frames_per_trial = args.frames

    all_rows = []
    for t in TRIALS:
        trial_rows = simulate_trial(t, rng, frames_per_trial)
        all_rows.extend(trial_rows)
        last = trial_rows[-1]
        bs = last['best_score']
        bs_txt = f"{bs:.3f}" if (isinstance(bs, float) and not math.isnan(bs)) else "nan"
        print(f"  Trial {t['trial_id']:>2} [{t['terrain_tag']:<22}] "
              f"frames={len(trial_rows)} "
              f"landing={last['landing_count']} "
              f"best_score={bs_txt} "
              f"dt={last['frame_dt_mean']:.1f}ms")

    if not all_rows:
        print("[ERROR] No data generated", file=sys.stderr)
        return 1

    fieldnames = list(all_rows[0].keys())
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in all_rows:
            # 格式化浮点数，nan 写为空字符串
            formatted = {}
            for k, v in row.items():
                if isinstance(v, float):
                    formatted[k] = "" if math.isnan(v) else f"{v:.6g}"
                else:
                    formatted[k] = v
            writer.writerow(formatted)

    print(f"\n[OK] 写入 {len(all_rows)} 行 → {out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
