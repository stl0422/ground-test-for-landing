#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <ros/ros.h>

namespace {

using Matrix = grid_map::Matrix;

// 根据想要的2m*2m着陆区，算出在地图上对应几个像素
int makeKernelSize(double window_size_m, float resolution, int minimum_size, bool force_odd) {
  int kernel_size = std::max(minimum_size, static_cast<int>(std::ceil(window_size_m / resolution)));
  if (force_odd && kernel_size % 2 == 0) {
    ++kernel_size;
  }
  return kernel_size;
}

Matrix makePrefixSum(const Matrix& input) {
  Matrix prefix = Matrix::Zero(input.rows() + 1, input.cols() + 1);// 创建全0矩阵，长宽比input大1
  for (int i = 0; i < input.rows(); ++i) {
    float row_sum = 0.0F;
    for (int j = 0; j < input.cols(); ++j) {
      row_sum += input(i, j);
      // 逐行逐列，遍历每个格子
      // prefix 矩阵里存的是从左上角一直累加到当前格子的总和
      prefix(i + 1, j + 1) = prefix(i, j + 1) + row_sum;
    }
  }
  return prefix;
}

// 求任意一个矩形区域的总和
float getRectSum(const Matrix& prefix, int top, int left, int bottom, int right) {
  return prefix(bottom, right) - prefix(top, right) - prefix(bottom, left) + prefix(top, left);
}

// 计算坡度（中心差分，自适应有效邻居）
//
// 改进说明：
//   原版在邻居无效时用 center 代替，等价于"把坑边当平地"，导致坑壁边缘格子的坡度被低估
//   新版策略：
//     - 两侧邻居均有效 → 标准中心差分（间距 2 格）
//     - 仅一侧有效 → 单侧差分（间距 1 格）
//     - 两侧均无效 → 该方向差分设为 0（只知道自己不知道邻居时，无法估计坡度）
//   地图边界（clamp 后 up==i 等）的处理与原版一致，不影响计算正确性
Matrix computeSlope(const Matrix& elevation, const Matrix& valid_mask, float resolution) {
  const int rows = static_cast<int>(elevation.rows());
  const int cols = static_cast<int>(elevation.cols());
  Matrix slope = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) {
        continue;
      }

      const int up    = std::max(0, i - 1);
      const int down  = std::min(rows - 1, i + 1);
      const int left  = std::max(0, j - 1);
      const int right = std::min(cols - 1, j + 1);

      const float center = elevation(i, j);

      // ---- 行方向差分（dx）：up/down 两侧自适应 ----
      // has_up/has_down：是否有超出 center 的有效邻居（排除 clamp 后与自身重合的情况）
      const bool has_up   = (up != i)   && (valid_mask(up,   j) > 0.5F);
      const bool has_down = (down != i) && (valid_mask(down, j) > 0.5F);
      float dx = 0.0F;
      if (has_up && has_down) {
        // 标准中心差分，间距 2 个格子
        dx = (elevation(down, j) - elevation(up, j)) / (2.0F * resolution);
      } else if (has_down) {
        dx = (elevation(down, j) - center) / resolution;
      } else if (has_up) {
        dx = (center - elevation(up, j)) / resolution;
      }
      // 两侧均无有效邻居 → dx = 0

      // ---- 列方向差分（dy）：left/right 两侧自适应 ----
      const bool has_left  = (left != j)  && (valid_mask(i, left)  > 0.5F);
      const bool has_right = (right != j) && (valid_mask(i, right) > 0.5F);
      float dy = 0.0F;
      if (has_left && has_right) {
        dy = (elevation(i, right) - elevation(i, left)) / (2.0F * resolution);
      } else if (has_right) {
        dy = (elevation(i, right) - center) / resolution;
      } else if (has_left) {
        dy = (center - elevation(i, left)) / resolution;
      }

      slope(i, j) = std::atan(std::sqrt(dx * dx + dy * dy));
    }
  }

  return slope;
}

// 计算局部区域的平均高度
Matrix computeBoxMean(const Matrix& values, const Matrix& valid_mask, int kernel_size) {
  const int rows = static_cast<int>(values.rows());
  const int cols = static_cast<int>(values.cols());
  Matrix sum_input = Matrix::Zero(rows, cols);// 构建全零矩阵
  for (int i = 0; i < rows; ++i) {// 过滤无效数据，无效数据会保持为0
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) > 0.5F) {
        sum_input(i, j) = values(i, j);
      }
    }
  }

  const Matrix sum_prefix = makePrefixSum(sum_input);// 高度数据算一次"前缀和"（为了后面能一秒算出框内总高度）
  const Matrix count_prefix = makePrefixSum(valid_mask);// 对有效掩膜算一次"前缀和"
  const int radius = kernel_size / 2;

  // 准备一个叫 mean 的空矩阵，初始全都填上 NaN
  Matrix mean = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) {
        continue;
      }

      const int top = std::max(0, i - radius);
      const int left = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right = std::min(cols, j + radius + 1);

      // 计算矩形框内有效数据的个数
      const float count = getRectSum(count_prefix, top, left, bottom, right);
      if (count > 0.0F) {
        mean(i, j) = getRectSum(sum_prefix, top, left, bottom, right) / count;// 获得局部平均高度
      }
    }
  }

  return mean;
}

// 计算局部区域的最大落差
Matrix computeLocalRange(const Matrix& values, const Matrix& valid_mask, int kernel_size) {
  const int rows = static_cast<int>(values.rows());
  const int cols = static_cast<int>(values.cols());
  const int radius = kernel_size / 2;
  // 创建一个叫 output 的空矩阵，初始的时候把所有格子填上 NaN
  Matrix output = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) {
        continue;
      }

      const int top = std::max(0, i - radius);
      const int left = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right = std::min(cols, j + radius + 1);

      // 初始化局部最小值和局部最大值为无穷大和无穷小
      float local_min = std::numeric_limits<float>::infinity();
      float local_max = -std::numeric_limits<float>::infinity();
      bool has_sample = false;// 布尔类型标志位，用来记录这个框内到底有没有采到有效点

      for (int r = top; r < bottom; ++r) {
        for (int c = left; c < right; ++c) {
          if (valid_mask(r, c) < 0.5F) {
            continue;
          }
          // // 不断更新局部最小值和最大值
          local_min = std::min(local_min, values(r, c));
          local_max = std::max(local_max, values(r, c));
          has_sample = true;// 只要有一个有效点，标志位置为真
        }
      }

      if (has_sample) {
        output(i, j) = local_max - local_min;
      }
    }
  }

  return output;
}

// 计算局部极值，bool compute_maximum控制找极大还是极小值
Matrix computeLocalExtremum(const Matrix& values, const Matrix& valid_mask, int kernel_size, bool compute_maximum) {
  const int rows = static_cast<int>(values.rows());
  const int cols = static_cast<int>(values.cols());
  const int radius = kernel_size / 2;
  Matrix output = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) {
        continue;
      }

      // 确定滑动窗口的上下左右边界，防止越界
      const int top = std::max(0, i - radius);
      const int left = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right = std::min(cols, j + radius + 1);

      float extremum = compute_maximum ? -std::numeric_limits<float>::infinity()
                                       : std::numeric_limits<float>::infinity();
      bool has_sample = false;

      for (int r = top; r < bottom; ++r) {
        for (int c = left; c < right; ++c) {
          // 只有当这个格子既在 valid_mask 中标记为有效，并且它自身的高度值也是一个正常的有限数值 才继续去计算极值
          if (valid_mask(r, c) < 0.5F || !std::isfinite(values(r, c))) {
            continue;
          }
          extremum = compute_maximum ? std::max(extremum, values(r, c))
                                     : std::min(extremum, values(r, c));
          has_sample = true;
        }
      }

      if (has_sample) {
        output(i, j) = extremum;
      }
    }
  }

  return output;
}

Matrix computeValidRatio(const Matrix& valid_mask, int kernel_size) {
  const int rows = static_cast<int>(valid_mask.rows());
  const int cols = static_cast<int>(valid_mask.cols());
  const int radius = kernel_size / 2;
  const Matrix prefix = makePrefixSum(valid_mask);
  Matrix output = Matrix::Zero(rows, cols);

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const int top = std::max(0, i - radius);
      const int left = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right = std::min(cols, j + radius + 1);
      const float area = static_cast<float>((bottom - top) * (right - left));
      if (area <= 0.0F) {
        continue;
      }
      output(i, j) = getRectSum(prefix, top, left, bottom, right) / area;
    }
  }

  return output;
}

Matrix computeBinaryErosion(const Matrix& binary_mask, int kernel_size) {
  const int rows = static_cast<int>(binary_mask.rows());
  const int cols = static_cast<int>(binary_mask.cols());
  const int anchor = kernel_size / 2;
  const Matrix prefix = makePrefixSum(binary_mask);
  Matrix output = Matrix::Zero(rows, cols);

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const int top = i - anchor;
      const int left = j - anchor;
      const int bottom = top + kernel_size;
      const int right = left + kernel_size;

      if (top < 0 || left < 0 || bottom > rows || right > cols) {
        continue;
      }

      const float sum = getRectSum(prefix, top, left, bottom, right);
      if (sum >= static_cast<float>(kernel_size * kernel_size)) {
        output(i, j) = 1.0F;
      }
    }
  }

  return output;
}

// =============================================================================
// Depression 凹陷度检测：专项识别地面向下的坑洼
// =============================================================================

// Step-1  原始凹陷值：local_background_mean - elevation
//   正值 → 当前点低于周围背景 → 凹陷
//   ≤ 0  → 凸起或与周围持平 → 不是坑
//
// bg_kernel_size：背景均值窗口（应比降落区大，默认对应 depression_bg_radius≈1.0m）
// 之所以用大窗口背景均值而非局部最大值：
//   大窗口均值代表"正常地面高度参考"，对噪声更鲁棒；
//   用 elevation - local_max 会把坑壁顶部误判为凹陷。
Matrix computeDepression(const Matrix& elevation,
                         const Matrix& valid_mask,
                         int bg_kernel_size) {
  const int rows = static_cast<int>(elevation.rows());
  const int cols = static_cast<int>(elevation.cols());

  // 计算大窗口背景均值
  const Matrix bg_mean = computeBoxMean(elevation, valid_mask, bg_kernel_size);

  // depression_raw = bg_mean - elevation  （正 = 凹陷，负 = 凸起）
  Matrix dep_raw = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) > 0.5F && std::isfinite(bg_mean(i, j))) {
        dep_raw(i, j) = bg_mean(i, j) - elevation(i, j);
        // 裁剪负值为 0：凸起对降落无害，凹陷指标只关注向下偏差
        if (dep_raw(i, j) < 0.0F) {
          dep_raw(i, j) = 0.0F;
        }
      }
    }
  }
  return dep_raw;
}

// Step-2  鲁棒归一化（类 Robust Scaler，基于百分位数）
//
// 算法：
//   1. 收集全图所有有效的 depression_raw > 0 的值（排除平地零值，只看真实凹陷分布）
//   2. 计算 P_lo（低百分位，默认P5）和 P_hi（高百分位，默认P95）
//   3. norm = clamp((raw - P_lo) / (P_hi - P_lo + ε), 0, 1)
//
// 优势（为什么不用 MinMax）：
//   MinMax 受单个极深坑或噪声离群点影响大，导致大多数正常地面都被压缩到0附近
//   Robust Scaler 忽略两端5%的极值，让中间80%的凹陷分布线性映射到 [0,1]
//   结果：0.3 阈值在不同地形下含义稳定——"全图凹陷深度排名前70%均视为安全"
//
// 注意：百分位数仅基于 dep_raw > 0 的格子（真实凹陷）计算，
//   平地零值不参与，避免平地主导分布使坑洼分辨率降低。
//   若全图无凹陷（无正值样本），返回全零（全部安全）。
Matrix computeRobustNormalize(const Matrix& dep_raw,
                              const Matrix& valid_mask,
                              float percentile_lo,   // 默认 0.05
                              float percentile_hi) { // 默认 0.95
  const int rows = static_cast<int>(dep_raw.rows());
  const int cols = static_cast<int>(dep_raw.cols());

  // 只收集真实凹陷（dep_raw > 0）参与百分位计算
  // 平地零值不参与，防止平地主导分布压缩坑洼的动态范围
  std::vector<float> samples;
  samples.reserve(static_cast<size_t>(rows * cols));
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) > 0.5F && std::isfinite(dep_raw(i, j))
          && dep_raw(i, j) > 0.0F) {  // ← 只取真实凹陷，排除平地零值
        samples.push_back(dep_raw(i, j));
      }
    }
  }

  // 无真实凹陷或样本不足：全图平坦，归一化结果全零（全部安全）
  if (samples.size() < 5) {
    return Matrix::Zero(rows, cols);
  }

  std::sort(samples.begin(), samples.end());
  const int n = static_cast<int>(samples.size());

  // 百分位插值（线性插值）
  auto getPercentile = [&](float p) -> float {
    const float idx_f = p * static_cast<float>(n - 1);
    const int idx_lo = static_cast<int>(std::floor(idx_f));
    const int idx_hi = std::min(idx_lo + 1, n - 1);
    const float frac = idx_f - static_cast<float>(idx_lo);
    return samples[idx_lo] * (1.0F - frac) + samples[idx_hi] * frac;
  };

  const float p_lo = getPercentile(percentile_lo);
  const float p_hi = getPercentile(percentile_hi);
  const float range = p_hi - p_lo;
  const float eps = 1e-6F;

  // 地图全平（无凹陷分布）时返回全零
  if (range < eps) {
    return Matrix::Zero(rows, cols);
  }

  // 归一化并 clamp 到 [0, 1]
  Matrix dep_norm = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) > 0.5F && std::isfinite(dep_raw(i, j))) {
        const float norm_val = (dep_raw(i, j) - p_lo) / (range + eps);
        dep_norm(i, j) = std::max(0.0F, std::min(1.0F, norm_val));
      }
    }
  }

  ROS_DEBUG("[safeland][depression] Robust normalize: P%.0f=%.4f  P%.0f=%.4f  range=%.4f",
            percentile_lo * 100.0F, p_lo, percentile_hi * 100.0F, p_hi, range);

  return dep_norm;
}

// Step-3  区域凹陷得分：滑窗内归一化凹陷值的最大值
//
// score(i,j) = max{ dep_norm(r,c) | (r,c) 在以(i,j)为中心的 score_kernel 窗口内 }
//
// 语义："以 (i,j) 为降落中心点，落区内最严重的凹陷程度"
// 阈值逻辑：score < dep_score_threshold（默认 0.3）→ 该区域内最深的坑也在全图排名
//           前30%以内 → 判定为平整良好，允许降落
//
// 使用最大值而非均值的原因：
//   均值会被大量平地格子稀释，掩盖落区内的单个深坑
//   最大值保守但安全：只要落区内有一个格子超阈值就拒绝
Matrix computeRegionDepScore(const Matrix& dep_norm,
                             const Matrix& valid_mask,
                             int score_kernel_size) {
  // 复用 computeLocalExtremum，取 maximum（最严重的凹陷）
  return computeLocalExtremum(dep_norm, valid_mask, score_kernel_size, /*compute_maximum=*/true);
}

// =============================================================================

// 计算每个格子的综合安全得分（越高越适合降落），供状态机做加权选点
// score = w_slope*(1-slope/s_t) + w_rough*(1-rough/r_t) + w_step*(1-step/st_t) + w_dep*(1-dep)
// 仅在单点 safe_mat=1（全部四项通过）时有意义
// 权重通过 safeland.yaml 参数传入，支持运行时调参（不再硬编码）
Matrix computeLandingScore(const Matrix& slope_f, const Matrix& rough_f,
                            const Matrix& step_f,  const Matrix& dep_score_f,
                            const Matrix& safe_mat,
                            float s_t, float r_t, float st_t,
                            float w_slope, float w_step, float w_dep, float w_rough) {
  const int rows = static_cast<int>(safe_mat.rows());
  const int cols = static_cast<int>(safe_mat.cols());
  Matrix score = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (safe_mat(i, j) < 0.5F) { continue; }
      if (!std::isfinite(slope_f(i, j)) || !std::isfinite(rough_f(i, j)) ||
          !std::isfinite(step_f(i, j))  || !std::isfinite(dep_score_f(i, j))) { continue; }
      const float slope_score = std::max(0.0F, 1.0F - slope_f(i, j) / (s_t + 1e-6F));
      const float rough_score = std::max(0.0F, 1.0F - rough_f(i, j) / (r_t + 1e-6F));
      const float step_score  = std::max(0.0F, 1.0F - step_f(i, j)  / (st_t + 1e-6F));
      const float dep_score   = std::max(0.0F, 1.0F - dep_score_f(i, j));
      score(i, j) = w_slope * slope_score + w_rough * rough_score
                  + w_step  * step_score  + w_dep   * dep_score;
    }
  }
  return score;
}

// =============================================================================

class SafelandNode {
 public:
  SafelandNode() : nh_(), pnh_("~"), frame_seq_(0) {
    loadParameters();

    output_pub_ = nh_.advertise<grid_map_msgs::GridMap>(output_topic_, 1, true);
    input_sub_ = nh_.subscribe(input_topic_, 1, &SafelandNode::gridMapCallback, this);

    ROS_INFO("[safeland] Subscribed : %s", input_topic_.c_str());
    ROS_INFO("[safeland] Publishing : %s", output_topic_.c_str());
    ROS_INFO("[safeland] Thresholds : slope=%.3f rad, rough=%.3f m, step=%.3f m",
             slope_threshold_, roughness_threshold_, step_threshold_);
    ROS_INFO("[safeland] Landing    : %.1f m x %.1f m", landing_size_, landing_size_);
    ROS_INFO("[safeland] Landing dz : %.3f m", landing_height_range_threshold_);
    ROS_INFO("[safeland] Depression : bg_radius=%.2f m, score_window=%.2f m, threshold=%.2f",
             depression_bg_radius_, depression_score_window_, depression_score_threshold_);
    ROS_INFO("[safeland] Z-filter   : max_valid_z=%.2f m", max_valid_z_);
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("input_grid_map_topic", input_topic_, "/global_grid_map");
    pnh_.param<std::string>("output_grid_map_topic", output_topic_, "/safeland/grid_map");

    pnh_.param("slope_threshold", slope_threshold_, 0.15);
    pnh_.param("roughness_threshold", roughness_threshold_, 0.05);
    pnh_.param("step_threshold", step_threshold_, 0.08);
    pnh_.param("landing_size", landing_size_, 2.0);
    pnh_.param("landing_height_range_threshold", landing_height_range_threshold_, step_threshold_);
    pnh_.param("landing_safety_margin", landing_safety_margin_, 0.3);
    pnh_.param("landing_valid_ratio_threshold", landing_valid_ratio_threshold_, 0.98);
    pnh_.param("min_cell_points", min_cell_points_, 3.0);

    // max_valid_z：高度上限过滤（m）
    //   超过此高度的格子视为悬空点（如树冠、建筑物反射等），不参与地面分析
    //   默认 1.0m：无人机飞行时周围 1m 以上通常不是地面
    //   负值（如 -999）= 不限制上限（室内或已知地形有高处时使用）
    pnh_.param("max_valid_z", max_valid_z_, 1.0);

    pnh_.param("smooth_radius", smooth_radius_, 0.3);
    pnh_.param("step_window", step_window_, 0.5);

    // ---- Depression 凹陷度参数 ----
    // depression_bg_radius：背景均值窗口半径（m），用于估计"正常地面高度"
    //   应大于最大预期坑洼半径，默认 1.0m
    pnh_.param("depression_bg_radius", depression_bg_radius_, 1.0);
    // depression_score_window：区域凹陷得分的滑窗边长（m），对应降落区大小
    //   默认与 landing_size 一致，即评估 1m×1m 落区内的最大归一化凹陷
    pnh_.param("depression_score_window", depression_score_window_, 1.0);
    // depression_score_threshold：区域得分阈值，低于此值认为平整良好
    //   0.3 含义：落区内最严重凹陷在全图凹陷排名前30%（即浅坑）→ 允许降落
    pnh_.param("depression_score_threshold", depression_score_threshold_, 0.3);
    // depression_percentile_lo/hi：鲁棒归一化的低/高百分位数
    pnh_.param("depression_percentile_lo", depression_percentile_lo_, 0.05);
    pnh_.param("depression_percentile_hi", depression_percentile_hi_, 0.95);

    pnh_.param("publish_slope_viz", publish_slope_, true);
    pnh_.param("publish_roughness_viz", publish_roughness_, true);
    pnh_.param("publish_landing_viz", publish_landing_, true);
    // 凹陷各图层独立开关（调试时开启，生产时可关闭 raw/norm 节省带宽）
    pnh_.param("publish_depression_raw_viz", publish_depression_raw_, true);
    pnh_.param("publish_depression_norm_viz", publish_depression_norm_, true);
    pnh_.param("publish_depression_score_viz", publish_depression_score_, true);
    pnh_.param("publish_flatness_score_viz", publish_flatness_score_, true);
    pnh_.param("publish_landing_score_viz", publish_landing_score_, true);

    // ---- landing_score 内部权重（可通过 safeland.yaml 调参）----
    // 默认分配：坡度 35% > 阶梯 30% > 凹陷 25% > 粗糙度 10%
    pnh_.param("landing_score_w_slope", landing_score_w_slope_, 0.35);
    pnh_.param("landing_score_w_step",  landing_score_w_step_,  0.30);
    pnh_.param("landing_score_w_dep",   landing_score_w_dep_,   0.25);
    pnh_.param("landing_score_w_rough", landing_score_w_rough_, 0.10);
  }

  void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
    const ros::WallTime t0 = ros::WallTime::now();

    try {
      grid_map::GridMap map;
      grid_map::GridMapRosConverter::fromMessage(*msg, map);

      if (!map.exists("elevation")) {
        ROS_WARN_THROTTLE(5.0, "[safeland] No elevation layer, skipping.");
        return;
      }

      // 归一化循环缓冲区，保证矩阵索引 == 空间邻接
      map.convertToDefaultStartIndex();

      const auto& elev = map["elevation"];
      const int rows = elev.rows();
      const int cols = elev.cols();

      if (rows < 3 || cols < 3) {
        ROS_WARN_THROTTLE(5.0, "[safeland] Map too small (%dx%d).", rows, cols);
        return;
      }

      // ---- 归一化成连续矩阵，NaN 置零 + 有效掩膜 ----
      Matrix elev_f = Matrix::Zero(rows, cols);
      Matrix valid = Matrix::Zero(rows, cols);
      int valid_count = 0;

      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          const float v = elev(i, j);
          if (std::isfinite(v)) {
            elev_f(i, j) = v;
            valid(i, j) = 1.0F;
            ++valid_count;
          }
        }
      }

      if (valid_count < 9) {
        ROS_WARN_THROTTLE(5.0, "[safeland] Too few valid cells (%d), skipping.", valid_count);
        return;
      }

      Matrix reliable_valid = valid;
      bool has_count_layer = false;
      if (map.exists("count")) {
        has_count_layer = true;
        reliable_valid.setZero();
        const auto& count = map["count"];
        const float min_points = static_cast<float>(min_cell_points_);
        for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
            if (valid(i, j) > 0.5F && std::isfinite(count(i, j)) && count(i, j) >= min_points) {
              reliable_valid(i, j) = 1.0F;
            }
          }
        }
      }

      // ---- 高度上限过滤：屏蔽悬空点（树冠/建筑物/线缆等反射到高程图上的异常高点）----
      // 超过 max_valid_z 的格子不参与地面评估（坡度/粗糙度/凹陷计算均剔除）
      if (max_valid_z_ > -100.0) {  // -999 等负大值表示不限制
        const float z_hi = static_cast<float>(max_valid_z_);
        for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
            if (reliable_valid(i, j) > 0.5F && elev_f(i, j) > z_hi) {
              reliable_valid(i, j) = 0.0F;
            }
          }
        }
      }

      int reliable_count = 0;
      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          if (reliable_valid(i, j) > 0.5F) {
            ++reliable_count;
          }
        }
      }

      const float res = static_cast<float>(map.getResolution());
      if (res <= 0.0F) {
        ROS_WARN_THROTTLE(5.0, "[safeland] Invalid map resolution %.6f.", res);
        return;
      }

      // ---- Slope（中心差分）----
      // 使用 reliable_valid（点数达标的格子）而非 valid，确保坡度来自可信观测
      Matrix slope_f = computeSlope(elev_f, reliable_valid, res);

      // ---- Roughness（|elevation − 局部均值|）----
      const int smooth_k = makeKernelSize(smooth_radius_, res, 3, true);
      const Matrix smooth_f = computeBoxMean(elev_f, reliable_valid, smooth_k);
      Matrix rough_f = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          if (reliable_valid(i, j) > 0.5F && std::isfinite(smooth_f(i, j))) {
            rough_f(i, j) = std::fabs(elev_f(i, j) - smooth_f(i, j));
          }
        }
      }

      // ---- Step（滑窗内 max − min）----
      const int step_k = makeKernelSize(step_window_, res, 3, true);
      const Matrix step_f = computeLocalRange(elev_f, reliable_valid, step_k);

      // ====================================================================
      // ---- Depression 凹陷度三步计算 ----
      // ====================================================================
      //
      // Step-1: 原始凹陷值（m）
      //   depression_raw = max(0, bg_mean - elevation)
      //   正值 = 当前点低于周围背景均值的深度（凹陷深度）
      //   大窗口（bg_radius=1.0m）背景均值代表"无坑时的正常地面高度"
      const int dep_bg_k = makeKernelSize(depression_bg_radius_ * 2.0, res, 5, true);
      const Matrix dep_raw_f = computeDepression(elev_f, reliable_valid, dep_bg_k);

      // Step-2: 鲁棒归一化（Robust Scaler，基于全图 P5~P95 百分位数）
      //   输出 [0,1]：0 = 最浅（平地），1 = 最深（全图最严重坑洼）
      //   阈值 0.3 的语义：该点凹陷深度在全图排名前30%（即浅坑）→ 安全
      const Matrix dep_norm_f = computeRobustNormalize(
          dep_raw_f, reliable_valid,
          static_cast<float>(depression_percentile_lo_),
          static_cast<float>(depression_percentile_hi_));

      // Step-3: 区域凹陷得分（score_window=1m 滑窗内归一化凹陷最大值）
      //   score(i,j) = max{ dep_norm(r,c) | (r,c) 在以(i,j)为中心的 1m×1m 窗口内 }
      //   用最大值：落区内只要有一个格子超阈值就拒绝降落（保守安全）
      const int dep_score_k = makeKernelSize(depression_score_window_, res, 3, true);
      const Matrix dep_score_f = computeRegionDepScore(dep_norm_f, reliable_valid, dep_score_k);
      // ====================================================================

      // ---- 单点安全评估 ----
      const auto s_t = static_cast<float>(slope_threshold_);
      const auto r_t = static_cast<float>(roughness_threshold_);
      const auto st_t = static_cast<float>(step_threshold_);
      const auto dep_t = static_cast<float>(depression_score_threshold_);

      Matrix safe_mat = Matrix::Zero(rows, cols);
      int safe_count = 0;
      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          if (reliable_valid(i, j) > 0.5F &&
              std::isfinite(slope_f(i, j)) &&
              std::isfinite(rough_f(i, j)) &&
              std::isfinite(step_f(i, j)) &&
              std::isfinite(dep_score_f(i, j)) &&
              slope_f(i, j) < s_t &&
              rough_f(i, j) < r_t &&
              step_f(i, j) < st_t &&
              dep_score_f(i, j) < dep_t) {  // ← 新增凹陷约束
            safe_mat(i, j) = 1.0F;
            ++safe_count;
          }
        }
      }

      // ---- 足迹级安全评估：landing_size 落区 + 周边安全裕度 ----
      const int land_k = std::max(1, static_cast<int>(std::ceil(landing_size_ / res)));
      const int landing_eval_k = makeKernelSize(landing_size_ + 2.0 * landing_safety_margin_, res, 3, true);
      const Matrix eroded = computeBinaryErosion(safe_mat, land_k);
      const Matrix landing_range_f = computeLocalRange(elev_f, reliable_valid, landing_eval_k);
      const Matrix landing_max_slope_f = computeLocalExtremum(slope_f, reliable_valid, landing_eval_k, true);
      const Matrix landing_max_rough_f = computeLocalExtremum(rough_f, reliable_valid, landing_eval_k, true);
      const Matrix landing_max_step_f = computeLocalExtremum(step_f, reliable_valid, landing_eval_k, true);
      const Matrix landing_max_dep_score_f = computeLocalExtremum(dep_score_f, reliable_valid, landing_eval_k, true);
      const Matrix landing_valid_ratio_f = computeValidRatio(reliable_valid, landing_eval_k);
      // landing_center_score：落区综合安全得分（仅对通过全部约束的格子赋值）
      // 权重由成员变量传入，支持通过 safeland.yaml 调参
      // 注意：传入 eroded 而非 safe_mat 作为驱动矩阵：
      //   - safe_mat=1 只保证该单格自身通过约束（单点级）
      //   - eroded=1   保证以该格为中心的整个落区均通过腐蚀（足迹级）
      //   使用 eroded 与 landing_center 图层的决策逻辑完全对齐，
      //   避免 landing_score 出现在无法作为降落中心的边缘格子上
      const Matrix landing_score_f = computeLandingScore(
          slope_f, rough_f, step_f, dep_score_f, eroded, s_t, r_t, st_t,
          static_cast<float>(landing_score_w_slope_),
          static_cast<float>(landing_score_w_step_),
          static_cast<float>(landing_score_w_dep_),
          static_cast<float>(landing_score_w_rough_));

      // flatness_score：全图连续平整度参考分 [0,1]。
      // 与 landing_score 不同，它不要求通过 landing_center 的硬约束；
      // 只要格子可靠，就按同一套 slope/step/depression/roughness 指标给连续分数。
      const Matrix flatness_score_f = computeLandingScore(
          slope_f, rough_f, step_f, dep_score_f, reliable_valid, s_t, r_t, st_t,
          static_cast<float>(landing_score_w_slope_),
          static_cast<float>(landing_score_w_step_),
          static_cast<float>(landing_score_w_dep_),
          static_cast<float>(landing_score_w_rough_));

      // ---- 构建输出 grid_map ----
      const float nan = std::numeric_limits<float>::quiet_NaN();
      grid_map::GridMap out_map;
      out_map.setFrameId(map.getFrameId());
      out_map.setGeometry(map.getLength(), map.getResolution(), map.getPosition());
      out_map.setTimestamp(map.getTimestamp());
      // elevation：保留 valid（全量观测），是高程图的原始数据基础
      writeLayer(out_map, "elevation", elev_f, valid, rows, cols, nan);
      out_map.setBasicLayers({"elevation"});
      // step/slope/roughness：与计算掩膜保持一致，均用 reliable_valid
      // 这样可视化图层中只展示"可信观测"区域，与决策依据完全对应
      writeLayer(out_map, "step", step_f, reliable_valid, rows, cols, nan);
      if (publish_slope_) {
        writeLayer(out_map, "slope", slope_f, reliable_valid, rows, cols, nan);
      }
      if (publish_roughness_) {
        writeLayer(out_map, "roughness", rough_f, reliable_valid, rows, cols, nan);
      }

      // ---- 凹陷度图层输出 ----
      if (publish_depression_raw_) {
        // 原始凹陷深度（m）：正值=坑深，0=平地/凸起，可在RViz中直观看出坑洼深度
        writeLayer(out_map, "depression_raw", dep_raw_f, reliable_valid, rows, cols, nan);
      }
      if (publish_depression_norm_) {
        // 归一化凹陷值 [0,1]：全图相对排名，0=最浅，1=最深
        // 用于调参参考：观察阈值0.3对应的绝对深度是否合理
        writeLayer(out_map, "depression_norm", dep_norm_f, reliable_valid, rows, cols, nan);
      }
      if (publish_depression_score_) {
        // 区域凹陷得分 [0,1]：落区（score_window×score_window）内最严重凹陷的归一化值
        // 这是最终用于降落决策的图层，<threshold=0.3 则该区域平整良好
        writeLayer(out_map, "depression_score", dep_score_f, reliable_valid, rows, cols, nan);
      }
      if (publish_flatness_score_) {
        // 全图连续平整度参考分 [0,1]：可靠格子均有值，越高表示越平整。
        // 该图层只供调试/可视化参考，不参与状态机选点。
        writeLayer(out_map, "flatness_score", flatness_score_f, reliable_valid, rows, cols, nan);
      }

      if (publish_landing_) {
        out_map.add("landing_center", 0.0F);
        auto& lc = out_map["landing_center"];

        // landing_score 图层：通过全部约束的格子写入综合安全得分 [0,1]
        // 分数越高越平整、越少凹陷 → 状态机可按「最近且得分最高」选最优落点
        const bool do_score = publish_landing_score_;
        if (do_score) { out_map.add("landing_score", nan); }
        auto* ls_ptr = do_score ? &out_map["landing_score"] : nullptr;
        int landing_count = 0;
        float best_landing_score = -std::numeric_limits<float>::infinity();

        for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
            if (reliable_valid(i, j) < 0.5F) {
              // 仅对可靠栅格做判断（同时也消除悬空高度被过滤的格子）
              lc(i, j) = nan;
              continue;
            }
            // ---- footprint_covered：落区内可靠栅格占比（使用 reliable_valid）----
            // 修正：原先用的是 valid（全部有值格子），现改为 reliable_valid
            // 保证落区内每格都有足够点数，消除坑底/稀疏区域的误判
            const bool footprint_covered =
                landing_valid_ratio_f(i, j) >= static_cast<float>(landing_valid_ratio_threshold_);
            const bool footprint_flat =
                std::isfinite(landing_range_f(i, j)) &&
                landing_range_f(i, j) < static_cast<float>(landing_height_range_threshold_);
            const bool footprint_safe =
                std::isfinite(landing_max_slope_f(i, j)) &&
                std::isfinite(landing_max_rough_f(i, j)) &&
                std::isfinite(landing_max_step_f(i, j)) &&
                landing_max_slope_f(i, j) < s_t &&
                landing_max_rough_f(i, j) < r_t &&
                landing_max_step_f(i, j) < st_t;
            // 落区内凹陷得分约束
            const bool footprint_no_depression =
                std::isfinite(landing_max_dep_score_f(i, j)) &&
                landing_max_dep_score_f(i, j) < dep_t;
            const bool pass = (eroded(i, j) > 0.5F &&
                               footprint_covered &&
                               footprint_flat &&
                               footprint_safe &&
                               footprint_no_depression);
            lc(i, j) = pass ? 1.0F : 0.0F;

            // landing_score：只有通过全部约束的格子才写入综合安全得分
            if (do_score && ls_ptr && pass && std::isfinite(landing_score_f(i, j))) {
              (*ls_ptr)(i, j) = landing_score_f(i, j);
              best_landing_score = std::max(best_landing_score, landing_score_f(i, j));
            }
            if (pass) {
              ++landing_count;
            }
          }
        }

        if (do_score && !std::isfinite(best_landing_score)) {
          best_landing_score = nan;
        }

        const double dt_ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
        ROS_INFO_THROTTLE(5.0,
            "[safeland] #%lu | %dx%d valid=%d reliable=%d safe=%d landing=%d best_score=%.3f | "
            "count=%s min_pts=%.1f | smooth_k=%d step_k=%d land_k=%d eval_k=%d dep_bg_k=%d dep_score_k=%d | %.1f ms",
            frame_seq_ + 1, rows, cols, valid_count, reliable_count, safe_count, landing_count,
            best_landing_score, has_count_layer ? "on" : "off", min_cell_points_,
            smooth_k, step_k, land_k, landing_eval_k, dep_bg_k, dep_score_k, dt_ms);
      } else {
        const double dt_ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
        ROS_INFO_THROTTLE(5.0,
            "[safeland] #%lu | %dx%d valid=%d reliable=%d safe=%d | "
            "count=%s min_pts=%.1f | smooth_k=%d step_k=%d land_k=%d eval_k=%d dep_bg_k=%d dep_score_k=%d | %.1f ms",
            frame_seq_ + 1, rows, cols, valid_count, reliable_count, safe_count,
            has_count_layer ? "on" : "off", min_cell_points_,
            smooth_k, step_k, land_k, landing_eval_k, dep_bg_k, dep_score_k, dt_ms);
      }

      grid_map_msgs::GridMap out_msg;
      grid_map::GridMapRosConverter::toMessage(out_map, out_msg);
      output_pub_.publish(out_msg);

      ++frame_seq_;

    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(3.0, "[safeland] Exception: %s", e.what());
    }
  }

  static void writeLayer(grid_map::GridMap& map, const std::string& name,
                         const Matrix& data, const Matrix& mask,
                          int rows, int cols, float fill) {
    map.add(name, fill);
    auto& layer = map[name];
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        if (mask(i, j) > 0.5F) {
          layer(i, j) = data(i, j);
        }
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;

  std::string input_topic_;
  std::string output_topic_;

  double slope_threshold_;
  double roughness_threshold_;
  double step_threshold_;
  double landing_size_;
  double landing_height_range_threshold_;
  double landing_safety_margin_;
  double landing_valid_ratio_threshold_;
  double min_cell_points_;
  double smooth_radius_;
  double step_window_;

  // ---- Depression 参数 ----
  double depression_bg_radius_;          // 背景均值窗口半径（m）
  double depression_score_window_;       // 区域得分滑窗边长（m）
  double depression_score_threshold_;    // 区域得分阈值，< 此值为平整良好
  double depression_percentile_lo_;      // 鲁棒归一化低百分位（默认0.05）
  double depression_percentile_hi_;      // 鲁棒归一化高百分位（默认0.95）

  double max_valid_z_;                   // 高度上限过滤（m），超过则屏蔽该格

  // ---- landing_score 内部权重（可通过 yaml 调参）----
  double landing_score_w_slope_;
  double landing_score_w_step_;
  double landing_score_w_dep_;
  double landing_score_w_rough_;

  bool publish_slope_;
  bool publish_roughness_;
  bool publish_landing_;
  bool publish_depression_raw_;
  bool publish_depression_norm_;
  bool publish_depression_score_;
  bool publish_flatness_score_;
  bool publish_landing_score_;

  unsigned long frame_seq_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "safeland_node");

  try {
    SafelandNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("[safeland] Fatal: " << e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
