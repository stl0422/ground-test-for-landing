#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

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
  Matrix prefix = Matrix::Zero(input.rows() + 1, input.cols() + 1);
  for (int i = 0; i < input.rows(); ++i) {
    float row_sum = 0.0F;
    for (int j = 0; j < input.cols(); ++j) {
      row_sum += input(i, j);
      prefix(i + 1, j + 1) = prefix(i, j + 1) + row_sum;
    }
  }
  return prefix;
}

float getRectSum(const Matrix& prefix, int top, int left, int bottom, int right) {
  return prefix(bottom, right) - prefix(top, right) - prefix(bottom, left) + prefix(top, left);
}

// 计算坡度（中心差分，自适应有效邻居）
Matrix computeSlope(const Matrix& elevation, const Matrix& valid_mask, float resolution) {
  const int rows = static_cast<int>(elevation.rows());
  const int cols = static_cast<int>(elevation.cols());
  Matrix slope = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) { continue; }

      const int up    = std::max(0, i - 1);
      const int down  = std::min(rows - 1, i + 1);
      const int left  = std::max(0, j - 1);
      const int right = std::min(cols - 1, j + 1);
      const float center = elevation(i, j);

      const bool has_up   = (up != i)   && (valid_mask(up,   j) > 0.5F);
      const bool has_down = (down != i) && (valid_mask(down, j) > 0.5F);
      float dx = 0.0F;
      if (has_up && has_down) {
        dx = (elevation(down, j) - elevation(up, j)) / (2.0F * resolution);
      } else if (has_down) {
        dx = (elevation(down, j) - center) / resolution;
      } else if (has_up) {
        dx = (center - elevation(up, j)) / resolution;
      }

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

Matrix computeBoxMean(const Matrix& values, const Matrix& valid_mask, int kernel_size) {
  const int rows = static_cast<int>(values.rows());
  const int cols = static_cast<int>(values.cols());
  Matrix sum_input = Matrix::Zero(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) > 0.5F) { sum_input(i, j) = values(i, j); }
    }
  }
  const Matrix sum_prefix   = makePrefixSum(sum_input);
  const Matrix count_prefix = makePrefixSum(valid_mask);
  const int radius = kernel_size / 2;
  Matrix mean = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) { continue; }
      const int top    = std::max(0, i - radius);
      const int left   = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right  = std::min(cols, j + radius + 1);
      const float count = getRectSum(count_prefix, top, left, bottom, right);
      if (count > 0.0F) {
        mean(i, j) = getRectSum(sum_prefix, top, left, bottom, right) / count;
      }
    }
  }
  return mean;
}

Matrix computeLocalRange(const Matrix& values, const Matrix& valid_mask, int kernel_size) {
  const int rows = static_cast<int>(values.rows());
  const int cols = static_cast<int>(values.cols());
  const int radius = kernel_size / 2;
  Matrix output = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) { continue; }
      const int top    = std::max(0, i - radius);
      const int left   = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right  = std::min(cols, j + radius + 1);
      float local_min = std::numeric_limits<float>::infinity();
      float local_max = -std::numeric_limits<float>::infinity();
      bool has_sample = false;
      for (int r = top; r < bottom; ++r) {
        for (int c = left; c < right; ++c) {
          if (valid_mask(r, c) < 0.5F) { continue; }
          local_min = std::min(local_min, values(r, c));
          local_max = std::max(local_max, values(r, c));
          has_sample = true;
        }
      }
      if (has_sample) { output(i, j) = local_max - local_min; }
    }
  }
  return output;
}

Matrix computeLocalExtremum(const Matrix& values, const Matrix& valid_mask,
                             int kernel_size, bool compute_maximum) {
  const int rows = static_cast<int>(values.rows());
  const int cols = static_cast<int>(values.cols());
  const int radius = kernel_size / 2;
  Matrix output = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) < 0.5F) { continue; }
      const int top    = std::max(0, i - radius);
      const int left   = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right  = std::min(cols, j + radius + 1);
      float extremum = compute_maximum ? -std::numeric_limits<float>::infinity()
                                       :  std::numeric_limits<float>::infinity();
      bool has_sample = false;
      for (int r = top; r < bottom; ++r) {
        for (int c = left; c < right; ++c) {
          if (valid_mask(r, c) < 0.5F || !std::isfinite(values(r, c))) { continue; }
          extremum = compute_maximum ? std::max(extremum, values(r, c))
                                     : std::min(extremum, values(r, c));
          has_sample = true;
        }
      }
      if (has_sample) { output(i, j) = extremum; }
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
      const int top    = std::max(0, i - radius);
      const int left   = std::max(0, j - radius);
      const int bottom = std::min(rows, i + radius + 1);
      const int right  = std::min(cols, j + radius + 1);
      const float area = static_cast<float>((bottom - top) * (right - left));
      if (area <= 0.0F) { continue; }
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
      const int top    = i - anchor;
      const int left   = j - anchor;
      const int bottom = top + kernel_size;
      const int right  = left + kernel_size;
      if (top < 0 || left < 0 || bottom > rows || right > cols) { continue; }
      const float sum = getRectSum(prefix, top, left, bottom, right);
      if (sum >= static_cast<float>(kernel_size * kernel_size)) {
        output(i, j) = 1.0F;
      }
    }
  }
  return output;
}

// =============================================================================
// Depression 凹陷度检测
// =============================================================================

Matrix computeDepression(const Matrix& elevation, const Matrix& valid_mask, int bg_kernel_size) {
  const int rows = static_cast<int>(elevation.rows());
  const int cols = static_cast<int>(elevation.cols());
  const Matrix bg_mean = computeBoxMean(elevation, valid_mask, bg_kernel_size);
  Matrix dep_raw = Matrix::Constant(rows, cols, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) > 0.5F && std::isfinite(bg_mean(i, j))) {
        dep_raw(i, j) = bg_mean(i, j) - elevation(i, j);
        if (dep_raw(i, j) < 0.0F) { dep_raw(i, j) = 0.0F; }
      }
    }
  }
  return dep_raw;
}

Matrix computeRobustNormalize(const Matrix& dep_raw, const Matrix& valid_mask,
                               float percentile_lo, float percentile_hi) {
  const int rows = static_cast<int>(dep_raw.rows());
  const int cols = static_cast<int>(dep_raw.cols());
  std::vector<float> samples;
  samples.reserve(static_cast<size_t>(rows * cols));
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (valid_mask(i, j) > 0.5F && std::isfinite(dep_raw(i, j)) && dep_raw(i, j) > 0.0F) {
        samples.push_back(dep_raw(i, j));
      }
    }
  }
  if (samples.size() < 5) { return Matrix::Zero(rows, cols); }
  std::sort(samples.begin(), samples.end());
  const int n = static_cast<int>(samples.size());
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
  if (range < eps) { return Matrix::Zero(rows, cols); }
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

Matrix computeRegionDepScore(const Matrix& dep_norm, const Matrix& valid_mask,
                              int score_kernel_size) {
  return computeLocalExtremum(dep_norm, valid_mask, score_kernel_size, /*compute_maximum=*/true);
}

// =============================================================================
// 综合安全得分
// =============================================================================

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
// 落点稳定性：滑动中值滤波器（抑制帧间抖动）
// =============================================================================
//
// 问题：每帧独立选点时，相邻帧的「最优落点」可能因点云轻微变化而跳动几十厘米，
//       导致下游状态机频繁修改目标点，引发无人机悬停抖动。
//
// 方案：对连续 N 帧的最优落点 XYZ 各维度分别取中值，输出稳定的参考点。
//       中值对离群帧的抵抗力强（一帧跳动不影响中值，需超过半数帧才改变输出）。
//
// 参数（safeland.yaml）：
//   best_point_median_window : 滑动窗口帧数（1=关闭，3~7=推荐）
//   best_point_jump_guard_m  : 跳变门限（m），超过此距离直接接受新落点
struct PointMedianFilter {
  explicit PointMedianFilter(int window, float jump_guard)
      : window_(std::max(1, window)), jump_guard_(jump_guard),
        last_pub_x_(std::numeric_limits<float>::quiet_NaN()),
        last_pub_y_(std::numeric_limits<float>::quiet_NaN()),
        last_pub_z_(std::numeric_limits<float>::quiet_NaN()) {}

  // 推入新落点，返回滤波后稳定落点；返回值表示落点是否有实质变化
  bool push(float x, float y, float z, float& out_x, float& out_y, float& out_z) {
    hist_x_.push_back(x);
    hist_y_.push_back(y);
    hist_z_.push_back(z);
    if (static_cast<int>(hist_x_.size()) > window_) {
      hist_x_.pop_front(); hist_y_.pop_front(); hist_z_.pop_front();
    }
    const float mx = median(hist_x_);
    const float my = median(hist_y_);
    const float mz = median(hist_z_);

    const bool first_pub = !std::isfinite(last_pub_x_);
    const float dist = first_pub ? 0.0F :
        std::sqrt((mx - last_pub_x_) * (mx - last_pub_x_) +
                  (my - last_pub_y_) * (my - last_pub_y_));

    if (!first_pub && dist < 1e-4F) {
      out_x = last_pub_x_; out_y = last_pub_y_; out_z = last_pub_z_;
      return false;
    }
    if (!first_pub && jump_guard_ > 0.0F && dist > jump_guard_) {
      ROS_DEBUG("[safeland][median] Large jump: %.3f m (guard=%.3f). Accepting.", dist, jump_guard_);
    }
    last_pub_x_ = mx; last_pub_y_ = my; last_pub_z_ = mz;
    out_x = mx; out_y = my; out_z = mz;
    return true;
  }

  void reset() {
    hist_x_.clear(); hist_y_.clear(); hist_z_.clear();
    last_pub_x_ = std::numeric_limits<float>::quiet_NaN();
    last_pub_y_ = std::numeric_limits<float>::quiet_NaN();
    last_pub_z_ = std::numeric_limits<float>::quiet_NaN();
  }

 private:
  static float median(const std::deque<float>& data) {
    std::vector<float> sorted(data.begin(), data.end());
    std::sort(sorted.begin(), sorted.end());
    const int n = static_cast<int>(sorted.size());
    if (n == 0) { return std::numeric_limits<float>::quiet_NaN(); }
    if (n % 2 == 1) { return sorted[n / 2]; }
    return (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5F;
  }

  int window_;
  float jump_guard_;
  std::deque<float> hist_x_, hist_y_, hist_z_;
  float last_pub_x_, last_pub_y_, last_pub_z_;
};

// =============================================================================
// 备降区搜索辅助结构体
// =============================================================================

struct LandingCandidate {
  int row;
  int col;
  float score;
  float dist_sq;
  bool is_primary;
};

std::vector<LandingCandidate> searchAlternateLandingZone(
    const Matrix& slope_f, const Matrix& rough_f,
    const Matrix& step_f, const Matrix& dep_score_f,
    const Matrix& eroded_alt,
    const Matrix& landing_range_f, const Matrix& landing_max_slope_f,
    const Matrix& landing_max_rough_f, const Matrix& landing_max_step_f,
    const Matrix& landing_max_dep_score_f, const Matrix& landing_valid_ratio_f,
    const Matrix& reliable_valid,
    int rows, int cols, int center_row, int center_col,
    float s_t_rel, float r_t_rel, float st_t_rel, float dep_t_rel,
    float height_range_t_rel, float valid_ratio_t_rel,
    float w_slope, float w_step, float w_dep, float w_rough,
    int max_candidates = 10) {
  std::vector<LandingCandidate> candidates;
  candidates.reserve(32);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      if (reliable_valid(i, j) < 0.5F) { continue; }
      if (eroded_alt(i, j) < 0.5F) { continue; }
      const bool footprint_covered = landing_valid_ratio_f(i, j) >= valid_ratio_t_rel;
      const bool footprint_flat = std::isfinite(landing_range_f(i, j)) &&
                                  landing_range_f(i, j) < height_range_t_rel;
      const bool footprint_safe = std::isfinite(landing_max_slope_f(i, j)) &&
                                  std::isfinite(landing_max_rough_f(i, j)) &&
                                  std::isfinite(landing_max_step_f(i, j)) &&
                                  landing_max_slope_f(i, j) < s_t_rel &&
                                  landing_max_rough_f(i, j) < r_t_rel &&
                                  landing_max_step_f(i, j) < st_t_rel;
      const bool footprint_dep = std::isfinite(landing_max_dep_score_f(i, j)) &&
                                 landing_max_dep_score_f(i, j) < dep_t_rel;
      if (!(footprint_covered && footprint_flat && footprint_safe && footprint_dep)) { continue; }
      float slope_sc = std::max(0.0F, 1.0F - slope_f(i, j) / (s_t_rel + 1e-6F));
      float rough_sc = std::max(0.0F, 1.0F - rough_f(i, j) / (r_t_rel + 1e-6F));
      float step_sc  = std::max(0.0F, 1.0F - step_f(i, j)  / (st_t_rel + 1e-6F));
      float dep_sc   = std::max(0.0F, 1.0F - dep_score_f(i, j));
      if (!std::isfinite(slope_sc) || !std::isfinite(rough_sc) ||
          !std::isfinite(step_sc)  || !std::isfinite(dep_sc)) { continue; }
      float score = w_slope * slope_sc + w_rough * rough_sc + w_step * step_sc + w_dep * dep_sc;
      float dr = static_cast<float>(i - center_row);
      float dc = static_cast<float>(j - center_col);
      candidates.push_back({i, j, score, dr * dr + dc * dc, false});
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const LandingCandidate& a, const LandingCandidate& b) {
              if (a.score != b.score) { return a.score > b.score; }
              return a.dist_sq < b.dist_sq;
            });
  if (static_cast<int>(candidates.size()) > max_candidates) {
    candidates.resize(static_cast<size_t>(max_candidates));
  }
  return candidates;
}

// =============================================================================
// 试验数据记录器
// =============================================================================

void appendMetricsToCSV(const std::string& csv_path,
                         unsigned long frame_seq,
                         int rows, int cols,
                         int valid_count, int reliable_count,
                         int safe_count, int landing_count,
                         float best_score, float resolution,
                         float slope_t, float rough_t, float step_t, float dep_t,
                         float landing_size,
                         int alt_landing_count, float alt_best_score,
                         double dt_ms) {
  if (csv_path.empty()) { return; }
  bool write_header = false;
  {
    std::ifstream check(csv_path);
    write_header = !check.good() || check.peek() == std::ifstream::traits_type::eof();
  }
  std::ofstream ofs(csv_path, std::ios::app);
  if (!ofs.is_open()) { return; }
  if (write_header) {
    ofs << "frame_seq,timestamp_s,rows,cols,valid_count,reliable_count,"
           "safe_count,landing_count,best_score,"
           "alt_landing_count,alt_best_score,"
           "resolution,slope_threshold,roughness_threshold,step_threshold,"
           "depression_threshold,landing_size,dt_ms\n";
  }
  auto ts = static_cast<double>(ros::Time::now().toSec());
  ofs << frame_seq << ","
      << std::fixed << std::setprecision(3) << ts << ","
      << rows << "," << cols << ","
      << valid_count << "," << reliable_count << ","
      << safe_count << "," << landing_count << ","
      << (std::isfinite(best_score) ? best_score : -1.0F) << ","
      << alt_landing_count << ","
      << (std::isfinite(alt_best_score) ? alt_best_score : -1.0F) << ","
      << resolution << ","
      << slope_t << "," << rough_t << "," << step_t << "," << dep_t << ","
      << landing_size << ","
      << std::setprecision(2) << dt_ms << "\n";
  ofs.flush();
}

// =============================================================================

class SafelandNode {
 public:
  SafelandNode()
      : nh_(), pnh_("~"), frame_seq_(0),
        best_point_filter_(1, 0.5F) {
    loadParameters();
    // 用加载好的参数重新初始化滤波器
    best_point_filter_ = PointMedianFilter(
        best_point_median_window_,
        static_cast<float>(best_point_jump_guard_m_));

    output_pub_ = nh_.advertise<grid_map_msgs::GridMap>(output_topic_, 1, true);
    input_sub_  = nh_.subscribe(input_topic_, 1, &SafelandNode::gridMapCallback, this);

    // 备降区状态话题
    alt_status_pub_  = nh_.advertise<std_msgs::String>("/safeland/alt_landing_status", 1, true);
    best_point_pub_  = nh_.advertise<geometry_msgs::PointStamped>("/safeland/best_landing_point", 1, true);

    ROS_INFO("[safeland] Subscribed : %s", input_topic_.c_str());
    ROS_INFO("[safeland] Publishing : %s", output_topic_.c_str());
    ROS_INFO("[safeland] Thresholds : slope=%.3f rad, rough=%.3f m, step=%.3f m",
             slope_threshold_, roughness_threshold_, step_threshold_);
    ROS_INFO("[safeland] Landing    : %.1f m x %.1f m", landing_size_, landing_size_);
    ROS_INFO("[safeland] Landing dz : %.3f m", landing_height_range_threshold_);
    ROS_INFO("[safeland] Depression : bg_radius=%.2f m, score_window=%.2f m, threshold=%.2f",
             depression_bg_radius_, depression_score_window_, depression_score_threshold_);
    ROS_INFO("[safeland] Z-filter   : max_valid_z=%.2f m", max_valid_z_);
    ROS_INFO("[safeland] AltLanding : %s, relax=%.2f (max=%.2f)",
             enable_alt_landing_ ? "ENABLED" : "DISABLED",
             alt_landing_relax_factor_, alt_landing_max_relax_);
    ROS_INFO("[safeland] BestPoint  : score_w=%.2f, median_win=%d, jump_guard=%.2fm",
             best_point_score_weight_, best_point_median_window_, best_point_jump_guard_m_);
    if (!metrics_csv_path_.empty()) {
      ROS_INFO("[safeland] Metrics CSV: %s (every %d frames)",
               metrics_csv_path_.c_str(), metrics_record_every_n_);
    }
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("input_grid_map_topic",  input_topic_,  "/global_grid_map");
    pnh_.param<std::string>("output_grid_map_topic", output_topic_, "/safeland/grid_map");

    pnh_.param("slope_threshold",               slope_threshold_,               0.15);
    pnh_.param("roughness_threshold",           roughness_threshold_,           0.05);
    pnh_.param("step_threshold",               step_threshold_,                0.08);
    pnh_.param("landing_size",                 landing_size_,                  2.0);
    pnh_.param("landing_height_range_threshold", landing_height_range_threshold_, step_threshold_);
    pnh_.param("landing_safety_margin",         landing_safety_margin_,         0.3);
    pnh_.param("landing_valid_ratio_threshold", landing_valid_ratio_threshold_, 0.98);
    pnh_.param("min_cell_points",              min_cell_points_,               3.0);
    pnh_.param("max_valid_z",                  max_valid_z_,                   1.0);
    pnh_.param("smooth_radius",                smooth_radius_,                 0.3);
    pnh_.param("step_window",                  step_window_,                   0.5);

    // ---- Depression 凹陷度参数 ----
    pnh_.param("depression_bg_radius",         depression_bg_radius_,          1.0);
    pnh_.param("depression_score_window",      depression_score_window_,       1.0);
    pnh_.param("depression_score_threshold",   depression_score_threshold_,    0.3);
    pnh_.param("depression_percentile_lo",     depression_percentile_lo_,      0.05);
    pnh_.param("depression_percentile_hi",     depression_percentile_hi_,      0.95);

    // ---- 可视化开关 ----
    pnh_.param("publish_slope_viz",            publish_slope_,            true);
    pnh_.param("publish_roughness_viz",        publish_roughness_,        true);
    pnh_.param("publish_landing_viz",          publish_landing_,          true);
    pnh_.param("publish_depression_raw_viz",   publish_depression_raw_,   true);
    pnh_.param("publish_depression_norm_viz",  publish_depression_norm_,  true);
    pnh_.param("publish_depression_score_viz", publish_depression_score_, true);
    pnh_.param("publish_flatness_score_viz",   publish_flatness_score_,   true);
    pnh_.param("publish_landing_score_viz",    publish_landing_score_,    true);

    // ---- landing_score 内部权重 ----
    pnh_.param("landing_score_w_slope", landing_score_w_slope_, 0.35);
    pnh_.param("landing_score_w_step",  landing_score_w_step_,  0.30);
    pnh_.param("landing_score_w_dep",   landing_score_w_dep_,   0.25);
    pnh_.param("landing_score_w_rough", landing_score_w_rough_, 0.10);

    // ---- 最优落点选择策略 ----
    // composite = score_weight * norm_score + (1-score_weight) * norm_dist_inv
    pnh_.param("best_point_score_weight", best_point_score_weight_, 0.7);

    // ---- 落点稳定性（中值滤波）----
    pnh_.param("best_point_median_window", best_point_median_window_, 5);
    pnh_.param("best_point_jump_guard_m",  best_point_jump_guard_m_,  1.5);

    // ---- 备降区参数 ----
    pnh_.param("alt_landing_relax_factor", alt_landing_relax_factor_, 1.5);
    pnh_.param("alt_landing_max_relax",    alt_landing_max_relax_,    2.0);
    pnh_.param("enable_alt_landing",       enable_alt_landing_,       true);
    pnh_.param("publish_alt_landing_viz",  publish_alt_landing_,      true);

    // ---- 数据记录 ----
    pnh_.param<std::string>("metrics_csv_path",        metrics_csv_path_,       "");
    pnh_.param("metrics_record_every_n_frames",         metrics_record_every_n_, 1);
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
      map.convertToDefaultStartIndex();

      const auto& elev = map["elevation"];
      const int rows = elev.rows();
      const int cols = elev.cols();
      if (rows < 3 || cols < 3) {
        ROS_WARN_THROTTLE(5.0, "[safeland] Map too small (%dx%d).", rows, cols);
        return;
      }

      // ---- 有效掩膜 ----
      Matrix elev_f = Matrix::Zero(rows, cols);
      Matrix valid  = Matrix::Zero(rows, cols);
      int valid_count = 0;
      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          const float v = elev(i, j);
          if (std::isfinite(v)) {
            elev_f(i, j) = v; valid(i, j) = 1.0F; ++valid_count;
          }
        }
      }
      if (valid_count < 9) {
        ROS_WARN_THROTTLE(5.0, "[safeland] Too few valid cells (%d), skipping.", valid_count);
        return;
      }

      // ---- 可靠栅格掩膜（点数达标）----
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

      // ---- 高度上限过滤 ----
      if (max_valid_z_ > -100.0) {
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
          if (reliable_valid(i, j) > 0.5F) { ++reliable_count; }
        }
      }

      const float res = static_cast<float>(map.getResolution());
      if (res <= 0.0F) {
        ROS_WARN_THROTTLE(5.0, "[safeland] Invalid resolution %.6f.", res);
        return;
      }

      // ---- Slope ----
      Matrix slope_f = computeSlope(elev_f, reliable_valid, res);

      // ---- Roughness ----
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

      // ---- Step ----
      const int step_k = makeKernelSize(step_window_, res, 3, true);
      const Matrix step_f = computeLocalRange(elev_f, reliable_valid, step_k);

      // ---- Depression 三步计算 ----
      const int dep_bg_k = makeKernelSize(depression_bg_radius_ * 2.0, res, 5, true);
      const Matrix dep_raw_f = computeDepression(elev_f, reliable_valid, dep_bg_k);
      const Matrix dep_norm_f = computeRobustNormalize(
          dep_raw_f, reliable_valid,
          static_cast<float>(depression_percentile_lo_),
          static_cast<float>(depression_percentile_hi_));
      const int dep_score_k = makeKernelSize(depression_score_window_, res, 3, true);
      const Matrix dep_score_f = computeRegionDepScore(dep_norm_f, reliable_valid, dep_score_k);

      // ---- 单点安全评估 ----
      const auto s_t   = static_cast<float>(slope_threshold_);
      const auto r_t   = static_cast<float>(roughness_threshold_);
      const auto st_t  = static_cast<float>(step_threshold_);
      const auto dep_t = static_cast<float>(depression_score_threshold_);

      Matrix safe_mat = Matrix::Zero(rows, cols);
      int safe_count = 0;
      for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          if (reliable_valid(i, j) > 0.5F &&
              std::isfinite(slope_f(i, j)) && std::isfinite(rough_f(i, j)) &&
              std::isfinite(step_f(i, j))  && std::isfinite(dep_score_f(i, j)) &&
              slope_f(i, j) < s_t && rough_f(i, j) < r_t &&
              step_f(i, j)  < st_t && dep_score_f(i, j) < dep_t) {
            safe_mat(i, j) = 1.0F; ++safe_count;
          }
        }
      }

      // ---- 足迹级安全评估 ----
      const int land_k = std::max(1, static_cast<int>(std::ceil(landing_size_ / res)));
      const int landing_eval_k = makeKernelSize(
          landing_size_ + 2.0 * landing_safety_margin_, res, 3, true);
      const Matrix eroded              = computeBinaryErosion(safe_mat, land_k);
      const Matrix landing_range_f     = computeLocalRange(elev_f, reliable_valid, landing_eval_k);
      const Matrix landing_max_slope_f = computeLocalExtremum(slope_f,     reliable_valid, landing_eval_k, true);
      const Matrix landing_max_rough_f = computeLocalExtremum(rough_f,     reliable_valid, landing_eval_k, true);
      const Matrix landing_max_step_f  = computeLocalExtremum(step_f,      reliable_valid, landing_eval_k, true);
      const Matrix landing_max_dep_f   = computeLocalExtremum(dep_score_f, reliable_valid, landing_eval_k, true);
      const Matrix landing_valid_ratio_f = computeValidRatio(reliable_valid, landing_eval_k);
      const Matrix landing_score_f = computeLandingScore(
          slope_f, rough_f, step_f, dep_score_f, eroded, s_t, r_t, st_t,
          static_cast<float>(landing_score_w_slope_), static_cast<float>(landing_score_w_step_),
          static_cast<float>(landing_score_w_dep_),   static_cast<float>(landing_score_w_rough_));
      const Matrix flatness_score_f = computeLandingScore(
          slope_f, rough_f, step_f, dep_score_f, reliable_valid, s_t, r_t, st_t,
          static_cast<float>(landing_score_w_slope_), static_cast<float>(landing_score_w_step_),
          static_cast<float>(landing_score_w_dep_),   static_cast<float>(landing_score_w_rough_));

      // ---- 构建输出 grid_map ----
      const float nan = std::numeric_limits<float>::quiet_NaN();
      grid_map::GridMap out_map;
      out_map.setFrameId(map.getFrameId());
      out_map.setGeometry(map.getLength(), map.getResolution(), map.getPosition());
      out_map.setTimestamp(map.getTimestamp());
      writeLayer(out_map, "elevation", elev_f, valid,          rows, cols, nan);
      out_map.setBasicLayers({"elevation"});
      writeLayer(out_map, "step",      step_f, reliable_valid, rows, cols, nan);
      if (publish_slope_)     { writeLayer(out_map, "slope",     slope_f,         reliable_valid, rows, cols, nan); }
      if (publish_roughness_) { writeLayer(out_map, "roughness", rough_f,         reliable_valid, rows, cols, nan); }
      if (publish_depression_raw_)   { writeLayer(out_map, "depression_raw",   dep_raw_f,     reliable_valid, rows, cols, nan); }
      if (publish_depression_norm_)  { writeLayer(out_map, "depression_norm",  dep_norm_f,    reliable_valid, rows, cols, nan); }
      if (publish_depression_score_) { writeLayer(out_map, "depression_score", dep_score_f,   reliable_valid, rows, cols, nan); }
      if (publish_flatness_score_)   { writeLayer(out_map, "flatness_score",   flatness_score_f, reliable_valid, rows, cols, nan); }

      // ---- 主降落区评估 ----
      int   landing_count      = 0;
      float best_landing_score = -std::numeric_limits<float>::infinity();
      const bool do_score = publish_landing_score_;

      if (publish_landing_) {
        out_map.add("landing_center", 0.0F);
        auto& lc = out_map["landing_center"];
        if (do_score) { out_map.add("landing_score", nan); }
        auto* ls_ptr = do_score ? &out_map["landing_score"] : nullptr;

        for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
            if (reliable_valid(i, j) < 0.5F) { lc(i, j) = nan; continue; }
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
            const bool footprint_dep =
                std::isfinite(landing_max_dep_f(i, j)) &&
                landing_max_dep_f(i, j) < dep_t;
            const bool pass = (eroded(i, j) > 0.5F &&
                               footprint_covered && footprint_flat &&
                               footprint_safe && footprint_dep);
            lc(i, j) = pass ? 1.0F : 0.0F;
            if (do_score && ls_ptr && pass && std::isfinite(landing_score_f(i, j))) {
              (*ls_ptr)(i, j) = landing_score_f(i, j);
              best_landing_score = std::max(best_landing_score, landing_score_f(i, j));
            }
            if (pass) { ++landing_count; }
          }
        }
        if (do_score && !std::isfinite(best_landing_score)) { best_landing_score = nan; }
      }

      // ======================================================================
      // ---- 主/备降区决策 + 最优落点发布 ----
      // ======================================================================
      int   alt_landing_count = 0;
      float alt_best_score    = nan;
      std::string alt_status  = "NONE";

      // 中心参考点（地图中心格子）
      const int cr = rows / 2;
      const int cc = cols / 2;

      if (landing_count > 0 && publish_landing_) {
        alt_status = "PRIMARY_OK";

        // 最优落点综合打分：score_weight * norm_score + (1-score_weight) * norm_dist_inv
        // 在安全性相当的候选点中优先选择离地图中心近的落点，减少无人机飞行距离
        const auto& lc_layer  = out_map["landing_center"];
        const auto* ls_layer  = do_score ? &out_map["landing_score"] : nullptr;
        const float w_score   = static_cast<float>(best_point_score_weight_);

        float max_score_val = 0.0F;
        float max_dist_sq   = 0.0F;
        for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
            if (lc_layer(i, j) < 0.5F) { continue; }
            float s = (ls_layer && std::isfinite((*ls_layer)(i, j))) ? (*ls_layer)(i, j) : 0.0F;
            float dr = static_cast<float>(i - cr);
            float dc = static_cast<float>(j - cc);
            max_score_val = std::max(max_score_val, s);
            max_dist_sq   = std::max(max_dist_sq, dr * dr + dc * dc);
          }
        }
        int   best_r = -1, best_c = -1;
        float best_composite = -std::numeric_limits<float>::infinity();
        for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
            if (lc_layer(i, j) < 0.5F) { continue; }
            float s = (ls_layer && std::isfinite((*ls_layer)(i, j))) ? (*ls_layer)(i, j) : 0.0F;
            float dr = static_cast<float>(i - cr);
            float dc = static_cast<float>(j - cc);
            float dist_sq = dr * dr + dc * dc;
            float norm_score    = (max_score_val > 1e-6F) ? s / max_score_val : 0.0F;
            float norm_dist_inv = (max_dist_sq   > 1e-6F) ?
                1.0F - std::sqrt(dist_sq / max_dist_sq) : 1.0F;
            float composite = w_score * norm_score + (1.0F - w_score) * norm_dist_inv;
            if (composite > best_composite) { best_composite = composite; best_r = i; best_c = j; }
          }
        }
        if (best_r >= 0) {
          grid_map::Index best_idx(best_r, best_c);
          grid_map::Position best_pos;
          if (out_map.getPosition(best_idx, best_pos)) {
            float fx, fy, fz;
            if (best_point_filter_.push(static_cast<float>(best_pos.x()),
                                        static_cast<float>(best_pos.y()),
                                        elev_f(best_r, best_c), fx, fy, fz)) {
              geometry_msgs::PointStamped pt_msg;
              pt_msg.header.stamp    = ros::Time::now();
              pt_msg.header.frame_id = out_map.getFrameId();
              pt_msg.point.x = fx; pt_msg.point.y = fy; pt_msg.point.z = fz;
              best_point_pub_.publish(pt_msg);
            }
          }
        }
      } else if (enable_alt_landing_ && publish_landing_) {
        // ---- 备降区：用放宽阈值重新搜索 ----
        const float relax = static_cast<float>(
            std::min(alt_landing_relax_factor_, alt_landing_max_relax_));
        const float s_t_rel   = s_t * relax;
        const float r_t_rel   = r_t * relax;
        const float st_t_rel  = st_t * relax;
        const float dep_t_rel = std::min(dep_t * relax, 1.0F);
        const float ht_t_rel  = static_cast<float>(landing_height_range_threshold_) * relax;
        const float vr_t_rel  = std::max(0.85F,
            static_cast<float>(landing_valid_ratio_threshold_) / relax);

        Matrix safe_alt = Matrix::Zero(rows, cols);
        for (int i = 0; i < rows; ++i) {
          for (int j = 0; j < cols; ++j) {
            if (reliable_valid(i, j) > 0.5F &&
                std::isfinite(slope_f(i, j)) && std::isfinite(rough_f(i, j)) &&
                std::isfinite(step_f(i, j))  && std::isfinite(dep_score_f(i, j)) &&
                slope_f(i, j)     < s_t_rel &&
                rough_f(i, j)     < r_t_rel &&
                step_f(i, j)      < st_t_rel &&
                dep_score_f(i, j) < dep_t_rel) {
              safe_alt(i, j) = 1.0F;
            }
          }
        }
        const Matrix eroded_alt = computeBinaryErosion(safe_alt, land_k);

        const auto alt_candidates = searchAlternateLandingZone(
            slope_f, rough_f, step_f, dep_score_f, eroded_alt,
            landing_range_f, landing_max_slope_f, landing_max_rough_f,
            landing_max_step_f, landing_max_dep_f, landing_valid_ratio_f,
            reliable_valid, rows, cols, cr, cc,
            s_t_rel, r_t_rel, st_t_rel, dep_t_rel, ht_t_rel, vr_t_rel,
            static_cast<float>(landing_score_w_slope_), static_cast<float>(landing_score_w_step_),
            static_cast<float>(landing_score_w_dep_),   static_cast<float>(landing_score_w_rough_));

        alt_landing_count = static_cast<int>(alt_candidates.size());

        if (alt_landing_count > 0) {
          alt_status    = "ALT_FOUND";
          alt_best_score = alt_candidates[0].score;
          ROS_WARN_THROTTLE(3.0,
              "[safeland] #%lu | No primary zone! ALT_FOUND: %d candidates, "
              "best_score=%.3f, relax=%.2f (slope<%.3f step<%.3f dep<%.3f)",
              frame_seq_ + 1, alt_landing_count, alt_best_score, relax,
              s_t_rel, st_t_rel, dep_t_rel);
        } else {
          alt_status = "ALL_FAILED";
          ROS_ERROR_THROTTLE(3.0,
              "[safeland] #%lu | ALL_FAILED: No landing zone even with relax=%.2f! "
              "safe_count=%d reliable=%d",
              frame_seq_ + 1, relax, safe_count, reliable_count);
        }

        if (publish_alt_landing_) {
          out_map.add("alt_landing_center", 0.0F);
          auto& alc = out_map["alt_landing_center"];
          for (const auto& cand : alt_candidates) {
            alc(cand.row, cand.col) = 1.0F;
          }
        }

        if (!alt_candidates.empty()) {
          grid_map::Index best_idx(alt_candidates[0].row, alt_candidates[0].col);
          grid_map::Position best_pos;
          if (out_map.getPosition(best_idx, best_pos)) {
            float fx, fy, fz;
            if (best_point_filter_.push(static_cast<float>(best_pos.x()),
                                        static_cast<float>(best_pos.y()),
                                        elev_f(alt_candidates[0].row, alt_candidates[0].col),
                                        fx, fy, fz)) {
              geometry_msgs::PointStamped pt_msg;
              pt_msg.header.stamp    = ros::Time::now();
              pt_msg.header.frame_id = out_map.getFrameId();
              pt_msg.point.x = fx; pt_msg.point.y = fy; pt_msg.point.z = fz;
              best_point_pub_.publish(pt_msg);
            }
          }
        }
      }

      // 发布备降区状态
      {
        std_msgs::String status_msg;
        status_msg.data = alt_status;
        alt_status_pub_.publish(status_msg);
      }

      // ======================================================================
      // ---- 数据记录（CSV）----
      // ======================================================================
      if (!metrics_csv_path_.empty() &&
          metrics_record_every_n_ > 0 &&
          (frame_seq_ % static_cast<unsigned long>(metrics_record_every_n_) == 0)) {
        appendMetricsToCSV(
            metrics_csv_path_, frame_seq_ + 1,
            rows, cols, valid_count, reliable_count,
            safe_count, landing_count, best_landing_score, res,
            s_t, r_t, st_t, dep_t, static_cast<float>(landing_size_),
            alt_landing_count, alt_best_score,
            (ros::WallTime::now() - t0).toSec() * 1000.0);
      }

      const double dt_ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
      if (publish_landing_) {
        ROS_INFO_THROTTLE(5.0,
            "[safeland] #%lu | %dx%d valid=%d reliable=%d safe=%d landing=%d best_score=%.3f | "
            "alt=%s alt_count=%d | "
            "count=%s min_pts=%.1f | smooth_k=%d step_k=%d land_k=%d eval_k=%d dep_bg_k=%d dep_score_k=%d | %.1f ms",
            frame_seq_ + 1, rows, cols, valid_count, reliable_count, safe_count, landing_count,
            std::isfinite(best_landing_score) ? best_landing_score : -1.0F,
            alt_status.c_str(), alt_landing_count,
            has_count_layer ? "on" : "off", min_cell_points_,
            smooth_k, step_k, land_k, landing_eval_k, dep_bg_k, dep_score_k, dt_ms);
      } else {
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
      ROS_ERROR_THROTTLE(3.0, "[safeland] Exception in callback: %s", e.what());
    }
  }

  // 辅助：写一层 grid_map（将 Matrix 值写入，无效格子填 NaN）
  static void writeLayer(grid_map::GridMap& out_map,
                         const std::string& layer_name,
                         const Matrix& data,
                         const Matrix& valid_mask,
                         int rows, int cols,
                         float nan_val) {
    out_map.add(layer_name, nan_val);
    auto& layer = out_map[layer_name];
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        if (valid_mask(i, j) > 0.5F && std::isfinite(data(i, j))) {
          layer(i, j) = data(i, j);
        }
      }
    }
  }

  // ---- ROS 句柄 ----
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber input_sub_;
  ros::Publisher  output_pub_;
  ros::Publisher  alt_status_pub_;
  ros::Publisher  best_point_pub_;

  // ---- 帧计数 ----
  unsigned long frame_seq_;

  // ---- 落点稳定性滤波器 ----
  PointMedianFilter best_point_filter_;

  // ---- 话题参数 ----
  std::string input_topic_;
  std::string output_topic_;

  // ---- 安全评估阈值 ----
  double slope_threshold_;
  double roughness_threshold_;
  double step_threshold_;
  double max_valid_z_;

  // ---- 足迹参数 ----
  double landing_size_;
  double landing_height_range_threshold_;
  double landing_safety_margin_;
  double landing_valid_ratio_threshold_;

  // ---- 点云过滤 ----
  double min_cell_points_;

  // ---- 平滑/步骤窗口 ----
  double smooth_radius_;
  double step_window_;

  // ---- Depression 参数 ----
  double depression_bg_radius_;
  double depression_score_window_;
  double depression_score_threshold_;
  double depression_percentile_lo_;
  double depression_percentile_hi_;

  // ---- 可视化开关 ----
  bool publish_slope_;
  bool publish_roughness_;
  bool publish_landing_;
  bool publish_depression_raw_;
  bool publish_depression_norm_;
  bool publish_depression_score_;
  bool publish_flatness_score_;
  bool publish_landing_score_;
  bool publish_alt_landing_;

  // ---- landing_score 权重 ----
  double landing_score_w_slope_;
  double landing_score_w_step_;
  double landing_score_w_dep_;
  double landing_score_w_rough_;

  // ---- 最优落点选择 ----
  double best_point_score_weight_;
  int    best_point_median_window_;
  double best_point_jump_guard_m_;

  // ---- 备降区参数 ----
  double alt_landing_relax_factor_;
  double alt_landing_max_relax_;
  bool   enable_alt_landing_;

  // ---- 数据记录 ----
  std::string metrics_csv_path_;
  int         metrics_record_every_n_;
};

} // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "safeland_node");
  SafelandNode node;
  ros::spin();
  return 0;
}
