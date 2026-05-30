#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// ============================================================================
// GlobalGridMapNode
// ============================================================================
// 支持三种工作模式（pcd_mode 参数）：
//
//   "online"  (默认)  — 与原版完全一致，仅订阅 /cloud_registered 实时流
//   "offline"         — 启动时一次性加载指定的 .pcd 文件，不订阅实时点云
//                       pcd 中点坐标已是世界系（camera_init / map），直接灌入
//   "both"            — 先加载离线 PCD 作为初始底图，再叠加在线流增量更新
//
// 离线 PCD 坐标系说明：
//   FAST-LIO 输出的 /cloud_registered 及保存的 .pcd 均在 camera_init（世界系）下，
//   不需要再做外参变换。若用户提供的是雷达本体系点云，需在 yaml 中设置
//   pcd_body_frame: true，并填写 pcd_extrinsic_T / pcd_extrinsic_R（LiDAR→world）。
// ============================================================================

namespace {

// 结构体，地图边界范围
struct Bounds2d {
  double min_x;
  double max_x;
  double min_y;
  double max_y;
};

class GlobalGridMapNode {
 public:
  GlobalGridMapNode()
      : nh_(),
        pnh_("~"),
        initialized_(false),
        publish_rate_(5.0),
        resolution_(0.2),
        voxel_leaf_size_x_(0.2),
        voxel_leaf_size_y_(0.2),
        voxel_leaf_size_z_(0.2),
        initial_padding_x_(5.0),
        initial_padding_y_(5.0),
        expand_margin_x_(3.0),
        expand_margin_y_(3.0),
        min_valid_z_(-1.8),
        max_valid_z_(1.0),
        cell_count_threshold_(0),
        publish_debug_cloud_(true),
        subscriber_queue_size_(1),
        // 离线/混合模式新增
        pcd_mode_("online"),
        pcd_path_(""),
        pcd_frame_id_("camera_init"),
        pcd_body_frame_(false) {
    loadParameters();

    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_, 1, true);
    if (publish_debug_cloud_) {
      debug_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_cloud_topic_, 1, true);
    }

    // ---- 离线模式：启动时一次性加载 PCD ----
    if (pcd_mode_ == "offline" || pcd_mode_ == "both") {
      loadOfflinePcd();
    }

    // ---- 在线/混合模式：订阅实时点云 ----
    if (pcd_mode_ == "online" || pcd_mode_ == "both") {
      cloud_sub_ = nh_.subscribe(input_cloud_topic_, subscriber_queue_size_,
                                 &GlobalGridMapNode::cloudCallback, this);
      ROS_INFO_STREAM("[GridMap] Subscribed to point cloud topic: " << input_cloud_topic_);
    }

    if (publish_rate_ > 0.0) {
      publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                       &GlobalGridMapNode::publishTimerCallback, this);
    }

    ROS_INFO_STREAM("[GridMap] Publishing grid map on: " << grid_map_topic_);
    ROS_INFO_STREAM("[GridMap] Mode: " << pcd_mode_);
  }

 private:
  // --------------------------------------------------------------------------
  // 参数加载
  // --------------------------------------------------------------------------
  void loadParameters() {
    pnh_.param<std::string>("input_cloud_topic",  input_cloud_topic_,  "/cloud_registered");
    pnh_.param<std::string>("map_frame",           map_frame_,          "camera_init");
    pnh_.param<std::string>("grid_map_topic",      grid_map_topic_,     "/global_grid_map");
    pnh_.param<std::string>("debug_cloud_topic",   debug_cloud_topic_,  "/global_grid_map/points");
    pnh_.param<std::string>("elevation_layer",     elevation_layer_,    "elevation");
    pnh_.param<std::string>("sum_layer",           sum_layer_,          "sum_z");
    pnh_.param<std::string>("count_layer",         count_layer_,        "count");

    pnh_.param("publish_rate",          publish_rate_,          publish_rate_);
    pnh_.param("resolution",            resolution_,            resolution_);
    pnh_.param("voxel_leaf_size_x",     voxel_leaf_size_x_,    voxel_leaf_size_x_);
    pnh_.param("voxel_leaf_size_y",     voxel_leaf_size_y_,    voxel_leaf_size_y_);
    pnh_.param("voxel_leaf_size_z",     voxel_leaf_size_z_,    voxel_leaf_size_z_);
    pnh_.param("initial_padding_x",     initial_padding_x_,    initial_padding_x_);
    pnh_.param("initial_padding_y",     initial_padding_y_,    initial_padding_y_);
    pnh_.param("expand_margin_x",       expand_margin_x_,      expand_margin_x_);
    pnh_.param("expand_margin_y",       expand_margin_y_,      expand_margin_y_);
    pnh_.param("min_valid_z",           min_valid_z_,          min_valid_z_);
    // max_valid_z：高度上限过滤（m）
    //   超过此高度的点被剔除，避免树冠、建筑物外堄1等非地面点混入高程图
    //   -999（或任意负大值）表示不限制上限
    pnh_.param("max_valid_z",           max_valid_z_,          max_valid_z_);
    pnh_.param("cell_count_threshold",  cell_count_threshold_, cell_count_threshold_);
    pnh_.param("publish_debug_cloud",   publish_debug_cloud_,  publish_debug_cloud_);
    pnh_.param("subscriber_queue_size", subscriber_queue_size_, subscriber_queue_size_);

    // ---- 离线 PCD 参数 ----
    // pcd_mode: "online" | "offline" | "both"
    pnh_.param<std::string>("pcd_mode",    pcd_mode_,    "online");
    // pcd_path: 离线 .pcd 文件的绝对路径（空 = 不加载）
    pnh_.param<std::string>("pcd_path",    pcd_path_,    "");
    // pcd_frame_id: 离线 PCD 所在坐标系（与地图坐标系一致时无需额外变换）
    //   FAST-LIO 保存的 .pcd 默认在 camera_init（世界系），设为 "camera_init"
    pnh_.param<std::string>("pcd_frame_id", pcd_frame_id_, "camera_init");
    // pcd_body_frame: 若 PCD 是雷达本体系点云，需设为 true 并填写外参
    pnh_.param("pcd_body_frame", pcd_body_frame_, false);

    // pcd_extrinsic_T: 雷达→世界 的平移 [x, y, z]（仅 pcd_body_frame=true 时生效）
    std::vector<double> ext_t = {0.0, 0.0, 0.0};
    pnh_.param("pcd_extrinsic_T", ext_t, ext_t);
    pcd_ext_T_ = Eigen::Vector3d(ext_t[0], ext_t[1], ext_t[2]);

    // pcd_extrinsic_R: 雷达→世界 的旋转矩阵（行主序 3×3，仅 pcd_body_frame=true 时生效）
    std::vector<double> ext_r = {1,0,0, 0,1,0, 0,0,1};
    pnh_.param("pcd_extrinsic_R", ext_r, ext_r);
    pcd_ext_R_ << ext_r[0], ext_r[1], ext_r[2],
                  ext_r[3], ext_r[4], ext_r[5],
                  ext_r[6], ext_r[7], ext_r[8];

    if (resolution_ <= 0.0) {
      throw std::runtime_error("Parameter `resolution` must be positive.");
    }
  }

  // --------------------------------------------------------------------------
  // 离线 PCD 加载（模式 offline / both）
  // --------------------------------------------------------------------------
  void loadOfflinePcd() {
    if (pcd_path_.empty()) {
      ROS_WARN("[GridMap] pcd_mode=%s but pcd_path is empty, skipping PCD load.",
               pcd_mode_.c_str());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *raw_cloud) < 0) {
      ROS_ERROR("[GridMap] Failed to load PCD file: %s", pcd_path_.c_str());
      return;
    }
    ROS_INFO("[GridMap] Loaded PCD: %s (%zu points)", pcd_path_.c_str(), raw_cloud->size());

    // ---- 坐标系变换：雷达本体系 → 世界系 ----
    // FAST-LIO 输出的 /cloud_registered 已在世界系（camera_init），
    // 保存的 .pcd 与之一致，pcd_body_frame=false 时无需变换。
    // 若用户提供的是原始雷达扫描帧（body frame），则用 pcd_extrinsic_R/T 变换。
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcd_body_frame_) {
      world_cloud->reserve(raw_cloud->size());
      for (const auto& pt : raw_cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
          continue;
        }
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d pw = pcd_ext_R_ * p + pcd_ext_T_;
        pcl::PointXYZ out_pt;
        out_pt.x = static_cast<float>(pw.x());
        out_pt.y = static_cast<float>(pw.y());
        out_pt.z = static_cast<float>(pw.z());
        world_cloud->push_back(out_pt);
      }
      world_cloud->width  = world_cloud->size();
      world_cloud->height = 1;
      world_cloud->is_dense = false;
      ROS_INFO("[GridMap] Transformed PCD from body frame to world frame (%zu → %zu pts)",
               raw_cloud->size(), world_cloud->size());
    } else {
      // 已在世界系，直接使用（仅过滤 NaN）
      world_cloud->reserve(raw_cloud->size());
      for (const auto& pt : raw_cloud->points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
          world_cloud->push_back(pt);
        }
      }
      world_cloud->width  = world_cloud->size();
      world_cloud->height = 1;
      world_cloud->is_dense = false;
    }

    // ---- 高度过滤 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered = filterCloudByHeight(world_cloud);
    if (height_filtered->empty()) {
      ROS_WARN("[GridMap] PCD is empty after height filtering (min_valid_z=%.2f)", min_valid_z_);
      return;
    }

    // ---- 体素降采样 ----
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = downsampleCloud(height_filtered);
    if (filtered->empty()) {
      ROS_WARN("[GridMap] PCD is empty after downsampling.");
      return;
    }

    // ---- 灌入 grid_map ----
    const Bounds2d bounds = computeBounds(*filtered);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_frame_id_ = pcd_frame_id_;
      last_stamp_    = ros::Time::now();
      ensureMapContains(bounds, pcd_frame_id_);
      updateMapFromCloud(*filtered);
      map_.setTimestamp(last_stamp_.toNSec());
    }

    ROS_INFO("[GridMap] Offline PCD fully ingested into grid map (%zu pts after filtering).",
             filtered->size());

    // ---- 离线模式下立即发布一次，让 safeland 能立刻收到 ----
    if (publish_rate_ <= 0.0) {
      publishGridMap();
    }
  }

  // --------------------------------------------------------------------------
  // 在线回调（流式点云，实时增量更新）
  // --------------------------------------------------------------------------
  // 连续性保障：
  //   grid_map 本身是累积式的（每帧叠加，sum_z / count 持续累加）。
  //   cell_count_threshold 参数控制每栅格最多累积多少帧点就"冻结"该格：
  //     - 冻结后该格高度稳定，不会被后续噪声点覆盖，保证帧间连续性。
  //     - threshold=0 表示不限制（全部累加取平均，适合稳定场景）。
  //     - threshold=20（默认）表示每格最多 20 帧更新，兼顾速度与稳定性。
  //   此外，体素降采样（voxel_grid）保证每帧输入点不超密，减少累积误差。
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    const ros::WallTime t0 = ros::WallTime::now();
    const std::string resolved_frame = resolveMapFrame(*cloud_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud = filterCloudByHeight(input_cloud);
    if (height_filtered_cloud->empty()) {
      ROS_WARN_THROTTLE(5.0, "[GridMap] Empty cloud after height filtering, skipping.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = downsampleCloud(height_filtered_cloud);
    if (filtered_cloud->empty()) {
      ROS_WARN_THROTTLE(5.0, "[GridMap] Empty cloud after downsampling, skipping.");
      return;
    }

    const Bounds2d bounds = computeBounds(*filtered_cloud);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_stamp_    = cloud_msg->header.stamp;
      last_frame_id_ = resolved_frame;
      ensureMapContains(bounds, resolved_frame);
      updateMapFromCloud(*filtered_cloud);
      map_.setTimestamp(last_stamp_.toNSec());
    }

    const double dt_ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
    ROS_INFO_THROTTLE(
        2.0,
        "[GridMap] frame input=%zu height=%zu downsampled=%zu resolution=%.3f voxel=(%.3f,%.3f,%.3f) dt=%.2fms",
        input_cloud->size(), height_filtered_cloud->size(), filtered_cloud->size(),
        resolution_, voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_, dt_ms);

    if (publish_rate_ <= 0.0) {
      publishGridMap();
    }
  }

  // --------------------------------------------------------------------------
  // 点云处理工具
  // --------------------------------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloudByHeight(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    output_cloud->reserve(input_cloud->size());

    // max_valid_z 不限制时，不做上限过滤
    const bool has_upper = (max_valid_z_ > -100.0);

    for (const auto& point : input_cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.z < min_valid_z_) {
        continue;
      }
      // 高度上限过滤：剔除树冠、建筑物外堄1、悬挂等非地面点
      if (has_upper && point.z > static_cast<float>(max_valid_z_)) {
        continue;
      }
      output_cloud->push_back(point);
    }

    output_cloud->width  = output_cloud->size();
    output_cloud->height = 1;
    output_cloud->is_dense = false;
    return output_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input_cloud) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (voxel_leaf_size_x_ <= 0.0 || voxel_leaf_size_y_ <= 0.0 || voxel_leaf_size_z_ <= 0.0) {
      *output_cloud = *input_cloud;
      return output_cloud;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(static_cast<float>(voxel_leaf_size_x_),
                             static_cast<float>(voxel_leaf_size_y_),
                             static_cast<float>(voxel_leaf_size_z_));
    voxel_filter.filter(*output_cloud);
    return output_cloud;
  }

  std::string resolveMapFrame(const sensor_msgs::PointCloud2& cloud_msg) const {
    if (!map_frame_.empty() && !cloud_msg.header.frame_id.empty() &&
        map_frame_ != cloud_msg.header.frame_id) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "[GridMap] Cloud frame_id='" << cloud_msg.header.frame_id
               << "' differs from configured map_frame='" << map_frame_
               << "'. Using incoming frame (no TF applied).");
    }
    return cloud_msg.header.frame_id.empty() ? map_frame_ : cloud_msg.header.frame_id;
  }

  // --------------------------------------------------------------------------
  // Grid Map 管理
  // --------------------------------------------------------------------------
  Bounds2d computeBounds(const pcl::PointCloud<pcl::PointXYZ>& cloud) const {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    return Bounds2d{min_pt.x, max_pt.x, min_pt.y, max_pt.y};
  }

  void ensureMapContains(const Bounds2d& cloud_bounds, const std::string& frame_id) {
    if (!initialized_) {
      const double padding_x = std::max(initial_padding_x_, resolution_);
      const double padding_y = std::max(initial_padding_y_, resolution_);
      initializeMap(makeExpandedBounds(cloud_bounds, padding_x, padding_y), frame_id);
      return;
    }

    const Bounds2d map_bounds = getCurrentMapBounds();
    const double margin_x = std::max(expand_margin_x_, resolution_);
    const double margin_y = std::max(expand_margin_y_, resolution_);

    if (cloud_bounds.min_x >= map_bounds.min_x && cloud_bounds.max_x <= map_bounds.max_x &&
        cloud_bounds.min_y >= map_bounds.min_y && cloud_bounds.max_y <= map_bounds.max_y) {
      if (map_.getFrameId() != frame_id) {
        ROS_WARN_STREAM_THROTTLE(5.0, "[GridMap] Frame changed from "
                                 << map_.getFrameId() << " to " << frame_id);
        map_.setFrameId(frame_id);
      }
      return;
    }

    Bounds2d expanded_bounds = map_bounds;
    expanded_bounds.min_x = std::min(map_bounds.min_x, cloud_bounds.min_x - margin_x);
    expanded_bounds.max_x = std::max(map_bounds.max_x, cloud_bounds.max_x + margin_x);
    expanded_bounds.min_y = std::min(map_bounds.min_y, cloud_bounds.min_y - margin_y);
    expanded_bounds.max_y = std::max(map_bounds.max_y, cloud_bounds.max_y + margin_y);
    expandMap(expanded_bounds, frame_id);
  }

  Bounds2d makeExpandedBounds(const Bounds2d& bounds,
                               double padding_x, double padding_y) const {
    return Bounds2d{bounds.min_x - padding_x, bounds.max_x + padding_x,
                    bounds.min_y - padding_y, bounds.max_y + padding_y};
  }

  void initializeMap(const Bounds2d& bounds, const std::string& frame_id) {
    map_ = createEmptyMap(bounds, frame_id);
    initialized_ = true;
    ROS_INFO_STREAM("[GridMap] Initialized grid map " << map_.getLength().x()
                    << " x " << map_.getLength().y() << " m @ " << map_.getResolution() << " m/cell");
  }

  void expandMap(const Bounds2d& bounds, const std::string& frame_id) {
    grid_map::GridMap expanded_map = createEmptyMap(bounds, frame_id);
    if (!expanded_map.addDataFrom(map_, false, true, true)) {
      // addDataFrom 失败通常因旧地图 frame 与新地图尺寸不兼容，
      // 此处降级处理：打印错误、保留当前地图继续运行，避免节点崩溃
      // 下一帧到来时若仍超界，会再次尝试扩展
      ROS_ERROR_THROTTLE(5.0,
          "[GridMap] Failed to copy data into expanded map (frames: %s -> %s). "
          "Keeping current map to avoid crash.",
          map_.getFrameId().c_str(), frame_id.c_str());
      return;
    }
    map_ = std::move(expanded_map);
    ROS_INFO_STREAM("[GridMap] Expanded grid map to "
                    << map_.getLength().x() << " x " << map_.getLength().y() << " m");
  }

  grid_map::GridMap createEmptyMap(const Bounds2d& bounds,
                                    const std::string& frame_id) const {
    const double min_length = resolution_;
    const double length_x = std::max(bounds.max_x - bounds.min_x, min_length);
    const double length_y = std::max(bounds.max_y - bounds.min_y, min_length);
    const grid_map::Length length(length_x, length_y);
    const grid_map::Position position((bounds.min_x + bounds.max_x) * 0.5,
                                       (bounds.min_y + bounds.max_y) * 0.5);

    grid_map::GridMap gm({elevation_layer_, sum_layer_, count_layer_});
    gm.setFrameId(frame_id);
    gm.setGeometry(length, resolution_, position);
    gm.add(elevation_layer_, std::numeric_limits<float>::quiet_NaN());
    gm.add(sum_layer_,       0.0F);
    gm.add(count_layer_,     0.0F);
    gm.setBasicLayers({elevation_layer_});
    return gm;
  }

  Bounds2d getCurrentMapBounds() const {
    const double half_x = map_.getLength().x() * 0.5;
    const double half_y = map_.getLength().y() * 0.5;
    return Bounds2d{map_.getPosition().x() - half_x, map_.getPosition().x() + half_x,
                    map_.getPosition().y() - half_y, map_.getPosition().y() + half_y};
  }

  // --------------------------------------------------------------------------
  // 地图更新：遍历点云叠加到对应栅格（累积均值高度）
  // --------------------------------------------------------------------------
  // 帧间连续性机制：
  //   每帧点云与历史数据 sum_z / count 累加，elevation = sum_z / count（增量均值）。
  //   这意味着：
  //     - 无论在线帧到达顺序如何，最终高程是所有有效点的均值，不会因单帧跳变而漂移。
  //     - cell_count_threshold > 0 时，该格一旦积累足够帧就停止更新，
  //       避免长时间漂移或过度平均导致坑洼被"填平"。
  void updateMapFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    for (const auto& point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      grid_map::Index index;
      if (!map_.getIndex(grid_map::Position(point.x, point.y), index)) {
        continue;
      }

      float& count = map_.at(count_layer_, index);
      if (cell_count_threshold_ > 0 && count >= static_cast<float>(cell_count_threshold_)) {
        continue;  // 该格已达上限，冻结不再更新
      }

      float& sum_z    = map_.at(sum_layer_, index);
      float& elevation = map_.at(elevation_layer_, index);

      sum_z    += point.z;
      count    += 1.0F;
      elevation = sum_z / count;
    }
  }

  // --------------------------------------------------------------------------
  // 定时发布
  // --------------------------------------------------------------------------
  void publishTimerCallback(const ros::TimerEvent&) {
    publishGridMap();
  }

  void publishGridMap() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return;
    }

    if (!last_frame_id_.empty()) {
      map_.setFrameId(last_frame_id_);
    }
    map_.setTimestamp(last_stamp_.isZero() ? ros::Time::now().toNSec() : last_stamp_.toNSec());

    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(map_, map_msg);
    grid_map_pub_.publish(map_msg);

    if (publish_debug_cloud_) {
      sensor_msgs::PointCloud2 cloud_msg;
      grid_map::GridMapRosConverter::toPointCloud(map_, elevation_layer_, cloud_msg);
      debug_cloud_pub_.publish(cloud_msg);
    }
  }

  // --------------------------------------------------------------------------
  // 成员变量
  // --------------------------------------------------------------------------
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher  grid_map_pub_;
  ros::Publisher  debug_cloud_pub_;
  ros::Timer      publish_timer_;

  mutable std::mutex mutex_;
  grid_map::GridMap  map_;
  bool initialized_;

  std::string input_cloud_topic_;
  std::string map_frame_;
  std::string grid_map_topic_;
  std::string debug_cloud_topic_;
  std::string elevation_layer_;
  std::string sum_layer_;
  std::string count_layer_;
  std::string last_frame_id_;

  double publish_rate_;
  double resolution_;
  double voxel_leaf_size_x_;
  double voxel_leaf_size_y_;
  double voxel_leaf_size_z_;
  double initial_padding_x_;
  double initial_padding_y_;
  double expand_margin_x_;
  double expand_margin_y_;
  double min_valid_z_;
  double max_valid_z_;        // 高度上限过滤，默认1.0m
  int    cell_count_threshold_;
  bool   publish_debug_cloud_;
  int    subscriber_queue_size_;

  ros::Time last_stamp_;

  // ---- 离线 PCD 参数 ----
  std::string     pcd_mode_;          // "online" | "offline" | "both"
  std::string     pcd_path_;          // 离线 .pcd 绝对路径
  std::string     pcd_frame_id_;      // 离线 PCD 所在坐标系
  bool            pcd_body_frame_;    // true = 需要做外参变换
  Eigen::Vector3d pcd_ext_T_;         // 雷达→世界 平移
  Eigen::Matrix3d pcd_ext_R_;         // 雷达→世界 旋转
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_grid_map_node");

  try {
    GlobalGridMapNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("[GridMap] Fatal: " << e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
