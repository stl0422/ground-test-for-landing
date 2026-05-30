#!/usr/bin/env python3
# coding=utf-8
"""
简易航点状态机（已接入 safeland + 地图成熟度评估）：

在线建图终止条件（两者同时满足才提前结束巡航）：
  ① 地图覆盖成熟：landing_center 图层中可降落栅格数 ≥ map_mature_min_cells
  ② 地图稳定：在连续 map_stable_window 秒内，landing_center 栅格数变化幅度
               < map_stable_delta_ratio × 当前数量（即变化率低于阈值）

触发后立刻进入 ST_WAIT_SAFELAND，停留一个 safeland_confirm_secs 的确认窗口
（让 safeland 节点再刷新 1~2 帧，确保拿到最新结果），然后飞往最优降落点。

若巡航完所有航点仍未满足条件，也会正常进入 ST_WAIT_SAFELAND 等待。
超时（safeland_timeout 秒）后回退到最后航点原地降落。

话题对齐：/goal、/drone_odometry、/mavros/state、/stop_super
安全图：  /safeland/grid_map  (landing_center 图层，值=1.0 表示可降落)
"""

import collections
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8
from tf.transformations import quaternion_from_euler

# ---------------------------------------------------------------------------
# 巡航航点（camera_init 系），格式 [x, y, z, yaw_deg]
# 最后一个航点作为"建图结束后的等待悬停点"
# ---------------------------------------------------------------------------
TAKEOFF_POINT = [0.0, 0.0, 2.5, 0.0]

WAYPOINTS = [
    [4.0,  0.0,  2.5,   0.0],
    [4.0,  5.0,  2.5,  90.0],
    [-4.0, 5.0,  2.5, 180.0],
    [-4.0, -5.0, 2.5, 270.0],
    [4.0,  -5.0, 2.5,   0.0],
    [0.0,  0.0,  2.5,   0.0],   # 巡航结束后原地悬停等待 safeland 结果
]

LAND_Z = 0.0   # 最终着地目标高度（camera_init 系 z）

# ---------------------------------------------------------------------------
# 状态编号
# ---------------------------------------------------------------------------
ST_IDLE           = 0   # 等待起飞/对齐
ST_CRUISE         = 1   # 沿航点巡航建图
ST_WAIT_SAFELAND  = 2   # 地图成熟/航点完成 → 等待 safeland 确认降落点
ST_FLY_TO_LAND    = 3   # 飞往最优降落点上方
ST_LAND           = 4   # 下降并上锁
ST_DONE           = 5   # 任务完成


class SimpleStateMachine:

    def __init__(self):
        rospy.init_node("iros_state_machine_simple")

        if not WAYPOINTS:
            rospy.logfatal("[SM] WAYPOINTS 为空，请至少填写一个目标点")
            raise SystemExit(1)

        # ---------- ROS 参数 ----------
        self.goal_frame_id     = rospy.get_param("~goal_frame_id",      "camera_init")
        self.takeoff_xy_tol    = float(rospy.get_param("~takeoff_xy_tol",   0.5))
        self.takeoff_z_tol     = float(rospy.get_param("~takeoff_z_tol",    0.35))
        self.cruise_xy_tol     = float(rospy.get_param("~cruise_xy_tol",    0.5))
        self.cruise_z_tol      = float(rospy.get_param("~cruise_z_tol",     0.5))
        self.land_xy_tol       = float(rospy.get_param("~land_xy_tol",      0.35))
        self.land_z_tol        = float(rospy.get_param("~land_z_tol",       0.2))

        # safeland 接口
        self.safeland_topic    = rospy.get_param("~safeland_topic",  "/safeland/grid_map")
        self.safeland_layer    = rospy.get_param("~safeland_layer",  "landing_center")
        self.safeland_max_dist = float(rospy.get_param("~safeland_max_dist",  0.0))
        self.safeland_timeout  = float(rospy.get_param("~safeland_timeout",  30.0))
        self.safeland_land_z   = float(rospy.get_param("~safeland_land_z",   LAND_Z))
        self.backup_landing_zones = self._parse_backup_zones(
            rospy.get_param("~backup_landing_zones", [
                [1.5, -0.5, 0.0, 1.0],
                [-2.5, 3.5, 0.0, 1.0],
                [4.5, -4.5, 0.0, 1.0],
            ])
        )
        self.backup_approach_height = float(rospy.get_param("~backup_approach_height", 2.5))
        # landing_score 图层名称：如果 safeland 发布了该图层，可用距离+得分加权选点
        # 当 safeland 未发布 landing_score 图层时，回退为纯最近距离选点
        self.safeland_score_layer = rospy.get_param("~safeland_score_layer", "landing_score")
        # 距离-得分权重：0.0 = 纯最近，1.0 = 纯得分最高，默认 0.4
        # 含义：60% 权重给距离，40% 权重给安全性，防止飞到很远的极高分点
        self.safeland_score_weight = float(rospy.get_param("~safeland_score_weight", 0.4))

        # ---- 地图成熟度判断参数 ----
        # map_mature_min_cells：全图 landing_center=1 的栅格数下限
        #   达到此数量说明已有足够面积的可降落区，可以提前结束巡航
        #   0 = 禁用（不提前结束，必须飞完所有航点）
        self.map_mature_min_cells = int(rospy.get_param("~map_mature_min_cells", 20))

        # map_stable_window：稳定性评估滑窗时长（秒）
        #   在此窗口内 landing_center 栅格数变化率 < map_stable_delta_ratio 才认为"稳定"
        self.map_stable_window    = float(rospy.get_param("~map_stable_window",   5.0))

        # map_stable_delta_ratio：稳定性变化率阈值 [0, 1]
        #   窗口内 (max_cnt - min_cnt) / max_cnt < 此值 → 稳定
        #   0.05 = 5% 变化率，即可降落面积基本不再增长
        self.map_stable_delta_ratio = float(rospy.get_param("~map_stable_delta_ratio", 0.05))

        # safeland_confirm_secs：进入 ST_WAIT_SAFELAND 后额外等待的确认秒数
        #   让 safeland 节点再刷新 1~2 帧，确保拿到最终稳定结果，然后才飞往降落点
        self.safeland_confirm_secs = float(rospy.get_param("~safeland_confirm_secs", 2.0))

        # ---------- 内部状态 ----------
        self.odom         = Odometry()
        self.mavros_state = State()
        self.state        = ST_IDLE
        self.wp_index     = 0
        self.last_request = rospy.Time(0)

        # safeland 解析结果
        self._land_target        = None   # [x, y, z_cruise, yaw_deg]
        self._safeland_recv_time = None   # 收到第一帧有效目标的时刻
        self._wait_start_time    = None   # 进入 ST_WAIT_SAFELAND 的时刻

        # 地图成熟度跟踪
        # 用双端队列保存 (timestamp, landing_cell_count) 对，供稳定性滑窗分析
        self._cell_history = collections.deque()  # (time.time(), count)
        self._map_mature_triggered = False         # 是否已触发过提前终止

        # 降落阶段辅助
        self._super_off_sent    = False
        self._land_disarm_start = None
        self._present_land_hold = False
        self._land_requested    = False
        self._land_target_source = "none"
        self._last_land_request = rospy.Time(0)
        self._land_enter_time   = None   # 进入 ST_LAND 的时刻（用于超时保护）
        # ST_LAND 最终超时：若 land_timeout_secs 秒内仍未贴地也未上锁，强制 disarm
        # 防止飞控 AUTO.LAND 异常时状态机永远卡死在 ST_LAND
        self.land_timeout_secs  = float(rospy.get_param("~land_timeout_secs", 30.0))

        # ---------- 发布 / 订阅 ----------
        self.pose = PoseStamped()
        self.pose.header.frame_id = self.goal_frame_id

        self.odom_sub         = rospy.Subscriber("/drone_odometry",    Odometry, self._odom_cb)
        self.mavros_state_sub = rospy.Subscriber("/mavros/state",      State,    self._mavros_cb)
        self.safeland_sub     = rospy.Subscriber(self.safeland_topic,  GridMap,  self._safeland_cb)

        self.goal_pub       = rospy.Publisher("/goal",          PoseStamped, queue_size=10)
        self.stop_super_pub = rospy.Publisher("/stop_super",    Bool,        queue_size=1)
        self.state_pub      = rospy.Publisher("/state_machine", Int8,        queue_size=10)

        # ---------- 服务 ----------
        self.arming_client   = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.land_client     = rospy.ServiceProxy("/mavros/cmd/land",   CommandTOL)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode",   SetMode)

        rospy.loginfo("[SM] 初始化完成。起飞=%s 航点=%d safeland=%s 备降区=%d",
                      TAKEOFF_POINT, len(WAYPOINTS), self.safeland_topic,
                      len(self.backup_landing_zones))
        rospy.loginfo("[SM] 地图成熟度: min_cells=%d stable_window=%.1fs delta_ratio=%.2f confirm=%.1fs",
                      self.map_mature_min_cells, self.map_stable_window,
                      self.map_stable_delta_ratio, self.safeland_confirm_secs)

    # -----------------------------------------------------------------------
    # 回调
    # -----------------------------------------------------------------------
    def _odom_cb(self, msg):
        self.odom = msg

    def _mavros_cb(self, msg):
        self.mavros_state = msg

    def _safeland_cb(self, msg):
        """
        解析 /safeland/grid_map 中的 landing_center 和 landing_score 图层。
        三个作用：
          1. [成熟度评估] 记录当前 landing_center 栅格数，供稳定性滑窗使用
          2. [目标选取]   用「距离归一化得分 + 安全性得分」加权综合已得分选最优降落点
             若无 landing_score 图层，回退为纯距离最近
        注意：ST_FLY_TO_LAND / ST_LAND / ST_DONE 阶段不更新降落目标，
             防止飞行中目标跳变。
        """
        if self.state not in (ST_CRUISE, ST_WAIT_SAFELAND):
            return

        # 找 landing_center 图层
        try:
            layer_idx = msg.layers.index(self.safeland_layer)
        except ValueError:
            rospy.logwarn_throttle(5.0, "[SM] /safeland/grid_map 无 '%s' 图层", self.safeland_layer)
            return

        # 尝试找 landing_score 图层（可能不存在）
        score_idx = None
        try:
            score_idx = msg.layers.index(self.safeland_score_layer)
        except ValueError:
            pass  # 没有得分图层时回退为纯距离选点

        # 解析元数据
        # grid_map 规约：size_(0) = X 方向行数（对应 length_x），
        #                 size_(1) = Y 方向列数（对应 length_y）
        # 注意：之前 rows/cols 赋值颠倒（nx 对应 rows 而非 cols），已修正
        res    = msg.info.resolution
        origin = msg.info.pose.position
        nx     = msg.info.length_x   # X 方向（行方向）总长度
        ny     = msg.info.length_y   # Y 方向（列方向）总长度
        rows   = int(round(nx / res))  # 行数 = X 方向格子数
        cols   = int(round(ny / res))  # 列数 = Y 方向格子数

        data = msg.data[layer_idx].data
        if len(data) != rows * cols:
            rospy.logwarn_throttle(5.0, "[SM] landing_center 数据长度 %d ≠ %d×%d",
                                   len(data), rows, cols)
            return

        # grid_map 列主序 (column-major)解析
        arr = np.array(data, dtype=np.float32).reshape((rows, cols), order='F')

        # 解析 landing_score 图层（如果存在）
        score_arr = None
        if score_idx is not None:
            score_data = msg.data[score_idx].data
            if len(score_data) == rows * cols:
                score_arr = np.array(score_data, dtype=np.float32).reshape((rows, cols), order='F')

        # ── 1. 成熟度追踪：统计 landing_center=1 的格子数 ──
        valid_mask = np.isfinite(arr) & (arr >= 0.5)
        cell_count = int(valid_mask.sum())
        now_t = time.time()
        self._cell_history.append((now_t, cell_count))
        # 清理超出时间窗口的旧记录
        cutoff = now_t - self.map_stable_window
        while self._cell_history and self._cell_history[0][0] < cutoff:
            self._cell_history.popleft()

        if not valid_mask.any():
            rospy.logwarn_throttle(5.0, "[SM] 暂无有效降落栅格 (max_dist=%.1f m, cells=0)",
                                   self.safeland_max_dist)
            return

        # ── 2. 向量化计算所有候选点坐标 + 距离 + 得分 ──
        cur_x = self.odom.pose.pose.position.x
        cur_y = self.odom.pose.pose.position.y

        rows_idx, cols_idx = np.where(valid_mask)
        if len(rows_idx) == 0:
            return

        # 将所有候选格子的坐标一次性转成向量运算，避免 Python级别循环
        map_x_arr = origin.x + (rows * 0.5 - 0.5 - rows_idx.astype(np.float32)) * res
        map_y_arr = origin.y + (cols * 0.5 - 0.5 - cols_idx.astype(np.float32)) * res
        dist_arr  = np.hypot(map_x_arr - cur_x, map_y_arr - cur_y)

        # 距离过滤
        if self.safeland_max_dist > 0.0:
            mask_dist = dist_arr <= self.safeland_max_dist
            if not mask_dist.any():
                rospy.logwarn_throttle(5.0, "[SM] 所有候选点超出 max_dist=%.1f m范围",
                                       self.safeland_max_dist)
                return
            rows_idx = rows_idx[mask_dist]
            cols_idx = cols_idx[mask_dist]
            map_x_arr = map_x_arr[mask_dist]
            map_y_arr = map_y_arr[mask_dist]
            dist_arr  = dist_arr[mask_dist]

        # 距离归一化：[0,1]，距离越近 dist_norm 越接近 0
        # 当只有一个候选点时 dist_max == dist_arr[0]，直接令 dist_norm=0（该点视为最近）
        dist_max  = dist_arr.max()
        if dist_max < 1e-3 or len(dist_arr) == 1:
            dist_norm = np.zeros_like(dist_arr)
        else:
            dist_norm = dist_arr / dist_max  # 越小越近

        # 得分图层存在时取对应得分；不存在时设为 0.5（中性得分）
        if score_arr is not None:
            safety_scores = score_arr[rows_idx, cols_idx].astype(np.float32)
            # NaN 处理：直接将 NaN 得分替换为 0.5（中性）
            safety_scores = np.where(np.isfinite(safety_scores), safety_scores, 0.5)
        else:
            safety_scores = np.full(len(rows_idx), 0.5, dtype=np.float32)

        # 综合得分 = (1-w)*近度得分 + w*安全性得分
        #   近度得分 = 1 - dist_norm（越近越高）
        #   安全性得分 = safety_scores（越高越平整）
        w = self.safeland_score_weight
        composite = (1.0 - w) * (1.0 - dist_norm) + w * safety_scores

        best_idx  = int(np.argmax(composite))
        best_xy   = (float(map_x_arr[best_idx]), float(map_y_arr[best_idx]))
        best_dist = float(dist_arr[best_idx])
        best_comp = float(composite[best_idx])

        # 偏航角沿用当前朝向（使用完整四元数公式，避免仅用 qz/qw 的近似误差）
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))  ← ZYX 欧拉角约定
        ori = self.odom.pose.pose.orientation
        qx, qy, qz_ori, qw_ori = ori.x, ori.y, ori.z, ori.w
        yaw_deg = math.degrees(
            math.atan2(2.0 * (qw_ori * qz_ori + qx * qy),
                       1.0 - 2.0 * (qy * qy + qz_ori * qz_ori))
        )

        self._land_target        = [best_xy[0], best_xy[1],
                                    self.safeland_land_z + 2.5, yaw_deg]
        self._land_target_source = "safeland"
        self._safeland_recv_time = rospy.Time.now()

        rospy.loginfo_throttle(2.0,
            "[SM] 降落候选: (%.2f,%.2f) dist=%.2fm composite=%.3f | "
            "cells=%d score_layer=%s",
            best_xy[0], best_xy[1], best_dist, best_comp,
            cell_count, "✔" if score_arr is not None else "✘")

    # -----------------------------------------------------------------------
    # 地图成熟度判断
    # -----------------------------------------------------------------------
    def _is_map_mature(self):
        """
        判断当前地图是否已足够成熟，可以提前终止巡航。
        条件（全部满足）：
          ① landing_center 栅格数 ≥ map_mature_min_cells
          ② 窗口内有效时间跨度 ≥ map_stable_window（避免刚起步就错误触发）
          ③ 该窗口内栅格数变化率 < map_stable_delta_ratio
        """
        if self.map_mature_min_cells <= 0:
            return False   # 禁用提前终止

        if len(self._cell_history) < 3:
            return False

        latest_count = self._cell_history[-1][1]

        # 条件①：绝对数量达标
        if latest_count < self.map_mature_min_cells:
            return False

        # 条件②：窗口时间跨度必须 >= map_stable_window
        # _cell_history 中的旧数据被 cutoff 清理后，队列头尾时间差即为实际窗口长度。
        # 此时间跨度必须 >= map_stable_window，否则说明数据积累还不足，不允许触发。
        # 注意：不使用 0.8 宽松倍数——cutoff 已确保队列跨度不超过 window，
        # 因此只要跨度满足 >= window，才说明稳定性窗口完整覆盖。
        oldest_t = self._cell_history[0][0]
        newest_t = self._cell_history[-1][0]
        time_span = newest_t - oldest_t
        if time_span < self.map_stable_window:
            rospy.loginfo_throttle(3.0,
                "[SM] 成熟度检查: 窗口时间跨度不足 (%.1fs < %.1fs)",
                time_span, self.map_stable_window)
            return False

        # 条件③：窗口内栅格数稳定
        counts = [c for _, c in self._cell_history]
        max_c  = max(counts)
        min_c  = min(counts)

        if max_c <= 0:
            return False

        delta_ratio = (max_c - min_c) / float(max_c)
        is_stable   = delta_ratio < self.map_stable_delta_ratio

        rospy.loginfo_throttle(3.0,
            "[SM] 成熟度检查: cells=%d min_cells=%d "
            "time_span=%.1fs delta_ratio=%.3f threshold=%.3f stable=%s",
            latest_count, self.map_mature_min_cells,
            time_span, delta_ratio, self.map_stable_delta_ratio, is_stable)

        return is_stable

    # -----------------------------------------------------------------------
    # 工具方法
    # -----------------------------------------------------------------------
    def _parse_backup_zones(self, zones):
        parsed = []
        if zones is None:
            return parsed
        for zone in zones:
            if not isinstance(zone, (list, tuple)) or len(zone) < 3:
                rospy.logwarn("[SM] 忽略非法备降区配置: %s", zone)
                continue
            radius = float(zone[3]) if len(zone) >= 4 else 1.0
            parsed.append([float(zone[0]), float(zone[1]), float(zone[2]), radius])
        return parsed

    def _current_yaw_deg(self):
        ori = self.odom.pose.pose.orientation
        return math.degrees(
            math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y),
                       1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))
        )

    def _select_backup_landing_target(self):
        if not self.backup_landing_zones:
            return None
        p = self.odom.pose.pose.position
        yaw_deg = self._current_yaw_deg()
        ranked = sorted(
            self.backup_landing_zones,
            key=lambda z: math.hypot(z[0] - p.x, z[1] - p.y)
        )
        chosen = ranked[0]
        dist = math.hypot(chosen[0] - p.x, chosen[1] - p.y)
        rospy.logwarn(
            "[SM] 使用备降区: (%.2f, %.2f), radius=%.2fm, dist=%.2fm",
            chosen[0], chosen[1], chosen[3], dist)
        return [chosen[0], chosen[1], chosen[2] + self.backup_approach_height, yaw_deg]

    def _start_final_approach(self, target, source):
        self._land_target        = target
        self._land_target_source = source
        self.state              = ST_FLY_TO_LAND
        self._present_land_hold = False
        self._land_disarm_start = None
        self._land_requested    = False
        self._land_enter_time   = None

    def is_close(self, odom, pose, xy, z):
        if pose is None:
            return False
        # 保护：odom 初始为默认 Odometry（header.seq=0, stamp=0），
        # 在收到第一帧真实里程计前不做距离判断，防止全零坐标误触发状态转移
        if odom.header.stamp.secs == 0 and odom.header.stamp.nsecs == 0:
            return False
        p = odom.pose.pose.position
        return (abs(p.x - pose[0]) < xy
                and abs(p.y - pose[1]) < xy
                and abs(p.z - pose[2]) < z)

    def go_to(self, pose):
        """发布目标位姿 pose = [x, y, z, yaw_deg]"""
        self.pose.pose.position.x = pose[0]
        self.pose.pose.position.y = pose[1]
        self.pose.pose.position.z = pose[2]
        yaw = math.radians(pose[3])
        q   = quaternion_from_euler(0.0, 0.0, yaw)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]
        self.pose.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.pose)

    def super_change(self, stop_super):
        msg      = Bool()
        msg.data = stop_super
        self.stop_super_pub.publish(msg)

    def _enter_wait_safeland(self, reason=""):
        """统一的进入 ST_WAIT_SAFELAND 入口"""
        rospy.loginfo("[SM] 进入 ST_WAIT_SAFELAND%s", (" — " + reason) if reason else "")
        self.state            = ST_WAIT_SAFELAND
        self._wait_start_time = time.time()
        self.super_change(True)

    def _try_offboard_and_arm(self):
        now = rospy.Time.now()
        if (now - self.last_request).to_sec() < 1.0:
            return
        self.last_request = now
        if self.mavros_state.mode != "OFFBOARD":
            rospy.loginfo_throttle(5.0, "[SM] 等待 QGC 手动切换到 OFFBOARD ...")
            return
        if not self.mavros_state.armed:
            rospy.loginfo("[SM] 正在 Arming ...")
            try:
                self.arming_client(True)
            except rospy.ServiceException as e:
                rospy.logwarn("[SM] arming 失败: %s", e)

    def _request_land(self):
        now = rospy.Time.now()
        if (now - self._last_land_request).to_sec() < 1.0:
            return False
        self._last_land_request = now

        try:
            resp = self.land_client(min_pitch=0.0, yaw=0.0,
                                    latitude=0.0, longitude=0.0, altitude=0.0)
            if getattr(resp, "success", False):
                rospy.loginfo("[SM] 已调用 /mavros/cmd/land，开始降落")
                self._land_requested = True
                return True
            rospy.logwarn("[SM] /mavros/cmd/land 未被接受，尝试 AUTO.LAND")
        except rospy.ServiceException as e:
            rospy.logwarn("[SM] /mavros/cmd/land 失败: %s，尝试 AUTO.LAND", e)

        try:
            resp = self.set_mode_client(0, "AUTO.LAND")
            if getattr(resp, "mode_sent", False):
                rospy.loginfo("[SM] 已切换 AUTO.LAND，开始降落")
                self._land_requested = True
                return True
            rospy.logwarn("[SM] AUTO.LAND 切换未被接受")
        except rospy.ServiceException as e:
            rospy.logerr("[SM] AUTO.LAND 调用失败: %s", e)

        return False

    # -----------------------------------------------------------------------
    # 主循环
    # -----------------------------------------------------------------------
    def run(self):
        st      = Int8()
        st.data = self.state
        self.state_pub.publish(st)

        # ── ST_IDLE ─────────────────────────────────────────────────────────
        if self.state == ST_IDLE:
            if not self._super_off_sent:
                self.super_change(False)
                self._super_off_sent = True

            if self.is_close(self.odom, TAKEOFF_POINT,
                             self.takeoff_xy_tol, self.takeoff_z_tol):
                rospy.loginfo("[SM] 已到达起飞高度，开始巡航建图")
                self.state    = ST_CRUISE
                self.wp_index = 0
                return

            self._try_offboard_and_arm()
            if self.mavros_state.mode == "OFFBOARD" and self.mavros_state.armed:
                self.go_to(TAKEOFF_POINT)
            return

        # ── ST_CRUISE ────────────────────────────────────────────────────────
        if self.state == ST_CRUISE:
            tgt = WAYPOINTS[self.wp_index]
            self.go_to(tgt)

            # ── 提前终止判断（在每帧都检查，不只在到达航点时）──────────────────
            # 条件：地图成熟度满足 AND 尚未触发过 AND 已拿到一个候选降落点
            if (not self._map_mature_triggered
                    and self._land_target is not None
                    and self._is_map_mature()):
                self._map_mature_triggered = True
                rospy.logwarn(
                    "[SM] ✅ 地图已成熟（cells≥%d 且 %.1fs 内稳定），"
                    "提前终止巡航，进入 safeland 确认",
                    self.map_mature_min_cells, self.map_stable_window)
                self._enter_wait_safeland("地图成熟提前终止")
                return
            # ─────────────────────────────────────────────────────────────────

            if self.is_close(self.odom, tgt,
                             self.cruise_xy_tol, self.cruise_z_tol):
                rospy.loginfo("[SM] 到达航点 %d/%d: %s",
                              self.wp_index + 1, len(WAYPOINTS), tgt)
                if self.wp_index >= len(WAYPOINTS) - 1:
                    self._enter_wait_safeland("所有航点完成")
                else:
                    self.wp_index += 1
            return

        # ── ST_WAIT_SAFELAND ─────────────────────────────────────────────────
        if self.state == ST_WAIT_SAFELAND:
            last_wp = WAYPOINTS[-1]
            self.go_to(last_wp)   # 在最后一个航点附近原地悬停

            elapsed = time.time() - self._wait_start_time

            # ---- 有候选点 + 确认等待期已过 → 飞往降落点 ----
            if self._land_target is not None:
                # safeland_confirm_secs 确认等待：
                # 计时基准是「最后一次收到有效降落目标」的时刻（_safeland_recv_time），
                # 而非「进入 ST_WAIT_SAFELAND」的时刻（_wait_start_time）。
                # 原因：若无人机等了很久才收到第一帧结果，elapsed 已远超 confirm_secs，
                # 第一帧结果会被立即采用，没有真正等待 safeland 的多帧刷新确认。
                # 改为从收到目标后计时，确保至少经历 confirm_secs 秒的刷新窗口。
                recv_age = (rospy.Time.now() - self._safeland_recv_time).to_sec() \
                           if self._safeland_recv_time is not None else 0.0
                if recv_age >= self.safeland_confirm_secs:
                    rospy.loginfo(
                        "[SM] ✅ 最优降落点 (%.2f,%.2f) 已确认（目标稳定 %.1fs），飞往目标",
                        self._land_target[0], self._land_target[1], recv_age)
                    self._start_final_approach(self._land_target, "safeland")
                    return
                else:
                    rospy.loginfo_throttle(1.0,
                        "[SM] 候选点已有，等待确认（目标稳定 %.1f/%.1fs）...",
                        recv_age, self.safeland_confirm_secs)
                    return

            # ---- 超时回退 ----
            if elapsed > self.safeland_timeout:
                backup = self._select_backup_landing_target()
                if backup is not None:
                    rospy.logwarn(
                        "[SM] ⚠️ 等待 safeland 超时 (%.0fs)，切换到最近备降区",
                        self.safeland_timeout)
                    self._start_final_approach(backup, "backup")
                else:
                    rospy.logwarn(
                        "[SM] ⚠️ 等待 safeland 超时且无备降区配置，回退到最后航点原地降落")
                    last = WAYPOINTS[-1]
                    self._start_final_approach(
                        [last[0], last[1], self.safeland_land_z + 2.5, last[3]],
                        "last_waypoint")
            else:
                rospy.loginfo_throttle(3.0,
                    "[SM] 等待 safeland 结果... %.0f/%.0fs",
                    elapsed, self.safeland_timeout)
            return

        # ── ST_FLY_TO_LAND ───────────────────────────────────────────────────
        if self.state == ST_FLY_TO_LAND:
            self.go_to(self._land_target)
            if self.is_close(self.odom, self._land_target,
                             self.land_xy_tol, self.land_z_tol):  # 修正: 使用 land_z_tol
                rospy.loginfo("[SM] 已到达降落目标正上方 (%.2f,%.2f)，开始下降",
                              self._land_target[0], self._land_target[1])
                rospy.loginfo("[SM] 降落目标来源: %s", self._land_target_source)
                self.state = ST_LAND
            return

        # ── ST_LAND ──────────────────────────────────────────────────────────
        if self.state == ST_LAND:
            # 进入 ST_LAND 时记录时刻，用于超时保护
            if self._land_enter_time is None:
                self._land_enter_time = time.time()
                rospy.loginfo("[SM] 进入 ST_LAND，超时保护 %.0fs", self.land_timeout_secs)

            hold_pose = [self._land_target[0],
                         self._land_target[1],
                         self._land_target[2],
                         self._land_target[3]]

            if not self._land_requested:
                self.go_to(hold_pose)
                # 修正: 使用 land_z_tol（更精确的高度容差）而非 cruise_z_tol
                if self.is_close(self.odom, hold_pose,
                                 self.land_xy_tol, self.land_z_tol):
                    self._request_land()
                return

            if not self.mavros_state.armed:
                rospy.loginfo("[SM] 飞控已自动上锁，任务完成 ✓")
                self.state = ST_DONE
                return

            p   = self.odom.pose.pose.position
            low = p.z < self.safeland_land_z + 0.12

            if low:
                if not self._present_land_hold:
                    rospy.loginfo("[SM] 已贴近地面，2s 后锁桨")
                    self._present_land_hold = True
                    self._land_disarm_start = time.time()
                elif time.time() - self._land_disarm_start >= 2.0:
                    try:
                        self.arming_client(False)
                        rospy.loginfo("[SM] 电机已锁定，任务完成 ✓")
                    except rospy.ServiceException as e:
                        rospy.logerr("[SM] disarm 失败: %s", e)
                    self.state = ST_DONE
            else:
                # ---- 超时保护：防止飞控 AUTO.LAND 异常导致永久悬停 ----
                # 若 land_timeout_secs 秒内仍未贴地也未上锁，强制 disarm 并完成任务
                land_elapsed = time.time() - self._land_enter_time
                if land_elapsed > self.land_timeout_secs:
                    rospy.logwarn(
                        "[SM] ⚠️ ST_LAND 超时 (%.0fs)，飞控未自动上锁且未贴地，"
                        "强制 disarm 防止卡死", land_elapsed)
                    try:
                        self.arming_client(False)
                    except rospy.ServiceException as e:
                        rospy.logerr("[SM] 强制 disarm 失败: %s", e)
                    self.state = ST_DONE
                else:
                    rospy.loginfo_throttle(3.0,
                        "[SM] 等待下降贴地... z=%.2fm target=%.2fm (%.0f/%.0fs)",
                        p.z, self.safeland_land_z + 0.12,
                        land_elapsed, self.land_timeout_secs)
            return

        # ── ST_DONE ──────────────────────────────────────────────────────────
        if self.state == ST_DONE:
            pass


if __name__ == "__main__":
    sm   = SimpleStateMachine()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        sm.run()
        rate.sleep()
