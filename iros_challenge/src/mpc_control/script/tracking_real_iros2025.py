#! /usr/bin/python3.8

import numpy as np
import casadi as ca
# import time
from std_msgs.msg import Int8

# ROS
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from ius_msgs.msg import Trajectory
# from trajectory_generator.msg import Trajectory

from nav_msgs.msg import Odometry,Path
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Quaternion 

import os
import sys
import time
import math

BASEPATH = os.path.abspath(__file__).split(
    'script', 1)[0]+'script/function_model/'
sys.path += [BASEPATH]

from tracker import TrackerMPC_AC
from quadrotor_control import QuadrotorSimpleModel

state_machine = Int8()
state_machine.data = 0
# 全局变量：用于记录状态13的起始时间
landing_start_time = None
lidar_odom=None
drone_odom=None
# altitude limits (parameterized)
z_min = -0.5
z_max = 4.0
first_odom=None
first_odom_flag=None

class Traj():
    def __init__(self, traj: Trajectory):
        global state_machine
        poss = []
        yaws = []
        ts = []
        #time_plus = 0.0
        #pos_init = traj.pos[0]
        # z_total = 0

        # for i, pos in enumerate(traj.pos):
        #     z_total += pos.z
        for i, pos in enumerate(traj.pos):
            #time_plus += abs(pos.z - pos_init.z)
            #pos_init = pos
            poss.append([pos.x, pos.y, pos.z])
            yaws.append(traj.yaw[i])
            # ts.append(traj.time[i] + time_plus*4.0)
            ts.append(traj.time[i] )
        if len(poss) >= 5:
            if (poss[0][0] - poss[2][0])**2 + (poss[0][1] - poss[2][1])**2 + (poss[0][2] - poss[2][2])**2 < 0.001:
                poss = poss[2:]
                yaws = yaws[2:]
                ts = ts[2:]
                #print(poss)

        self._poss = np.array(poss)
        self._yaws = np.array(yaws)
        self._N = self._poss.shape[0]
        self.t_rcv = traj.header.stamp.to_sec()  # 轨迹接收时间 (ROS时间转换为秒)
        self._ts = np.array(ts)
        if self._N < 2:
            return
        dir = self._poss[1:] - self._poss[: -1]
        # _dir_norm是每个点的方向向量的模
        self._dir_norm = np.linalg.norm(dir, axis=1)+1e-7
        # _u_dir是每个点的方向向量
        self._u_dir = dir/self._dir_norm[:, np.newaxis]
        

    def sample(self, pos, dt, N,use_pos=False):
        # calculate t0 and idx0
        t0 = 0
        idx0 = 0
        if use_pos:
            pos = np.array(pos)
            dl = np.linalg.norm(self._poss - pos, axis=1)  # 求每个点到当前位置的距离
            min_idx = np.argmin(dl)  # 索引的最小值
            print("min_idx",min_idx)
            # 如果min_idx == 0，说明当前位置在第一个点之前，那么t0就是第一个点的时间
            if min_idx == 0:
                idx0 = min_idx
                d_v = pos - self._poss[0]
                u_dir = self._u_dir[0]
                u_t = np.dot(d_v, u_dir) / self._dir_norm[0]
                if u_t < 0:
                    t0 = self._ts[0] #poss在后
                else:
                    t0 = u_t * (self._ts[1] - self._ts[0]) + self._ts[0]
            else:
                idx0 = min_idx - 1
                d_v = pos - self._poss[idx0]
                u_dir = self._u_dir[idx0]
                u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
                if u_t > 1:
                    idx0 = idx0 + 1
                    if idx0 == self._N - 1:
                        t0 = self._ts[-1]
                    else:
                        d_v = pos - self._poss[idx0]
                        u_dir = self._u_dir[idx0]
                        u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
                        if u_t < 0:   #poss在后
                            t0 = self._ts[idx0] 
                        else:
                            t0 = u_t * (self._ts[idx0 + 1] -
                                        self._ts[idx0]) + self._ts[idx0]
                else:
                    t0 = u_t * (self._ts[idx0 + 1] -
                                self._ts[idx0]) + self._ts[idx0]
        else:
            t_now= rospy.Time.now().to_sec()
            t_d_in_tar=t_now-self.t_rcv
            idx0 = 0
            idx0 = np.searchsorted(self._ts, t_d_in_tar) - 1
            idx0 = max(0, min(idx0, self._N - 2))  # 确保索引有效
            t0=t_d_in_tar
            # t0 = self._ts[idx0]  # 对应的轨迹起始时间
        
        
        ts = np.linspace(t0 + dt, t0 + dt * N, N)
        # print(ts)
        idx = idx0
        poss = []
        yaws = []
        for t in ts:
            while idx + 1 < self._N and t > self._ts[idx + 1]:
                idx += 1
            if idx == self._N - 1:
                poss.append(self._poss[-1])
                yaws.append(self._yaws[-1])
                continue
            u_dir = self._u_dir[idx]
            u_t = (t - self._ts[idx]) / (self._ts[idx + 1] - self._ts[idx])
            poss.append(self._poss[idx] + u_t * self._dir_norm[idx] * u_dir)
            if(abs(self._yaws[idx]-self._yaws[idx + 1])>(np.pi/2)):
                yaws.append(self._yaws[idx])
            else:
                yaws.append(self._yaws[idx] + u_t *
                        (self._yaws[idx + 1] - self._yaws[idx]))
  
        # print(poss)
        return np.array(poss), np.array(yaws), ts


def quat_mul(q1: np.array, q2: np.array):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    q = np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])
    return q
# 四元数旋转向量
# q: [qw, qx, qy, qz]
# v: [vx, vy, vz]


def quat_rot_vector(q: np.array, v: np.array):
    q = q / np.linalg.norm(q)
    q_inv = np.array([q[0], -q[1], -q[2], -q[3]])
    qv = quat_mul(quat_mul(q, np.concatenate([[0], v])), q_inv)
    return qv[1:]



def publish_tarj_marker(points,msg,type):
    marker_array = MarkerArray()
    for i, p in enumerate(points):
        point_marker = Marker()
        point_marker.header.frame_id = "world"  # 替换为你的坐标系
        point_marker.header.stamp = rospy.Time.now()#msg.header.stamp
        point_marker.id = i
        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.pose.position.x, point_marker.pose.position.y, point_marker.pose.position.z = p
        point_marker.pose.orientation.w = 1.0  # 默认朝向
        point_marker.scale.x = 0.1  # 点的大小
        point_marker.scale.y = 0.1
        point_marker.scale.z = 0.1
        if type==1:
            point_marker.ns = "tarj_raw"
            point_marker.color.r = 0.0
            point_marker.color.g = 1.0##绿色
            point_marker.color.b = 0.0
            point_marker.color.a = 1.0
            point_marker.scale.x = 0.05  # 点的大小
            point_marker.scale.y = 0.05
            point_marker.scale.z = 0.05
            
        elif type==2:
            point_marker.ns = "tarj_after_sample"
            point_marker.color.r = 1.0
            point_marker.color.g = 1.0##绿色
            point_marker.color.b = 0.0
            point_marker.color.a = 1.0
            

        marker_array.markers.append(point_marker)
    marker_pub.publish(marker_array)
    

def publish_path(positions):
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()

    for pos in positions:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = pos[0]
        pose_stamped.pose.position.y = pos[1]
        pose_stamped.pose.position.z = pos[2]
        pose_stamped.pose.orientation.w = 1.0  # 默认四元数表示无旋转
        path.poses.append(pose_stamped)

    path_pub.publish(path)


def track_traj_cb(msg: Trajectory):
    #global trajectory
    #print(msg)
    #trajectory = Traj(msg)
    #publish_tarj_marker(trajectory._poss,msg,1)
    global trajectory
    if msg.pos:
        trajectory = Traj(msg)
        publish_tarj_marker(trajectory._poss,msg,1)
    else:
        rospy.logwarn("traj.pos is empty")

def state_callback(msg: Int8):
    global state_machine
    state_machine = msg
    
def mavros_state_cb(msg: State):
    global mavros_state
    mavros_state = msg

counter=0
mpc_rate_filter=1  #每接收到 mpc_rate_filter 次消息，执行控制

def odom_cb(msg: Odometry):
    global mavros_state, auto_offboard, state_machine,counter,mpc_rate_filter
    global landing_start_time,lidar_odom,drone_odom
    global first_odom,first_odom_flag
    drone_odom=msg
    # Force odometry frame to world to align with planner/map
    drone_odom.header.frame_id = "world"

    # 计数器增加
    counter += 1
    # 每接收 mpc_rate_filter 次消息，进行一次控制
    if counter >= mpc_rate_filter:
        # 重置计数器
        counter = 0
    else:
        return

    # if mavros_state == None or not mavros_state.connected:
    #     return

    u = AttitudeTarget()
    u.type_mask = AttitudeTarget.IGNORE_ATTITUDE
    u.body_rate.x = 0
    u.body_rate.y = 0
    u.body_rate.z = 0
    u.thrust = 0
    
    # 直接降落
    if state_machine.data == 7:  #备用，防止触发
        if landing_start_time is None:
            landing_start_time = rospy.get_time()  # 记录开始时间

        elapsed = rospy.get_time() - landing_start_time  # 计算经过时间

        # 油门参数
        T0 = 0.3
        # 使得1秒后衰减到约0.01，3.4

        k = 0.9210 #使得5秒衰减到0.01
        

        # 如果时间超过5秒，直接设为0，否则指数衰减
        if elapsed >= 5:
            thrust = 0.0
        else:
            thrust = T0 * math.exp(-k * elapsed)

        # 发布控制指令
        u.body_rate.x = 0
        u.body_rate.y = 0
        u.body_rate.z = 0
        u.thrust = thrust
        setpoint_raw_pub.publish(u)
        print(f"Thrust: {u.thrust:.3f}, Time: {elapsed:.2f}s")
        return
    
    # if auto_offboard and mavros_state.mode != "OFFBOARD":   #关闭自动offboard
    #     set_mode_client.call(0, "OFFBOARD")
    #     setpoint_raw_pub.publish(u)
    #     return
    # if auto_offboard and not mavros_state.armed:
    #     arming_client.call(True)
    #     setpoint_raw_pub.publish(u)
    #     return
    
    # print(len(trajectory._poss) == 0)
    if trajectory != None and len(trajectory._poss) != 0:

        if first_odom is None:
            first_odom=msg
            if first_odom.pose.pose.orientation.w<-0.5:
                first_odom_flag=-1
            else:
                first_odom_flag=1
            print("first_odom_flag:",first_odom_flag)
            return

        time_start = time.time()  # 记录开始时间
        q = np.array([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])*first_odom_flag
        # q = np.array([1,0,0,0])
        v_b = np.array([msg.twist.twist.linear.x,
                       msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        v = quat_rot_vector(q, v_b)  # v in inertial frame
        # w = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        #p的z单独处理
        p = np.array([msg.pose.pose.position.x,
                     msg.pose.pose.position.y, msg.pose.pose.position.z])
        
        if lidar_odom != None:
            if  abs((lidar_odom.header.stamp - msg.header.stamp).to_sec())<0.5:
                if z_min < lidar_odom.pose.pose.position.z < z_max:
                    old_z = p[2]
                    new_z = lidar_odom.pose.pose.position.z
                    p[2] = new_z
                    dt = abs((lidar_odom.header.stamp - msg.header.stamp).to_sec())
                    print(f"reset mpc state z: {old_z:.3f} -> {new_z:.3f} (|dt|={dt:.3f}s)")

        x0 = np.concatenate([p, v, q])

        poss, yaws, ts = trajectory.sample(p, tracker._step, tracker._Herizon,use_pos=False)
        # print("poss:", poss)
        print("yaws:", yaws)
        #print("ts:", ts)
        time_sample = time.time()  # 记录结束时间
        time_sum = time_sample - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
        rospy.logwarn(f"sample time:{time_sum*1000},ms")
        print("##############################################")
        # print(poss)
        # print("q: " ,q)
        us,xs = tracker.solve(x0, poss.reshape(-1), yaws.reshape(-1))
        time_end = time.time()  # 记录结束时间
        time_sum = time_end - time_sample  # 计算的时间差为程序的执行时间，单位为秒/s
        rospy.logwarn(f"solve time:{time_sum*1000},ms")
        
        u_tmp = us.flatten()
        x = xs.flatten()
        Tt = u_tmp[0]
        wx = u_tmp[1]
        wy = u_tmp[2]
        wz = u_tmp[3]
        u.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        u.body_rate.x = wx
        u.body_rate.y = wy
        u.body_rate.z = wz
        u.thrust = min(Tt/quad._a_z_max*0.7, 0.7)
        # u.thrust = 0
        mpc_path_pos = [x[i*10:i*10+3] for i in range(tracker._Herizon)]
   
        # print(us)
     # if state_machine.data < 9:
        if True:
            u.thrust = min(u.thrust, 0.7)
            setpoint_raw_pub.publish(u)
            print("U:",u.thrust, u.body_rate.x, u.body_rate.y, u.body_rate.z)
            
        # print(mpc_path_pos)
        # 发布 Path
        publish_path(mpc_path_pos)
        # publish_tarj_marker(poss,msg,2)
        
    
        time_end = time.time()  # 记录结束时间
        time_sum = time_end - time_start  # 计算的时间差为程序的执行时间，单位为秒/s
        rospy.logwarn(f"odom_cbk time:{time_sum*1000},ms")
    else:
        print("没有轨迹")
        setpoint_raw_pub.publish(u)

def odom_lidar_cb(msg: Odometry):
    global lidar_odom
    lidar_odom=msg
def timer_callback(event):
    global lidar_odom,drone_odom
    if lidar_odom != None and drone_odom!=None :
            if  abs((lidar_odom.header.stamp - drone_odom.header.stamp).to_sec())<0.5:
                if z_min < lidar_odom.pose.pose.position.z < z_max:
                    drone_odom.pose.pose.position.z= lidar_odom.pose.pose.position.z
                    # print("reset mpc state z!!!!!!!!!")
    else:
        return
    # ensure frame id
   # if not drone_odom.header.frame_id:
    #drone_odom.header.frame_id = "world"
    drone_odometry_pub.publish(drone_odom)

     
if __name__ == "__main__":
    trajectory = None
    odom_msg= None
    rospy.init_node("tracking")
    # load altitude limits
    z_min = rospy.get_param('~z_min', z_min)
    z_max = rospy.get_param('~z_max', z_max)

    auto_offboard = rospy.get_param('~auto_offboard', False)
    # mavros/setpoint_raw/attitude 用于发送 姿态 控制指令，包括航向（yaw）、俯仰（pitch）、滚转（roll）角度以及角速度。
    # 这个话题允许用户直接控制无人机的姿态，而不必使用更高级的导航目标或位置控制。
    setpoint_raw_pub = rospy.Publisher(
        "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1, tcp_nodelay=True)

    mavros_state = None
    
    path_pub = rospy.Publisher('/mpc_trajectory_path', Path, queue_size=1)
    marker_pub = rospy.Publisher('/mpc_trajectory_marker', MarkerArray, queue_size=10)
    drone_odometry_pub=rospy.Publisher("/drone_odometry",Odometry, queue_size=1)
    rospy.Subscriber("/mavros/state", State, mavros_state_cb,
                    queue_size=1, tcp_nodelay=True)
    #arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    #set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    quad = QuadrotorSimpleModel(BASEPATH+'quad/quad_real_rm2025.yaml')
    # quad = QuadrotorSimpleModel(BASEPATH+'quad/quad_sim.yaml')
    tracker = TrackerMPC_AC(quad)


    
    rospy.Subscriber("~odom", Odometry, odom_cb, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber("~odom_lidar", Odometry, odom_lidar_cb, queue_size=1, tcp_nodelay=True)
    rospy.Timer(rospy.Duration(1/1000), timer_callback)  # 15Hz 固定检测
    rospy.Subscriber("~track_traj", Trajectory, track_traj_cb,
                    queue_size=1, tcp_nodelay=True)
    rospy.Subscriber('/state_machine', Int8, state_callback)

    rospy.spin()
