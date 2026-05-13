###启动仿真环境（飞机雷达安装改为朝下向前）
--需要将livox_custom.launch中加载.world和.sdf的绝对路径改为自己的路径
--将nagetive_terrain这个文件放到.gazebo文件夹的模型路径下
cd PX4_Firmware
roslaunch px4 livox_custom.launch



###启动感知、导航、控制器和状态机
cd iros_challenged
./bot.sh
或者下面四条命令
roslaunch fast_lio mapping_mid360.launch
roslaunch mpc_control control_sim_iros2025.launch
roslaunch super_planner iros_real.launch
roslaunch state_machine iros_state_machine_simple_sim.launch



###启动QG
cd doc
./QGroundControl.AppImage



###点云处理
cd /costmap_ws
// 将fast lio发布的/cloud_registered点降采样后转换为栅格地图，地图分为三层结构：高度层，z_sum和点数count，可在src/fast_lio_global_grid_map/config/global_grid_map.yaml调整参数，有相应的注释
roslaunch fast_lio_global_grid_map global_grid_map.launch
// 计算栅格的坡度、粗糙度和阶梯，并设置相应的阈值，筛选出满足栅格内有效点数足够多、坡度、粗糙度和阶梯均合理的区域，可在src/safeland/config/safeland.yaml这个参数文件中调整参数，有相应注释
roslaunch safeland safeland.launch
