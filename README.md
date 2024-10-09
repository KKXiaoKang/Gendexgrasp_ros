# Gendexgrasp_ros
a simpe demo for ros workspace Gendexgrasp_ros

# quick start
```bash
# 抓取姿态可视化Marker节点 | 目标位置可视化Marker节点 | 抓取ik逆解前姿态可视化 | 发布姿态到ik节点 | 创建灵巧手调用服务端
roslaunch grasp_ik_arm_traj robot_grasp_one_start.launch

# 启动ik节点
roslaunch motion_capture_ik visualize.launch visualize:=true robot_version:=4 control_hand_side:=0 send_srv:=0
roslaunch motion_capture_ik visualize.launch control_hand_side:=2 send_srv:=0 eef_z_bias:=-0.15 visualize:=1 enable_quest3:=0 use_cxx:=1 # 新节点

# 发布真实物体的坐标（启动相机 | 启动yolo-onnxruntime）
roslaunch grasp_ik_arm_traj sensor_robot_enable.launch

# 在线生成服务端 -- 发布ros_gendexgrasp服务端
roslaunch ros_gendexgrasp gendexgrasp_ros_service.launch

# 离线生成服务端
rosrun grasp_filter_gendex grasp_filter_node.py

# 发布物体姿态四元数(Gen6D)
cd /home/lab/GenDexGrasp/Gendexgrasp_ros/ros_vision/6DOF_Gen_ros
python3 predict_realsense.py --cfg configs/gen6d_pretrain.yaml --database custom/bottle --output data/custom/bottle/test

# 运行演示demo
cd /home/lab/GenDexGrasp/Gendexgrasp_ros/scripts
python3 demo_offline.py
```

# 请注意（运行环境下的numpy环境的不同）
* 在运行drake-visualizer之前，numpy所需环境为1.20.0
* 但是在运行gendexgrasp的时候，numpy所需的版本为1.24.4

## for water_bottle_grasp
```bash
# 旧：生成抓取图（根据指定的面片索引进行接触图的生成）
python inf_cvae.py --pre_process sharp_lift --s_model PointNetCVAE_SqrtFullRobots --num_per_object 2 --comment leju

# 旧：根据抓取图不断生成抓取姿态ros信息
python run_grasp_gen_ros.py --robot_name lejuhand --max_iter 100 --num_particles 32 --learning_rate 5e-3 --init_rand_scale 0.5 --object_name contactdb+water_bottle --cmap_dir logs_inf_cvae/PointNetCVAE_SqrtFullRobots/sharp_lift/leju

# 新：统一发布ros_gendexgrasp服务端
roslaunch ros_gendexgrasp gendexgrasp_ros_service.launch
```

## 最新启动
```bash
# 引入固定物体的抓取姿态 
source /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/devel/setup.bash

# 启动ik
roslaunch motion_capture_ik visualize.launch visualize:=true robot_version:=4 control_hand_side:=0 send_srv:=0

# 启动姿态框架
roslaunch grasp_ik_arm_traj all_in_one.launch

# 启动演示脚本
cd /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/scripts
python3 demo_offline.py
```