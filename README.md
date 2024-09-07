# Gendexgrasp_ros
a simpe demo for ros workspace Gendexgrasp_ros

# build
```bash
# 抓取姿态可视化Marker节点 | 目标位置可视化Marker节点 | 抓取ik逆解前姿态可视化 | 发布姿态到ik节点 | 创建灵巧手调用服务端
roslaunch grasp_ik_arm_traj robot_grasp_one_start.launch

# 启动ik节点
roslaunch motion_capture_ik visualize.launch visualize:=true robot_version:=4 control_hand_side:=0 send_srv:=0

# 发布虚拟物体的坐标
cd /home/lab/GenDexGrasp/Gendexgrasp_ros/src/ros_pose_visualizer/object_pose_visualizer/scripts
python3 publish_object_pose.py

# 发布真实物体的坐标（启动相机 | 启动yolo-onnxruntime）
roslaunch grasp_ik_arm_traj sensor_robot_enable.launch
```

# 请注意（运行环境下的numpy环境的不同）
* 在运行drake-visualizer之前，numpy所需环境为1.20.0
* 但是在运行gendexgrasp的时候，numpy所需的版本为1.24.4

## for water_bottle_grasp
```bash
# 生成抓取图（根据指定的面片索引进行接触图的生成）
python inf_cvae.py --pre_process sharp_lift --s_model PointNetCVAE_SqrtFullRobots --num_per_object 2 --comment leju

# 根据抓取图不断生成抓取姿态ros信息
python run_grasp_gen_ros.py --robot_name lejuhand --max_iter 100 --num_particles 32 --learning_rate 5e-3 --init_rand_scale 0.5 --object_name contactdb+water_bottle --cmap_dir logs_inf_cvae/PointNetCVAE_SqrtFullRobots/sharp_lift/leju
```