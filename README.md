# Gendexgrasp_ros
a simpe demo for ros workspace Gendexgrasp_ros

# build
```bash
# 抓取姿态可视化Marker节点 | 目标位置可视化Marker节点 | 抓取ik逆解前姿态可视化 | 发布姿态到ik节点
roslaunch grasp_ik_arm_traj robot_grasp_one_start.launch

# 启动ik节点
roslaunch motion_capture_ik visualize.launch visualize:=true robot_version:=4 control_hand_side:=0 send_srv:=0

# 发布虚拟物体的坐标
cd /home/lab/GenDexGrasp/Gendexgrasp_ros/src/ros_data_process/object_pose_visualizer/scripts
python3 publish_object_pose.py
```

# 请注意（运行环境下的numpy环境的不同）
* 在运行drake-visualizer之前，numpy所需环境为1.20.0
* 但是在运行gendexgrasp的时候，numpy所需的版本为1.24.4
