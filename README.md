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

```bash
contactdb+water_bottle_11_best_q
contactdb+water_bottle_23_best_q
contactdb+water_bottle_47_best_q
contactdb+water_bottle_126_best_q
contactdb+water_bottle_170_best_q
contactdb+water_bottle_197_best_q
```

## docker
```bash
# 启动docker并且挂载
docker run -it -v /home/lab/GenDexGrasp/Gendexgrasp_ros_ok:/home/lab/GenDexGrasp/Gendexgrasp_ros_ok ubuntu:20.04 /bin/bash 

# 提交
docker commit 184c388c6b97 kuavo/gendexgrasp-dev:v1.0

# 重新开启kuavo/gendexgrasp-dev:v1.0 挂载
docker run -it -v /home/lab/GenDexGrasp/Gendexgrasp_ros_ok:/home/lab/GenDexGrasp/Gendexgrasp_ros_ok kuavo/gendexgrasp-dev:v1.0 /bin/bash 
```

## docker - CHANGELOG
```bash
# 环境配置
v1.0 : 
    1. 基础ubuntu20.04

# 环境配置
v2.0 :
    1. ros1 noeitc/ ros2 foxy
    2. vim
    3. x11 host / rviz 映射外部

# 环境配置
v3.0 :
    ## for kuavo_opensource 
    1. ros noetic : apriltag-ros | moveit | trac-ik
    2. apt-get install python3-pip
    3. pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
    4. pip install rospy-message-converter
    5. apt install git
    6. cassie_alip_mpc : https://github.com/UMich-BipedLab/cassie_alip_mpc.git
    7. sudo apt-get install liblcm-dev libgflags-dev libgoogle-glog-dev liblmdb-dev
    8. drake : https://drake.mit.edu/apt.html | sudo apt install drake-dev=1.19.0-1
    9. sudo apt-get install libncurses5-dev libncursesw5-dev
    10. sudo apt-get install libprotobuf-c-dev

# 环境配置
v4.0 : （在不使用姿态估计和Gendexgrasp生成的姿态下，已经基本按特定的姿态跑通demo）
    ## for ros-noetic-binary install
    1. vision_msgs | apt-get install ros-noetic-vision*
    2. rviz_visual_tools | apt-get install ros-noetic-rviz*
    3. realsense2_camera | apt-get install ros-noetic-realsense2*
    4. tf2_ros | apt-get install ros-noetic-tf2*

    ## for ros-foxy-binary install | rs-enumerate-devices
    1. realsense2 | sudo apt-get install ros-foxy-realsense*

    ## for torch - gpu
    1. pip3 install networkx
    2. pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

    ## for onnxruntime-gpu | yolov5
    1. pip install onnxruntime-gpu

    ## for yolov5
    1. pip3 install -r requirements.txt

    ## for motion_capture_ik
    1. pip3 install numpy-quaternion

    ## for demo_offline.py
    1. pip3 install rich

# 环境配置
v5.0 :
    ## 配置anaconda
    # default python path 
    export PATH=/usr/bin:$PATH
    1. bash Anaconda3-2024.10-1-Linux-x86_64.sh -b -p /opt/anaconda 
    2. export LC_ALL=C.UTF-8 | export LANG=C.UTF-8
    /opt/anaconda/bin/conda init
    conda activate

    ## for Gendexgrasp
    ### 快速 copy anaconda 环境
    1. conda env export --name gendexgrasp > gendexgrasp.yml
    ### 快速在另外一台机器上运行
    2. conda env create -f gendexgrasp.yml
    3. conda activate gendexgrasp
    4. pip3 install trimesh
    5. pip3 install plotly 
    6. pip3 install scipy
    7. pip3 install matplotlib
    8. pip3 install pytorch-kinematics
    9. pip3 install transforms3d
    10. pip3 install open3d
    11. pip3 install tensorboard
    12. pip3 install rospkg

    ## for Gen-6D-Pose-Estimation
    ### need to download VGG11_BN_Weights.DEFAULT into default torch
    1. python3 predict_realsense.py --cfg configs/gen6d_pretrain.yaml --database custom/bottle --output data/custom/bottle/test
    
```