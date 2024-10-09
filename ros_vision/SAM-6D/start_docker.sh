# # 先杀掉并移除现有的 "model" 容器
docker stop sam6d
docker rm sam6d

# 运行 Docker 容器
docker run -it --gpus all --runtime=nvidia --name sam6d --network host \
    -v /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/ros_vision/SAM-6D:/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/ros_vision/SAM-6D \
    -e ROS_MASTER_URI=http://192.168.0.147:11311 \
    -e ROS_IP=192.168.0.147 \
    --rm --workdir /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/ros_vision/SAM-6D \
    lihualiu/sam-6d:1.0 /bin/bash

# 设置 ROS 环境变量
export ROS_IP=192.168.0.147
export ROS_MASTER_URI=http://192.168.0.147:11311
