# # 先杀掉并移除现有的 "model" 容器
docker stop kuavo_gendexgrasp_dev
docker rm kuavo_gendexgrasp_dev

# 运行 Docker 容器
docker run -it --gpus all --runtime=nvidia --name kuavo_gendexgrasp_dev --network host \
    -v /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/:/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/ \
    -e ROS_MASTER_URI=http://192.168.0.147:11311 \
    -e ROS_IP=192.168.0.147 \
    --rm --workdir /home/lab/GenDexGrasp/Gendexgrasp_ros_ok/ \
    kuavo/gendexgrasp-dev:v1.0  /bin/bash

# 设置 ROS 环境变量
export ROS_IP=192.168.0.147
export ROS_MASTER_URI=http://192.168.0.147:11311
