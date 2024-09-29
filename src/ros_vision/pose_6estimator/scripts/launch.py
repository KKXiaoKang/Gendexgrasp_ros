import subprocess
import os

def activate_conda_and_run():
    # 切换到指定目录
    target_dir = "/home/lab/GenDexGrasp/Gendexgrasp_ros/src/ros_vision/pose_6estimator"
    os.chdir(target_dir)

    # 运行两个 Python 文件
    contact_service_command = "python3 predict_realsense.py --cfg configs/gen6d_pretrain.yaml --database custom/bottle --output data/custom/bottle/test"

    # 使用 Popen 来并行运行两个命令
    try:
        # 启动 ros_Contact_service.py
        contact_process = subprocess.Popen(contact_service_command, shell=True)
        print("ros_Contact_service.py started")

        # 等待两个进程完成（如果需要，可以添加超时等机制）
        contact_process.wait()

    except subprocess.CalledProcessError as e:
        print(f"Error running the services: {e}")

if __name__ == "__main__":
    activate_conda_and_run()
