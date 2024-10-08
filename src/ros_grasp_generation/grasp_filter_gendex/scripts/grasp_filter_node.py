#!/usr/bin/env python

import rospy
import os
import rospkg
import yaml
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_matrix, euler_from_quaternion
from grasp_filter_gendex.srv import offlineGraspButton, offlineGraspButtonResponse, offlineGraspButtonRequest
import random

# 加载配置文件
rospack = rospkg.RosPack()

class GraspFilterNode:
    def __init__(self):
        rospy.init_node('grasp_filter_node')
        
        # 初始化标志位
        self.START_STOP_PUB_FLAG = False  # 默认关闭
        self.IK_SUCCESS_FLAG = False  # 默认关闭
        
        # Load grasp data from config.yaml
        self.config_file = os.path.join(rospack.get_path('grasp_filter_gendex'), 'scripts/config/config.yaml')
        with open(self.config_file, 'r') as file:
            self.grasp_data = yaml.safe_load(file)
        rospy.loginfo(f"Loaded grasp data from {self.config_file}")
        rospy.loginfo(f"Number of grasps: {len(self.grasp_data)}")
        
        # Publishers
        self.pose_pub = rospy.Publisher('/best_grasp_pose', PoseStamped, queue_size=10)
        self.joint_pub = rospy.Publisher('/best_hand_pos', JointState, queue_size=10)

        # Subscribers
        self.ik_status_sub = rospy.Subscriber("/ik_solver_status", Int32, self.IK_status_callback)

        # Set the publish rate to 10Hz
        self.publish_rate = rospy.Rate(10)

        # Iterator to cycle through grasp data
        self.grasp_keys = iter(self.grasp_data)

        # 创建服务
        self.grasp_service = rospy.Service('/offline_grasp_service', offlineGraspButton, self.handle_grasp_service)

        # 姿态筛选器开关
        self.POSE_FILTER_FLAG = True  # 默认关闭姿态筛选
        
        # # 随机开始遍历索引
        # self.start_index = random.randint(0, len(self.grasp_data) - 1)
        # self.current_index = self.start_index  # 当前索引

    def IK_status_callback(self, msg):
        """
        监听 IK 求解状态，只有在 START_STOP_PUB_FLAG 为 True 时判断 IK 是否成功。
        如果 IK 求解成功（msg.data == 1），将 START_STOP_PUB_FLAG 置为 False，停止发布话题。
        """
        if self.START_STOP_PUB_FLAG:  # 只有在标志位为 True 时才处理 IK 状态
            if msg.data == 1:  # IK 求解成功
                rospy.loginfo("IK solver succeeded, stopping grasp publishing.")
                self.START_STOP_PUB_FLAG = False  # 停止发布抓取数据
                self.IK_SUCCESS_FLAG = True  # 表示 IK 求解成功
            else:
                # rospy.loginfo("IK solver failed.")
                pass

    def handle_grasp_service(self, req):
        """
        处理服务请求，控制 START_STOP_PUB_FLAG 标志位。
        """
        if req.data == 1:
            self.START_STOP_PUB_FLAG = True
            rospy.loginfo("Grasp topic publishing started.")
        elif req.data == 0:
            self.START_STOP_PUB_FLAG = False
            rospy.loginfo("Grasp topic publishing stopped.")
        
        # 返回服务响应，通知操作是否成功
        return offlineGraspButtonResponse(success=True)

    def best_q_to_posestamped(self, best_q, frame_id="torso"):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = frame_id
        translation = np.array(best_q[:3])
        rotation_matrix_components = np.array(best_q[3:9])
        rotation_matrix = np.zeros((3, 3))
        rotation_matrix[:, 0] = rotation_matrix_components[:3]
        rotation_matrix[:, 1] = rotation_matrix_components[3:6]
        rotation_matrix[:, 1] -= np.dot(rotation_matrix[:, 0], rotation_matrix[:, 1]) * rotation_matrix[:, 0]
        rotation_matrix[:, 1] /= np.linalg.norm(rotation_matrix[:, 1])
        rotation_matrix[:, 2] = np.cross(rotation_matrix[:, 0], rotation_matrix[:, 1])
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = rotation_matrix
        quaternion = quaternion_from_matrix(homogeneous_matrix)
        pose_msg.pose.position.x = translation[0]
        pose_msg.pose.position.y = translation[1]
        pose_msg.pose.position.z = translation[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        return pose_msg

    def best_q_to_posehand(self, best_q, frame_id="torso"):
        joint_names = [
            "base_link", "thumb1", "thumb2", "index1", "index2",
            "middle1", "middle2", "ring1", "ring2", "little1", "little2"
        ]
        joint_positions = np.array(best_q[9:])
        robot_hand_joint_msg = JointState()
        robot_hand_joint_msg.header.stamp = rospy.Time.now()
        robot_hand_joint_msg.header.frame_id = frame_id
        robot_hand_joint_msg.name = joint_names
        robot_hand_joint_msg.position = joint_positions
        return robot_hand_joint_msg

    def get_euler_angles(self, pose):
        # 从PoseStamped提取四元数
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )

        # 转换为欧拉角
        euler_angles = euler_from_quaternion(quaternion)
        return euler_angles  # 返回 (roll, pitch, yaw)
    
    def is_pose_valid(self, pose, key):
        euler_angles = self.get_euler_angles(pose)
        roll, pitch, yaw = euler_angles

        # 检查roll是否在指定范围内（-20度到90度）
        if not (-75 * np.pi / 180 <= roll <= 75 * np.pi / 180):  # 将度数转换为弧度
            return False

        # # 检查yaw是否符合范围
        # if not (-110 * np.pi / 180 <= roll <= 110 * np.pi / 180):  # 将度数转换为弧度
        #     return False
        
        # 检索角度
        rospy.loginfo(f"Pose {key} is {euler_angles}.")
        return True
    
    def publish_grasp_data(self):
        if not self.START_STOP_PUB_FLAG:
            return  # 当标志位为 False 时，不发布数据

        try:
            key = next(self.grasp_keys)
        except StopIteration:
            self.grasp_keys = iter(self.grasp_data)
            key = next(self.grasp_keys)
        
        # 赋值设置best_q
        best_q_list = self.grasp_data[key]
        best_q = best_q_list[0]
        pose_msg = self.best_q_to_posestamped(best_q)
        joint_msg = self.best_q_to_posehand(best_q)

        # 检查姿态筛选开关
        if self.POSE_FILTER_FLAG:
            if not self.is_pose_valid(pose_msg, key):
                # rospy.logwarn("Pose does not meet the filter criteria, skipping.")
                return  # 如果姿态不符合规则，直接返回
        # 符合规则，发布数据
        self.pose_pub.publish(pose_msg)
        self.joint_pub.publish(joint_msg)
        # 打印符合规则的key
        rospy.loginfo(f"Published key: {key}")

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_grasp_data()  # 只有在 START_STOP_PUB_FLAG 为 True 时才会发布
            self.publish_rate.sleep()  # This enforces the 10Hz rate

if __name__ == "__main__":
    try:
        node = GraspFilterNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
