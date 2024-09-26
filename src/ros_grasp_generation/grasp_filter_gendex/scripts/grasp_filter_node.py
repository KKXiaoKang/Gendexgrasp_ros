#!/usr/bin/env python

import rospy
import os
import rospkg
import yaml
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix

# 加载配置文件
rospack = rospkg.RosPack()

# 标志位
START_STOP_PUB_FLAG = False # 默认关闭，当True开始发布话题，False停止发布话题
IK_SUCCESS_FLAG = False # 默认关闭，当True表示IK求解成功，False表示IK求解失败, 求解成功之后直接停止发布话题

class GraspFilterNode:
    def __init__(self):
        rospy.init_node('grasp_filter_node')
        
        # Load grasp data from config.yaml
        self.config_file = os.path.join(rospack.get_path('grasp_filter_gendex'), 'scripts/config/config.yaml')
        with open(self.config_file, 'r') as file:
            self.grasp_data = yaml.safe_load(file)
        rospy.loginfo(f"Loaded grasp data from {self.config_file}")
        rospy.loginfo(f"Number of grasps: {len(self.grasp_data)}")
        
        # Publishers
        self.pose_pub = rospy.Publisher('/best_grasp_pose', PoseStamped, queue_size=10)
        self.joint_pub = rospy.Publisher('/best_hand_pos', JointState, queue_size=10)

        # Set the publish rate to 10Hz
        self.publish_rate = rospy.Rate(10)

        # Iterator to cycle through grasp data
        self.grasp_keys = iter(self.grasp_data)

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

    def publish_grasp_data(self):
        try:
            key = next(self.grasp_keys)
        except StopIteration:
            self.grasp_keys = iter(self.grasp_data)
            key = next(self.grasp_keys)
        
        best_q_list = self.grasp_data[key]
        best_q = best_q_list[0]
        pose_msg = self.best_q_to_posestamped(best_q)
        joint_msg = self.best_q_to_posehand(best_q)

        self.pose_pub.publish(pose_msg)
        self.joint_pub.publish(joint_msg)
        # rospy.loginfo(f"Published grasp data for {key}")

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_grasp_data()
            self.publish_rate.sleep()  # This enforces the 10Hz rate

if __name__ == "__main__":
    try:
        node = GraspFilterNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
