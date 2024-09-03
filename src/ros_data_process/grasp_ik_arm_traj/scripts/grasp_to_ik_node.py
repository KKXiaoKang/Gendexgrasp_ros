#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from motion_capture_ik.msg import twoArmHandPose, armHandPose
from visualization_msgs.msg import Marker
import numpy as np

class GraspToIK:
    def __init__(self):
        # 订阅最佳抓取姿态
        self.grasp_sub = rospy.Subscriber('/best_grasp_pose', PoseStamped, self.grasp_callback)
        
        # 订阅机器人轨迹
        self.traj_sub = rospy.Subscriber('/kuavo_arm_traj', JointState, self.traj_callback)

        # 发布 IK 命令
        self.ik_pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPose, queue_size=10)
        
        # 发布机器人的关节状态
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # 发布 Marker
        self.marker_pub = rospy.Publisher('/ik_before_visualization_marker', Marker, queue_size=10)

        # 模型路径
        self.model_path = "/home/lab/GenDexGrasp/Gendexgrasp_ros/src/ros_robot_model/biped_s4/meshes/l_hand_roll.STL"

        # 增加真实的物体的姿态
        self.object_real_sub = rospy.Subscriber("object_visualization_marker", Marker, self.object_real_callback)
        self.object_real_msg_info = Marker()

    def object_real_callback(self, msg):
        # 接收到物体的实际位置
        # rospy.loginfo("Received a new object real pose")
        
        # 构建物体实际位置的数组
        self.object_real_msg_info.pose = msg.pose
        self.object_real_msg_info.header = msg.header

    def grasp_callback(self, msg):
        # 归一化四元数
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        norm = np.linalg.norm(quat)
        if norm > 0:
            quat /= norm
        else:
            rospy.logerr("四元数的模为零！")

        # 初始化 IK 命令消息
        ik_msg = twoArmHandPose()

        # 假设姿态是针对右手的
        final_grasp_pose_x = self.object_real_msg_info.pose.position.x + msg.pose.position.x
        final_grasp_pose_y = self.object_real_msg_info.pose.position.y + msg.pose.position.y
        final_grasp_pose_z = self.object_real_msg_info.pose.position.z + msg.pose.position.z

        ik_msg.left_pose.pos_xyz = [final_grasp_pose_x, final_grasp_pose_y, final_grasp_pose_z]
        ik_msg.left_pose.quat_xyzw = quat.tolist()

        # 简单起见，将肘部位置和关节角度设置为零
        ik_msg.left_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        ik_msg.left_pose.joint_angles = [0.0] * 7

        # 如果需要，可以选择填充 left_pose
        ik_msg.right_pose.pos_xyz = [0.0, 0.0, 0.0]
        ik_msg.right_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
        ik_msg.right_pose.elbow_pos_xyz = [0.0, 0.0, 0.0]
        ik_msg.right_pose.joint_angles = [0.0] * 7

        # 发布 IK 命令
        rospy.loginfo("基于最佳抓取姿态发布 IK 命令")
        self.ik_pub.publish(ik_msg)

        # 创建并发布 Marker
        marker = Marker()
        marker.header.frame_id = "torso"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "file://" + self.model_path

        marker.pose.position.x = ik_msg.left_pose.pos_xyz[0] 
        marker.pose.position.y = ik_msg.left_pose.pos_xyz[1] 
        marker.pose.position.z = ik_msg.left_pose.pos_xyz[2] 

        marker.pose.orientation.x = ik_msg.left_pose.quat_xyzw[0]
        marker.pose.orientation.y = ik_msg.left_pose.quat_xyzw[1]
        marker.pose.orientation.z = ik_msg.left_pose.quat_xyzw[2]
        marker.pose.orientation.w = ik_msg.left_pose.quat_xyzw[3]

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.5

        rospy.loginfo("发布 IK 前的 Marker")
        self.marker_pub.publish(marker)

    def traj_callback(self, msg):
        # 重新发布关节状态到 joint_state_publisher
        rospy.loginfo("重新发布关节状态轨迹")
        self.joint_state_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('grasp_to_ik_node', anonymous=True)
    node = GraspToIK()
    rospy.spin()
