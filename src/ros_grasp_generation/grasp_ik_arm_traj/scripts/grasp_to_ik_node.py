import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from motion_capture_ik.msg import twoArmHandPose, armHandPose
from visualization_msgs.msg import Marker
import numpy as np
import math

class GraspToIK:
    def __init__(self):
        # 订阅最佳抓取姿态
        self.grasp_sub = rospy.Subscriber('/best_grasp_pose', PoseStamped, self.grasp_callback)
        
        # 订阅机器人轨迹
        self.traj_sub = rospy.Subscriber('/kuavo_arm_traj', JointState, self.traj_callback)
        self.robot_arm_traj = JointState()

        # 订阅手部状态
        self.hand_pose_sub = rospy.Subscriber('/best_hand_pos', JointState, self.hand_pose_callback)
        self.hand_pose = JointState()

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

        # 初始化joint_state变量
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.define_joint_names()
        self.joint_state_msg.position = [0.0] * len(self.joint_state_msg.name)

        # 50Hz频率发布JointState
        rospy.Timer(rospy.Duration(0.02), self.publish_joint_states)

    def define_joint_names(self):
        return [
            # 左臂和左手
            "l_arm_pitch", "l_arm_roll", "l_arm_yaw", 
            "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", 
            "l_hand_roll",
            # 左手手指
            "l_thumb_proximal_yaw", "l_thumb_distal_pitch", 
            "l_index_proximal_finger", "l_index_distal_finger", 
            "l_middle_proximal_finger", "l_middle_distal_finger", 
            "l_ring_proximal_finger", "l_ring_distal_finger", 
            "l_pinky_proximal_finger", "l_pinky_distal_finger",
            # 右臂和右手
            "r_arm_pitch", "r_arm_roll", "r_arm_yaw", 
            "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", 
            "r_hand_roll",
            # 剩余关节名称
            "r_thumb_proximal_yaw", "r_thumb_distal_pitch",
            "r_index_proximal_finger", "r_index_distal_finger",
            "r_middle_proximal_finger", "r_middle_distal_finger",
            "r_ring_proximal_finger", "r_ring_distal_finger",
            "r_pinky_proximal_finger", "r_pinky_distal_finger",
            "l_leg_roll", "l_leg_yaw", "l_leg_pitch", "l_knee",
            "l_foot_pitch", "l_foot_roll", "l_l_bar", "l_l_tendon",
            "l_r_bar", "l_r_tendon", "r_leg_roll", "r_leg_yaw",
            "r_leg_pitch", "r_knee", "r_foot_pitch", "r_foot_roll",
            "r_l_bar", "r_l_tendon", "r_r_bar", "r_r_tendon",
            "head_yaw", "head_pitch"
        ]
    
    def hand_pose_callback(self, msg):
        # 接收到手部状态
        rospy.loginfo("Received a new hand 灵巧手 pose")
        self.hand_pose = msg

    def object_real_callback(self, msg):
        # 接收到物体的实际位置
        self.object_real_msg_info.pose = msg.pose
        self.object_real_msg_info.header = msg.header

    def traj_callback(self, msg):
        if self.hand_pose is None:
            rospy.logwarn("Hand pose not received yet. Skipping this cycle.")
            return

        # 提取左臂和右臂的关节角度
        if len(msg.position) >= 14:
            left_arm_positions = list(msg.position[:7])
            right_arm_positions = list(msg.position[7:14])

            # TODO: 角度转弧度
            left_arm_positions = [math.radians(angle) for angle in left_arm_positions]
            right_arm_positions = [math.radians(angle) for angle in right_arm_positions]

        else:
            rospy.logerr("Received joint trajectory does not have enough positions. Expected at least 14, got %d", len(msg.position))
            return

        # 提取左手手指的关节弧度
        if len(self.hand_pose.position) >= 10:
            left_finger_positions = list(self.hand_pose.position[:10])
            print("left_finger_positions : ", left_finger_positions)
        else:
            rospy.logerr("Received hand pose does not have enough positions. Expected at least 10, got %d", len(self.hand_pose.position))
            return

        # 更新 joint_state_msg 的位置
        positions = left_arm_positions + left_finger_positions + right_arm_positions
        remaining_joints = len(self.joint_state_msg.name) - len(positions)
        positions.extend([0.0] * remaining_joints)
        self.joint_state_msg.position = positions

    def publish_joint_states(self, event):
        # 发布 JointState 消息
        # rospy.loginfo("Publishing joint state at 50Hz")
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state_msg)

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

        # 如果需要，可以选择填充 right_pose
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

if __name__ == '__main__':
    rospy.init_node('grasp_to_ik_node', anonymous=True)
    node = GraspToIK()
    rospy.spin()
