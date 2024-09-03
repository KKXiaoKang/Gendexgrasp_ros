import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from motion_capture_ik.msg import twoArmHandPose, armHandPose
from visualization_msgs.msg import Marker
import numpy as np
import math
from dynamic_biped.msg import robotHandPosition
from hand_sdk_control.srv import handPosService, handPosServiceResponse, handPosServiceRequest

GLOBAL_IK_SUCCESS = False

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
        
        # TODO:增加灵巧手实物控制服务
        rospy.wait_for_service('/hand_sdk_control_service')
        self.hand_control_client = rospy.ServiceProxy('/hand_sdk_control_service', handPosService)       

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

    def send_hand_control_service(self, hand_positions):
        try:
            """
                客户端传入手指的弧度值，发送给服务端，服务端控制会自动做角度的转换，并且控制灵巧手
            """
            # 创建服务请求对象
            request = handPosServiceRequest()

            # 构建 Header
            request.header = rospy.Header()
            request.header.stamp = rospy.Time.now()

            # 将手指弧度值转换为角度
            hand_positions_degrees = [math.degrees(pos) for pos in hand_positions]
            
            # 选择 hand_positions 中的第1, 2, 3, 5, 7, 9 作为控制维度
            selected_positions = [int(hand_positions_degrees[i]) for i in [0, 1, 2, 4, 6, 8]]

            # 将 selected_positions 中的值限制在 0 到 255 之间
            selected_positions = [max(0, min(255, pos)) for pos in selected_positions]
            print(" selected_positions : ", selected_positions)

            # 设置 left_hand_position 和 right_hand_position
            request.left_hand_position = selected_positions
            request.right_hand_position = [0] * 6  # 假设右手没有动作

            # 调用服务并处理响应
            response = self.hand_control_client(request)
            if response.result:
                rospy.loginfo("Hand control command successfully sent to the real hand controller.")
            else:
                rospy.logwarn("Failed to send hand control command.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))

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
        """
            brief: 用于 发布服务灵巧手服务实物可以进行控制 | 重写joint_state将手臂和灵巧手的可视化结果发布出来
            param msg: 机器人轨迹
        """
        # 接收到机器人轨迹 | 代表IK逆解成功
        rospy.loginfo("Received a new robot arm trajectory | IK Success!")
        
        # 寻找该ik下的hand_pose是不是正确的 
        if self.hand_pose is None:
            rospy.logwarn("Hand pose not received yet. Skipping this cycle.")
            return

        # 提取左臂和右臂的关节角度
        if len(msg.position) >= 14:
            left_arm_positions = list(msg.position[:7])
            right_arm_positions = list(msg.position[7:14])

            # 角度转弧度
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