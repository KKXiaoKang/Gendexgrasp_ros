#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker

class GraspPoseVisualizer:
    def __init__(self):
        rospy.init_node('grasp_pose_visualizer', anonymous=True)

        # 创建Publisher用于发布Marker
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

        # 订阅/best_grasp_pose话题
        self.pose_sub = rospy.Subscriber("/best_grasp_pose", PoseStamped, self.pose_callback)
        self.model_path = "/home/lab/GenDexGrasp/Gendexgrasp_ros/src/ros_robot_model/biped_s4"

    def pose_callback(self, msg):
        # 接收到位置 
        rospy.loginfo("Received a new grasp pose")

        # 提取位置和姿态数据
        position = msg.pose.position
        orientation = msg.pose.orientation

        # 构建并发布Marker
        # marker = self.construct_marker(
        #     [position.x, position.y, position.z],
        #     [orientation.x, orientation.y, orientation.z, orientation.w],
        #     1.0, 0.0, 0.0, "Right"
        # )
        marker = self.construct_marker(
            [(0.6 + position.x), (0.1 + position.y), (0.0 + position.z)], # 先给一个固定位置（测试抓取姿态生成稳定性）
            [orientation.x, orientation.y, orientation.z, orientation.w],
            1.0, 0.0, 0.0, "Left"
        )
        if marker:
            self.marker_pub.publish(marker)

    def construct_marker(self, arm_pose_p, arm_pose_q, r, g, b, side):
        if len(arm_pose_q) != 4 or len(arm_pose_p) != 3:
            rospy.logerr("Invalid arm pose, cannot construct marker")
            return None

        marker = Marker()
        marker.header.frame_id = "torso"
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        if side == "Left":
            marker.mesh_resource = (
                "file://" + self.model_path + "/meshes/l_hand_roll.STL"
            )
        elif side == "Right":
            marker.mesh_resource = (
                "file://" + self.model_path + "/meshes/r_hand_roll.STL"
            )
        elif side == "Torso":
            marker.mesh_resource = (
                "file://" + self.model_path + "/meshes/base_link.STL"
            )

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.3
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = arm_pose_p[0]
        marker.pose.position.y = arm_pose_p[1]
        marker.pose.position.z = arm_pose_p[2]
        marker.pose.orientation = Quaternion(*arm_pose_q)

        return marker

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    visualizer = GraspPoseVisualizer()
    visualizer.run()
