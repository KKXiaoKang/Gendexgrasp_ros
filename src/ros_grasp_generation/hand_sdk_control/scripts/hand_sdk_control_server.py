#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from dynamic_biped.msg import robotHandPosition
from hand_sdk_control.srv import handPosService, handPosServiceResponse
import time

class HandSDKControlServer:
    def __init__(self):
        rospy.init_node('hand_sdk_control_server')
        
        self.control_robot_hand_position_pub = rospy.Publisher(
            "/control_robot_hand_position", robotHandPosition, queue_size=10
        )
        
        self.service = rospy.Service(
            "/hand_sdk_control_service", handPosService, self.handle_hand_position_request
        )
        rospy.loginfo("Hand SDK Control Service Ready.")
    
    def handle_hand_position_request(self, req):
        # 将uint8[]转换为list，以避免它们被误解为bytes
        left_hand_position = list(req.left_hand_position)
        right_hand_position = list(req.right_hand_position)

        # 打印接收到的请求
        rospy.loginfo("Received Hand Position Request: Left Hand Position: {}, Right Hand Position: {}".format(
            left_hand_position, right_hand_position
        ))

        # 等待3s
        time.sleep(3)

        # 发布到 /control_robot_hand_position 话题
        hand_position_msg = robotHandPosition()
        hand_position_msg.header = req.header
        hand_position_msg.left_hand_position = req.left_hand_position
        hand_position_msg.right_hand_position = req.right_hand_position
        
        self.control_robot_hand_position_pub.publish(hand_position_msg)
        
        return handPosServiceResponse(result=True)
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    server = HandSDKControlServer()
    server.run()
