#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from coco_interfaces.msg import BodyPoints

mp_pose = mp.solutions.pose

class BodyPointsDetectorNode(Node):
    def __init__(self):
        super().__init__('body_points_detector_node')
        self.bridge = CvBridge()
        self.pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(BodyPoints, 'body_points', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)

        points_msg = BodyPoints()
        points_msg.is_detected = False
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            points_msg.is_detected = True
            
            points_msg.right_shoulder_x = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x
            points_msg.right_shoulder_y = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y
            
            points_msg.right_elbow_x = landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x
            points_msg.right_elbow_y = landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y
            
            points_msg.right_wrist_x = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x
            points_msg.right_wrist_y = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y
            
            points_msg.left_shoulder_x = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x
            points_msg.left_shoulder_y = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y
            
            points_msg.left_elbow_x = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x
            points_msg.left_elbow_y = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y
            
            points_msg.left_wrist_x = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x
            points_msg.left_wrist_y = landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y
            
            self.publisher.publish(points_msg)        

def main(args=None):
    rclpy.init(args=args)
    node = BodyPointsDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()