#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
from buddy_interfaces.msg import HandPosition

mp_pose = mp.solutions.pose

class ArmTrackerNode(Node):
    def __init__(self):
        super().__init__('arm_tracker_node')
        self.bridge = CvBridge()
        self.pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher_right = self.create_publisher(HandPosition, 'right_arm_tracker', 10)
        self.publisher_left = self.create_publisher(HandPosition, 'left_arm_tracker', 10)

    def calculate_angle_with_vertical(self, p1, p2, plane="ZY"):
        if plane == "ZY":
            v = np.array([p2[1] - p1[1], p2[2] - p1[2]])  # (y, z)
            u = np.array([1, 0])  # Vector vertical en Y
        elif plane == "YX":
            v = np.array([p2[0] - p1[0], p2[1] - p1[1]])  # (x, y)
            u = np.array([0, 1])  # Vector vertical en Y
        else:
            raise ValueError("El plano debe ser 'ZY' o 'YX'.")
        
        angle = np.arctan2(np.linalg.det([u, v]), np.dot(u, v))  # Mantiene signo
        return np.degrees(angle)

    def calculate_relative_angle(self, p1, p2, p3):
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])  # Vector hombro-codo
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])  # Vector codo-mano
        
        angle = np.arctan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
        return np.degrees(angle)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image)
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            shoulder_right = (landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                        landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y,
                        landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z)
            shoulder_left = (landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y,
                        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z)

            elbow_right = (landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x,
                     landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y,
                     landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z)
            elbow_left = (landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x,
                     landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y,
                     landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z)

            wrist_right = (landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x,
                     landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y,
                     landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z)
            wrist_left = (landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x,
                        landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y,
                        landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z)

            angle_shoulder_right_elbow_ZY = self.calculate_angle_with_vertical(shoulder_right, elbow_right, "ZY")
            angle_shoulder_left_elbow_ZY = self.calculate_angle_with_vertical(shoulder_left, elbow_left, "ZY")

            angle_elbow_right_wrist_ZY = self.calculate_angle_with_vertical(elbow_right, wrist_right, "ZY")
            angle_elbow_left_wrist_ZY = self.calculate_angle_with_vertical(elbow_left, wrist_left, "ZY")

            angle_shoulder_right_elbow_YX = self.calculate_angle_with_vertical(shoulder_right, elbow_right, "YX")
            angle_shoulder_left_elbow_YX = -self.calculate_angle_with_vertical(shoulder_left, elbow_left, "YX")

            angle_elbow_right_wrist_YX = -self.calculate_relative_angle(shoulder_right, elbow_right, wrist_right)
            angle_elbow_left_wrist_YX = -self.calculate_relative_angle(shoulder_left, elbow_left, wrist_left)

            cv2.putText(frame, f"Ángulo ZY Hombro-Codo: {angle_shoulder_left_elbow_ZY:.2f}°",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Ángulo ZY Codo-Mano: {angle_elbow_left_wrist_ZY:.2f}°",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Ángulo YX Hombro-Codo: {angle_shoulder_left_elbow_YX:.2f}°",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Ángulo YX Codo-Mano: {angle_elbow_left_wrist_YX:.2f}°",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
            mp.solutions.drawing_utils.draw_landmarks(
                frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            cv2.imshow('MediaPipe Pose', frame)

            hand_msg_right = HandPosition()
            hand_msg_right.angle_shoulder_elbow_zy = 0.0 #float(angle_shoulder_elbow_ZY)
            hand_msg_right.angle_elbow_wrist_zy = 0.0 #float(angle_elbow_wrist_ZY)
            hand_msg_right.angle_shoulder_elbow_yx = float(angle_shoulder_right_elbow_YX)
            hand_msg_right.angle_elbow_wrist_yx = float(angle_elbow_right_wrist_YX)

            hand_msg_left = HandPosition()
            hand_msg_left.angle_shoulder_elbow_zy = 0.0
            hand_msg_left.angle_elbow_wrist_zy = 0.0
            hand_msg_left.angle_shoulder_elbow_yx = float(angle_shoulder_left_elbow_YX)
            hand_msg_left.angle_elbow_wrist_yx = float(angle_elbow_left_wrist_YX)
            
            self.publisher_right.publish(hand_msg_right)
            self.publisher_left.publish(hand_msg_left)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArmTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
