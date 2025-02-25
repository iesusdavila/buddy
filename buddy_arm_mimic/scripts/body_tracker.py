#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
from buddy_interfaces.msg import BodyPosition

mp_pose = mp.solutions.pose

class ArmTrackerNode(Node):
    def __init__(self):
        super().__init__('body_tracker_node')
        self.bridge = CvBridge()
        self.pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(BodyPosition, 'body_tracker', 10)

    def calculate_angle_with_vertical(self, shoulder, elbow):
        shoulder_x, shoulder_y = shoulder
        elbow_x, elbow_y = elbow

        v_x = elbow_x - shoulder_x  
        v_y = elbow_y - shoulder_y 

        return np.degrees(np.arctan2(-v_x, v_y))
    
    def calculate_shoulder_tilt(self, left_shoulder, right_shoulder):
        # Calcula el ángulo de inclinación entre hombros en el plano XY
        dx = right_shoulder[0] - left_shoulder[0]  # Diferencia en X
        dy = right_shoulder[1] - left_shoulder[1]  # Diferencia en Y
        
        # Ángulo entre la línea de hombros y la horizontal
        angle = np.degrees(np.arctan2(dy, dx))
        return angle


    def calculate_relative_angle(self, shoulder, elbow, wrist):
        # shoulder_right, elbow_right, wrist_right
        shoulder_x, shoulder_y = shoulder
        elbow_x, elbow_y = elbow
        wrist_x, wrist_y = wrist

        # Precalcular diferencias
        v_x, v_y = wrist_x - elbow_x, wrist_y - elbow_x  # Codo → Muñeca
        u_x, u_y = elbow_x - shoulder_x, elbow_y - shoulder_y  # Hombro → Codo

        # Calcular producto escalar y determinante
        det_v_u = u_x * v_y - u_y * v_x
        dot_v_u = u_x * v_x + u_y * v_y

        # Calcular ángulo con optimización
        return np.degrees(np.arctan2(det_v_u, dot_v_u))


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
            
            # Calcular inclinación de hombros
            shoulder_tilt = -self.calculate_shoulder_tilt(shoulder_left, shoulder_right)

            angle_shoulder_right_elbow_YX = self.calculate_angle_with_vertical(shoulder_right, elbow_right)
            angle_shoulder_left_elbow_YX = -self.calculate_angle_with_vertical(shoulder_left, elbow_left)

            angle_elbow_right_wrist_YX = -self.calculate_relative_angle(shoulder_right, elbow_right, wrist_right)
            angle_elbow_left_wrist_YX = -self.calculate_relative_angle(shoulder_left, elbow_left, wrist_left)
            
            mp.solutions.drawing_utils.draw_landmarks(
                frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            cv2.imshow('MediaPipe Pose', frame)

            arm_msg = BodyPosition()
            arm_msg.shoulder_tilt_angle = float(shoulder_tilt)

            arm_msg.angle_shoulder_right_elbow_zy = 0.0 #float(angle_shoulder_elbow_ZY)
            arm_msg.angle_elbow_right_wrist_zy = 0.0 #float(angle_elbow_wrist_ZY)
            arm_msg.angle_shoulder_right_elbow_yx = float(angle_shoulder_right_elbow_YX)
            arm_msg.angle_elbow_right_wrist_yx = float(angle_elbow_right_wrist_YX)

            arm_msg.angle_shoulder_left_elbow_zy = 0.0
            arm_msg.angle_elbow_left_wrist_zy = 0.0
            arm_msg.angle_shoulder_left_elbow_yx = float(angle_shoulder_left_elbow_YX)
            arm_msg.angle_elbow_left_wrist_yx = float(angle_elbow_left_wrist_YX)
            
            self.publisher.publish(arm_msg)
            
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
