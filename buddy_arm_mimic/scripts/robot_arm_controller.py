#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from buddy_interfaces.msg import HandPosition
import math

class DualArmJointPublisher(Node):
    def __init__(self):
        super().__init__('dual_arm_joint_publisher')
        
        # Límites articulares para ambos brazos
        self.joint_limits = {
            # Brazo derecho
            'joint_5': (-0.7853, 1.5708),
            'joint_6': (0.0, 1.0472),
            'joint_7': (-0.7853, 0.7853),
            'joint_8': (0.1745, 1.5708),
            # Brazo izquierdo (mismos límites que derecho)
            'joint_9': (-0.7853, 1.5708),
            'joint_10': (0.0, 1.0472),
            'joint_11': (-0.7853, 0.7853),
            'joint_12': (0.1745, 1.5708)
        }

        # Posiciones iniciales (punto medio de los límites)
        self.last_right_pos = self.calculate_midpoints(['joint_5', 'joint_6', 'joint_7', 'joint_8'])
        self.last_left_pos = self.calculate_midpoints(['joint_9', 'joint_10', 'joint_11', 'joint_12'])

        # Suscriptores
        self.create_subscription(HandPosition, 'right_arm_tracker', self.right_arm_callback, 10)
        self.create_subscription(HandPosition, 'left_arm_tracker', self.left_arm_callback, 10)
        
        # Publicador y temporizador
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def calculate_midpoints(self, joints):
        return {j: (self.joint_limits[j][0] + self.joint_limits[j][1])/2 for j in joints}

    def process_arm_data(self, msg, arm_joints):
        positions = {
            arm_joints[0]: math.radians(msg.angle_shoulder_elbow_zy),
            arm_joints[1]: math.radians(msg.angle_shoulder_elbow_yx),
            arm_joints[2]: math.radians(msg.angle_elbow_wrist_yx),
            arm_joints[3]: math.radians(msg.angle_elbow_wrist_zy)
        }
        
        # Aplicar límites
        for joint in arm_joints:
            positions[joint] = max(
                self.joint_limits[joint][0],
                min(self.joint_limits[joint][1], positions[joint])
            )
        
        return positions

    def right_arm_callback(self, msg):
        self.last_right_pos = self.process_arm_data(msg, ['joint_5', 'joint_6', 'joint_7', 'joint_8'])
        self.get_logger().debug('Datos brazo derecho actualizados')

    def left_arm_callback(self, msg):
        self.last_left_pos = self.process_arm_data(msg, ['joint_9', 'joint_10', 'joint_11', 'joint_12'])
        self.get_logger().debug('Datos brazo izquierdo actualizados')

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f'joint_{i}' for i in range(1, 13)]
        
        # Construir arreglo de posiciones
        joint_state.position = [
            # Joints 1-4 (no controlados)
            0.0, 0.0, 0.0, 0.0,
            
            # Brazo derecho (joints 5-8)
            self.last_right_pos['joint_5'],
            self.last_right_pos['joint_6'],
            self.last_right_pos['joint_7'],
            self.last_right_pos['joint_8']*0 + 0.87265,
            
            # Brazo izquierdo (joints 9-12)
            self.last_left_pos['joint_9'],
            self.last_left_pos['joint_10'],
            self.last_left_pos['joint_11'],
            self.last_left_pos['joint_12']*0 + 0.87265
        ]
        
        self.publisher.publish(joint_state)
        self.get_logger().info('Posiciones publicadas para ambos brazos')
        self.get_logger().debug(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = DualArmJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()