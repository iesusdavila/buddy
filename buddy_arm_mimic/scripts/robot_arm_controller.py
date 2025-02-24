#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from buddy_interfaces.msg import HandPosition
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Almacenar la última posición de las articulaciones
        self.last_joint_positions = None
        
        # Suscriptor para la posición de la mano
        self.subscription = self.create_subscription(
            HandPosition,
            'right_arm_tracker',
            self.listener_callback,
            10)
        
        # Publicador para joint_states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Temporizador para publicar a 10 Hz (cada 0.1 segundos)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Límites de las articulaciones
        self.joint_limits = {
            'joint_5': (-0.7853, 1.5708),
            'joint_6': (0.0, 1.0472),
            'joint_7': (-0.7853, 0.7853),
            'joint_8': (0.1745, 1.5708)
        }
    
    def listener_callback(self, msg):
        # Procesar los datos recibidos y guardarlos
        joint_positions = {
            'joint_5': math.radians(msg.angle_shoulder_elbow_zy),
            'joint_6': math.radians(msg.angle_shoulder_elbow_yx),
            'joint_7': math.radians(msg.angle_elbow_wrist_yx),
            'joint_8': math.radians(msg.angle_elbow_wrist_zy)
        }
        
        # Aplicar límites a las articulaciones
        for joint, (lower, upper) in self.joint_limits.items():
            joint_positions[joint] = max(lower, min(upper, joint_positions[joint]))
        
        self.last_joint_positions = joint_positions
    
    def timer_callback(self):
        if self.last_joint_positions is not None:
            # Construir el mensaje de JointState con la última posición conocida
            joint_state = JointState()
            joint_state.name = [
                'joint_1', 'joint_2', 'joint_3', 'joint_4',
                'joint_5', 'joint_6', 'joint_7', 'joint_8',
                'joint_9', 'joint_10', 'joint_11', 'joint_12'
            ]
            
            joint_state.position = [
                0.0, 0.0, 0.0, 0.0,  # Articulaciones sin datos
                self.last_joint_positions['joint_5'],
                self.last_joint_positions['joint_6'],
                self.last_joint_positions['joint_7'],
                self.last_joint_positions['joint_8']*0 + 0.87265,
                0.0, 0.0, 0.0, 0.87265  # Articulación 12 fija
            ]
            
            joint_state.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(joint_state)
            self.get_logger().info(f'Publicado JointState: {joint_state.position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()