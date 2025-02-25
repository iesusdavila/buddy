#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from buddy_interfaces.msg import BodyPosition
import math

class DualArmJointPublisher(Node):
    def __init__(self):
        super().__init__('dual_arm_joint_publisher')
        
        # Límites articulares para ambos brazos y torso
        self.joint_limits = {
            # Torso
            'joint_2': (-0.5235, 0.5235),  # Inclinación del torso
            
            # Brazo derecho
            'joint_5': (-0.7853, 1.5708),
            'joint_6': (0.0, 1.0472),
            'joint_7': (-0.7853, 0.7853),
            'joint_8': (0.1745, 1.5708),
            
            # Brazo izquierdo
            'joint_9': (-0.7853, 1.5708),
            'joint_10': (0.0, 1.0472),
            'joint_11': (-0.7853, 0.7853),
            'joint_12': (0.1745, 1.5708)
        }
        
        # Posiciones iniciales (punto medio de los límites)
        self.last_right_pos = self.calculate_midpoints(['joint_5', 'joint_6', 'joint_7', 'joint_8'])
        self.last_left_pos = self.calculate_midpoints(['joint_9', 'joint_10', 'joint_11', 'joint_12'])
        
        # Inicializar posición del torso en posición neutral (0)
        self.torso_tilt = 0.0
        
        # Suscriptor único para ambos brazos
        self.subscription = self.create_subscription(
            HandPosition,
            'arm_tracker',
            self.arm_tracker_callback,
            10
        )
        
        # Publicador y temporizador
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def calculate_midpoints(self, joints):
        return {j: (self.joint_limits[j][0] + self.joint_limits[j][1])/2 for j in joints}
    
    def process_arm_data(self, angles, arm_joints):
        positions = {
            arm_joints[0]: math.radians(angles[0]),  # shoulder_elbow_zy
            arm_joints[1]: math.radians(angles[1]),  # shoulder_elbow_yx
            arm_joints[2]: math.radians(angles[2]),  # elbow_wrist_yx
            arm_joints[3]: math.radians(angles[3])   # elbow_wrist_zy
        }
        
        # Aplicar límites
        for joint in arm_joints:
            lower, upper = self.joint_limits[joint]
            positions[joint] = max(lower, min(upper, positions[joint]))
            
        return positions
        
    def arm_tracker_callback(self, msg):
        # Procesar inclinación del torso (joint_2)
        if msg.shoulder_tilt_angle > 0:
            tilt_angle = msg.shoulder_tilt_angle - 180
        else:
            tilt_angle = msg.shoulder_tilt_angle + 180
        lower, upper = self.joint_limits['joint_2']
        self.torso_tilt = max(lower, min(upper, math.radians(tilt_angle)))
        self.get_logger().info('Angulo tilt del torso actualizado y su valor es: ' + str(self.torso_tilt))
        
        # Procesar brazo derecho
        right_angles = (
            msg.angle_shoulder_right_elbow_zy,
            msg.angle_shoulder_right_elbow_yx,
            msg.angle_elbow_right_wrist_yx,
            msg.angle_elbow_right_wrist_zy
        )
        self.last_right_pos = self.process_arm_data(right_angles, ['joint_5', 'joint_6', 'joint_7', 'joint_8'])
        
        # Procesar brazo izquierdo
        left_angles = (
            msg.angle_shoulder_left_elbow_zy,
            msg.angle_shoulder_left_elbow_yx,
            msg.angle_elbow_left_wrist_yx,
            msg.angle_elbow_left_wrist_zy
        )
        self.last_left_pos = self.process_arm_data(left_angles, ['joint_9', 'joint_10', 'joint_11', 'joint_12'])
        
        self.get_logger().debug('Datos actualizados para torso y ambos brazos')
        
    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f'joint_{i}' for i in range(1, 13)]
        
        # Construir arreglo de posiciones
        joint_state.position = [
            # Joint 1 (no controlado)
            0.0,
            # Joint 2 (torso tilt)
            self.torso_tilt,
            # Joints 3-4 (no controlados)
            0.0, 0.0,
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
        self.get_logger().info('Posiciones publicadas para torso y brazos', throttle_duration_sec=1)

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