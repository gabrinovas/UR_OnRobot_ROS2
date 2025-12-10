#!/usr/bin/env python3
"""
Demo de pick and place para UR robot con gripper OnRobot
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import math


class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/onrobot_gripper_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Pick & Place Demo inicializado')

    def move_arm(self, positions, duration=3.0):
        """Mover brazo a posiciones específicas"""
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        msg.points.append(point)
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Moviendo brazo: {positions}')
        time.sleep(duration + 0.5)

    def control_gripper(self, position, duration=1.0):
        """Controlar gripper"""
        msg = JointTrajectory()
        msg.joint_names = [
            'onrobot2fg7_finger1_joint',
            'onrobot2fg7_finger2_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        msg.points.append(point)
        self.gripper_pub.publish(msg)
        
        action = "abriendo" if position < 0.01 else "cerrando"
        self.get_logger().info(f'Gripper {action}')
        time.sleep(duration + 0.5)

    def run_demo(self):
        """Ejecutar demo completa de pick and place"""
        self.get_logger().info("=== INICIANDO DEMO PICK & PLACE ===")
        
        # 1. Home position
        self.get_logger().info("1. Yendo a HOME")
        self.move_arm([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], 5.0)
        
        # 2. Approach pick position
        self.get_logger().info("2. Acercándose a posición de pick")
        self.move_arm([0.0, -0.8, 0.8, -1.57, -1.57, 0.0], 4.0)
        
        # 3. Open gripper
        self.get_logger().info("3. Abriendo gripper")
        self.control_gripper(0.0)  # Abierto
        
        # 4. Move to pick position
        self.get_logger().info("4. Moviendo a posición de pick")
        self.move_arm([0.0, -0.9, 0.9, -1.57, -1.57, 0.0], 2.0)
        
        # 5. Close gripper (pick)
        self.get_logger().info("5. Cerrando gripper (PICK)")
        self.control_gripper(0.02)  # Cerrado para agarrar
        
        # 6. Lift object
        self.get_logger().info("6. Levantando objeto")
        self.move_arm([0.0, -0.7, 0.7, -1.57, -1.57, 0.0], 3.0)
        
        # 7. Move to place position
        self.get_logger().info("7. Moviendo a posición de place")
        self.move_arm([1.57, -0.8, 0.8, -1.57, -1.57, 1.57], 4.0)
        
        # 8. Lower to place
        self.get_logger().info("8. Bajando para place")
        self.move_arm([1.57, -0.9, 0.9, -1.57, -1.57, 1.57], 2.0)
        
        # 9. Open gripper (place)
        self.get_logger().info("9. Abriendo gripper (PLACE)")
        self.control_gripper(0.0)
        
        # 10. Retract from place
        self.get_logger().info("10. Retirándose de posición de place")
        self.move_arm([1.57, -0.7, 0.7, -1.57, -1.57, 1.57], 2.0)
        
        # 11. Return to home
        self.get_logger().info("11. Regresando a HOME")
        self.move_arm([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], 5.0)
        
        self.get_logger().info("=== DEMO COMPLETADA ===")


def main(args=None):
    rclpy.init(args=args)
    
    demo = PickPlaceDemo()
    
    print("Iniciando demo de Pick & Place en 3 segundos...")
    time.sleep(3)
    
    try:
        demo.run_demo()
    except KeyboardInterrupt:
        print("\nDemo interrumpida por el usuario")
    except Exception as e:
        print(f"Error en demo: {e}")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()