#!/usr/bin/env python3
"""
Script para mover el robot a posiciones específicas
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import time


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        
        # Publisher para control de trayectorias
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Publisher para gripper
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/onrobot_gripper_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Robot Mover inicializado')

    def move_to_joint_positions(self, positions, duration=5.0):
        """Mover a posiciones articulares específicas"""
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
        
        self.joint_trajectory_pub.publish(msg)
        self.get_logger().info(f'Moviendo a posiciones: {positions}')
        
        # Esperar un momento
        time.sleep(0.1)

    def move_to_home(self):
        """Mover a posición home"""
        home_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.move_to_joint_positions(home_positions, 5.0)

    def move_to_ready(self):
        """Mover a posición ready para pick/place"""
        ready_positions = [0.0, -1.0, 1.0, -1.57, -1.57, 0.0]
        self.move_to_joint_positions(ready_positions, 5.0)

    def control_gripper(self, position, duration=1.0):
        """Controlar el gripper (0=cerrado, 1=abierto)"""
        # Para gripper simétrico, ambas dedos se mueven igual
        msg = JointTrajectory()
        msg.joint_names = [
            'onrobot2fg7_finger1_joint',
            'onrobot2fg7_finger2_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]  # Ambas dedos igual
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        msg.points.append(point)
        
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper a posición: {position}')
        
        # Esperar un momento
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    
    mover = RobotMover()
    
    if len(sys.argv) < 2:
        print("Uso: ros2 run ur_onrobot_gazebo move_to_position.py [comando]")
        print("Comandos:")
        print("  home                - Mover a posición home")
        print("  ready               - Mover a posición ready")
        print("  joints j1 j2 j3 j4 j5 j6 - Mover a posiciones específicas")
        print("  gripper open|close|0.5  - Controlar gripper")
        print("")
        print("Ejemplos:")
        print("  ros2 run ur_onrobot_gazebo move_to_position.py home")
        print("  ros2 run ur_onrobot_gazebo move_to_position.py joints 0.0 -1.57 1.57 -1.57 -1.57 0.0")
        print("  ros2 run ur_onrobot_gazebo move_to_position.py gripper open")
        return
    
    command = sys.argv[1]
    
    if command == 'home':
        mover.move_to_home()
        print("Robot moviéndose a HOME")
        
    elif command == 'ready':
        mover.move_to_ready()
        print("Robot moviéndose a READY")
        
    elif command == 'joints':
        if len(sys.argv) >= 8:
            positions = [float(x) for x in sys.argv[2:8]]
            mover.move_to_joint_positions(positions, 5.0)
            print(f"Robot moviéndose a: {positions}")
        else:
            print("Error: Se necesitan 6 valores de posición")
            
    elif command == 'gripper':
        if len(sys.argv) >= 3:
            action = sys.argv[2]
            if action == 'open':
                mover.control_gripper(0.0)  # Abierto
                print("Gripper abriendo")
            elif action == 'close':
                mover.control_gripper(0.03)  # Cerrado (ajustar según necesidad)
                print("Gripper cerrando")
            elif action.replace('.', '', 1).isdigit():
                position = float(action)
                mover.control_gripper(position)
                print(f"Gripper a posición: {position}")
            else:
                print(f"Acción de gripper desconocida: {action}")
        else:
            print("Error: Se necesita especificar acción para gripper")
            
    else:
        print(f"Comando desconocido: {command}")
    
    # Mantener el nodo vivo brevemente para enviar mensajes
    time.sleep(2)
    
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()