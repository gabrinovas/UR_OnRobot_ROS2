#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')
        
        # Subscriptores
        self.ur_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.ur_callback,
            10)
        
        self.gripper_sub = self.create_subscription(
            JointState,
            '/onrobot/joint_states', 
            self.gripper_callback,
            10)
        
        # Publicador
        self.merged_pub = self.create_publisher(
            JointState,
            '/merged_joint_states',
            10)
        
        # Variables para almacenar estados
        self.ur_joint_state = None
        self.gripper_joint_state = None
        self.lock = threading.Lock()
        
        # Timer para publicar estados combinados
        self.timer = self.create_timer(0.1, self.publish_merged_states)
        
        self.get_logger().info("Joint State Merger iniciado - Combinando /joint_states y /onrobot/joint_states")
        
    def ur_callback(self, msg):
        with self.lock:
            # Filtrar: tomar solo joints del UR (excluir finger_width)
            filtered_msg = JointState()
            filtered_msg.header = msg.header
            
            ur_joints = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            
            for i, name in enumerate(msg.name):
                if name in ur_joints:  # Solo joints del brazo UR
                    filtered_msg.name.append(name)
                    filtered_msg.position.append(msg.position[i] if i < len(msg.position) else 0.0)
                    filtered_msg.velocity.append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
                    filtered_msg.effort.append(msg.effort[i] if i < len(msg.effort) else 0.0)
            
            self.ur_joint_state = filtered_msg
            
    def gripper_callback(self, msg):
        with self.lock:
            self.gripper_joint_state = msg
            
    def publish_merged_states(self):
        with self.lock:
            if self.ur_joint_state is None and self.gripper_joint_state is None:
                return
                
            merged_msg = JointState()
            merged_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 1. Añadir joints del UR
            if self.ur_joint_state:
                merged_msg.name.extend(self.ur_joint_state.name)
                merged_msg.position.extend(self.ur_joint_state.position)
                merged_msg.velocity.extend(self.ur_joint_state.velocity)
                merged_msg.effort.extend(self.ur_joint_state.effort)
            
            # 2. Añadir joints del OnRobot
            if self.gripper_joint_state:
                merged_msg.name.extend(self.gripper_joint_state.name)
                merged_msg.position.extend(self.gripper_joint_state.position)
                merged_msg.velocity.extend(self.gripper_joint_state.velocity)
                merged_msg.effort.extend(self.gripper_joint_state.effort)
                
            self.merged_pub.publish(merged_msg)
            # self.get_logger().info(f"Publicados {len(merged_msg.name)} joints combinados")

def main():
    rclpy.init()
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()