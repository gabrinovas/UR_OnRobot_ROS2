#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joint_state_merger.py - Combina de forma reactiva y con cero latencia los estados articulares
del brazo UR (/joint_states) y del efector OnRobot (/onrobot/joint_states) en /merged_joint_states.
"""

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    UR_BASE_JOINTS = {
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    }

    def __init__(self):
        super().__init__('joint_state_merger')

        # Parámetros opcionales
        self.declare_parameter('tf_prefix', '')
        self.declare_parameter('sim_env', 'basic')
        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value

        # Subscriptores
        self.ur_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.ur_callback,
            10
        )

        self.gripper_sub = self.create_subscription(
            JointState,
            '/onrobot/joint_states',
            self.gripper_callback,
            10
        )

        # Publicador
        self.merged_pub = self.create_publisher(
            JointState,
            '/merged_joint_states',
            10
        )

        # Variables para almacenar estados con protección de hilos
        self.ur_joint_state = None
        self.gripper_joint_state = None
        self.lock = threading.Lock()

        self.get_logger().info(
            f"Joint State Merger reactivo iniciado (Zero-Latency) - tf_prefix='{self.tf_prefix}'"
        )

    def ur_callback(self, msg: JointState):
        with self.lock:
            # Filtrar: tomar solo joints del brazo UR
            filtered_msg = JointState()
            filtered_msg.header = msg.header

            for i, name in enumerate(msg.name):
                # Comprobar si corresponde a una articulación del UR
                is_ur_joint = any(name.endswith(base_j) for base_j in self.UR_BASE_JOINTS)
                if is_ur_joint:
                    filtered_msg.name.append(name)
                    filtered_msg.position.append(msg.position[i] if i < len(msg.position) else 0.0)
                    filtered_msg.velocity.append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
                    filtered_msg.effort.append(msg.effort[i] if i < len(msg.effort) else 0.0)

            self.ur_joint_state = filtered_msg
            self._publish_merged_locked(msg.header.stamp)

    def gripper_callback(self, msg: JointState):
        with self.lock:
            self.gripper_joint_state = msg
            self._publish_merged_locked(msg.header.stamp)

    def _publish_merged_locked(self, stamp):
        """Publica el mensaje combinado de forma inmediata. Debe llamarse con self.lock adquirido."""
        if self.ur_joint_state is None and self.gripper_joint_state is None:
            return

        merged_msg = JointState()
        merged_msg.header.stamp = stamp

        # 1. Articulaciones del brazo UR
        if self.ur_joint_state:
            merged_msg.name.extend(self.ur_joint_state.name)
            merged_msg.position.extend(self.ur_joint_state.position)
            merged_msg.velocity.extend(self.ur_joint_state.velocity)
            merged_msg.effort.extend(self.ur_joint_state.effort)

        # 2. Articulaciones del gripper OnRobot (2FG7, 3FG15, VGC10)
        if self.gripper_joint_state:
            merged_msg.name.extend(self.gripper_joint_state.name)
            merged_msg.position.extend(self.gripper_joint_state.position)
            merged_msg.velocity.extend(self.gripper_joint_state.velocity)
            merged_msg.effort.extend(self.gripper_joint_state.effort)

        self.merged_pub.publish(merged_msg)


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