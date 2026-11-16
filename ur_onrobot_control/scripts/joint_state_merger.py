#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Merge the joint states of the arm and of the gripper into one topic.

The gripper runs in a controller manager of its own, so its joints arrive on another topic. Every
message of the arm is republished at once with the last state of the gripper added, so the merged
topic has the rate and the time stamps of the arm.
"""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ur_onrobot_control.node_parameters import INTEGER, required, STRING, STRING_ARRAY


class JointStateMerger(Node):

    def __init__(self):
        super().__init__('joint_state_merger')
        prefix = required(self, 'tf_prefix', STRING)
        self.arm_joints = {prefix + joint for joint in required(self, 'arm_joints', STRING_ARRAY)}
        self.minimum_step_ns = required(self, 'minimum_stamp_step_ns', INTEGER)
        depth = required(self, 'queue_depth', INTEGER)

        self.lock = threading.Lock()
        self.arm_state = None
        self.gripper_state = None
        self.last_stamp_ns = 0
        self.publisher = self.create_publisher(
            JointState, required(self, 'merged_topic', STRING), depth)
        self.create_subscription(
            JointState, required(self, 'arm_topic', STRING), self.on_arm_state, depth)
        self.create_subscription(
            JointState, required(self, 'gripper_topic', STRING), self.on_gripper_state, depth)
        self.get_logger().info(f"Joint state merger started (tf_prefix='{prefix}')")

    def on_arm_state(self, msg):
        """Keep the joints of the arm and publish the merged state."""
        with self.lock:
            arm = JointState()
            arm.header = msg.header
            for index, name in enumerate(msg.name):
                if name not in self.arm_joints:
                    continue
                arm.name.append(name)
                arm.position.append(msg.position[index] if index < len(msg.position) else 0.0)
                arm.velocity.append(msg.velocity[index] if index < len(msg.velocity) else 0.0)
                arm.effort.append(msg.effort[index] if index < len(msg.effort) else 0.0)
            self.arm_state = arm
            self.publish_merged(msg.header.stamp)

    def on_gripper_state(self, msg):
        """Keep the state of the gripper; publish it alone while the arm has not reported yet."""
        with self.lock:
            self.gripper_state = msg
            if self.arm_state is None:
                self.publish_merged(msg.header.stamp)

    def publish_merged(self, stamp):
        """Publish arm and gripper together. Called with the lock held."""
        merged = JointState()
        # Stamps never go back or repeat: TF would drop the message otherwise.
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + self.minimum_step_ns
        self.last_stamp_ns = stamp_ns
        merged.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        merged.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        for state in (self.arm_state, self.gripper_state):
            if state is not None:
                merged.name.extend(state.name)
                merged.position.extend(state.position)
                merged.velocity.extend(state.velocity)
                merged.effort.extend(state.effort)
        self.publisher.publish(merged)


def main():
    rclpy.init()
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # a stop signal raises either
        pass
    except RuntimeError:
        # rclpy can fail to take a last message while the stop signal shuts it down
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():  # a SIGINT already shut rclpy down: shutting down again is an error
            rclpy.shutdown()


if __name__ == '__main__':
    main()
