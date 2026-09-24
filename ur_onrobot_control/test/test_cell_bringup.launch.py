import os
import time
import unittest
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch_testing
import launch_testing.actions
import launch_testing.markers


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_control'),
                'launch',
                'start_simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'onrobot_type': '2fg7',
            'sim_env': 'basic',
            'launch_rviz': 'false',
            'launch_watchdog': 'true',
        }.items()
    )

    return LaunchDescription([
        sim_launch,
        launch_testing.actions.ReadyToTest()
    ])


class TestCellBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_cell_bringup_checker')

    def tearDown(self):
        self.node.destroy_node()

    def test_merged_joint_states_published(self):
        """Verifica que el tópico unificado de articulaciones publica estados de robot y efector."""
        received_msgs = []

        sub = self.node.create_subscription(
            JointState,
            '/merged_joint_states',
            lambda msg: received_msgs.append(msg),
            10
        )

        start_time = time.time()
        timeout = 30.0
        found_ur = False
        found_gripper = False
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if received_msgs:
                latest = received_msgs[-1]
                if any('shoulder_pan_joint' in name for name in latest.name):
                    found_ur = True
                if any('finger_width' in name for name in latest.name):
                    found_gripper = True
                if found_ur and found_gripper:
                    break

        self.node.destroy_subscription(sub)

        self.assertGreaterEqual(len(received_msgs), 1,
                                "No se recibieron mensajes en /merged_joint_states")
        self.assertTrue(found_ur,
                        "Falta shoulder_pan_joint en /merged_joint_states")
        self.assertTrue(found_gripper,
                        "Falta finger_width en /merged_joint_states")

    def test_watchdog_diagnostics_published(self):
        """Verifica que el supervisor de seguridad publica diagnósticos nominales."""
        diag_msgs = []

        sub = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            lambda msg: diag_msgs.append(msg),
            10
        )

        start_time = time.time()
        timeout = 25.0
        watchdog_found = False
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            for arr in diag_msgs:
                for status in arr.status:
                    if 'Safety & Interlock Monitor' in status.name or 'UR Safety Watchdog' in status.hardware_id:
                        watchdog_found = True
                        self.assertEqual(ord(status.level), 0,
                                         f"Nivel esperado OK(0), recibido {status.level}")
                        break
                if watchdog_found:
                    break
            if watchdog_found:
                break

        self.node.destroy_subscription(sub)

        self.assertGreaterEqual(len(diag_msgs), 1, "No se recibieron mensajes en /diagnostics")
        self.assertTrue(watchdog_found,
                        "No se encontró el estado de UR Safety Watchdog en /diagnostics")
