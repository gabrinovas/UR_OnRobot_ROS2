#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spawn_and_pick.py - Utilidad CLI para el Pipeline Autónomo de Pick & Place MTC
Permite posicionar piezas de trabajo y disparar la secuencia de manipulación.
"""

import argparse
import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class SpawnAndPickClient(Node):
    def __init__(self):
        super().__init__('spawn_and_pick_client')
        self.trigger_client = self.create_client(Trigger, '/ur_onrobot_mtc_node/execute_task')

    def trigger_task(self, timeout_sec=5.0):
        self.get_logger().info('Esperando al servicio /ur_onrobot_mtc_node/execute_task...')
        if not self.trigger_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                'El servicio /ur_onrobot_mtc_node/execute_task no está disponible. '
                '¿Está lanzado ur_onrobot_mtc.launch.py?'
            )
            return False

        req = Trigger.Request()
        future = self.trigger_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            res = future.result()
            if res.success:
                self.get_logger().info(f'ÉXITO: {res.message}')
            else:
                self.get_logger().warn(f'FALLO: {res.message}')
            return res.success
        else:
            self.get_logger().error('Error en la llamada al servicio MTC.')
            return False


def main():
    parser = argparse.ArgumentParser(description='Utilidad de disparo para Pick & Place MTC (UR5e + OnRobot)')
    parser.add_argument('--trigger-only', action='store_true', help='Llamar al servicio de ejecución MTC sin modificar parámetros')
    args, _ = parser.parse_known_args()

    rclpy.init()
    client = SpawnAndPickClient()

    print("\n╔═══════════════════════════════════════════════════════════════╗")
    print("║     ORQUESTADOR PICK & PLACE AUTÓNOMO (MOVEIT MTC)            ║")
    print("╚═══════════════════════════════════════════════════════════════╝")
    print("  Disparando pipeline de 11 etapas sobre el robot...")

    success = client.trigger_task(timeout_sec=8.0)
    client.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
