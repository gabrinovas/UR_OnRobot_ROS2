#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
servo_keyboard_teleop.py - Teleoperación interactiva por teclado para MoveIt 2 Servo
Permite comandar el TCP del UR5e y accionar las pinzas OnRobot desde el terminal.
"""

import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


INSTRUCTIONS = """
╔═══════════════════════════════════════════════════════════════╗
║         TELEOPERACIÓN CARTESIANA MOVEIT SERVO (UR5e)          ║
╚═══════════════════════════════════════════════════════════════╝

  Movimiento Lineal:             Rotación Angular:
      [W] : +X                       [U] : +Roll (X)
  [A] [S] [D] : -Y / -X / +Y     [J] [K] [L] : -Yaw / -Pitch / +Yaw
      [R] : +Z                       [I] : +Pitch (Y)
      [F] : -Z                       [O] : -Roll (X)

  Control de Gripper / Vacío:
      [ESPACIO] : Cerrar pinza / Activar vacío
      [B]       : Abrir pinza / Desactivar vacío

  Ajustes y Parámetros:
      [1] : Frame de referencia BASE_LINK
      [2] : Frame de referencia GRIPPER_TCP
      [+] : Incrementar velocidad (+0.05 m/s)
      [-] : Reducir velocidad (-0.05 m/s)

      [Q] / [Ctrl+C] : Salir
─────────────────────────────────────────────────────────────────
"""


class ServoKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('servo_keyboard_teleop')

        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/onrobot/gripper_action_controller/gripper_cmd')
        self.start_servo_client = self.create_client(Trigger, '/servo_node/start_servo')

        self.linear_speed = 0.15   # m/s
        self.angular_speed = 0.40  # rad/s
        self.frame_id = 'base_link'
        self.gripper_closed = False

        self.enable_servo()
        self.get_logger().info('Nodo de teleoperación por teclado iniciado')

    def enable_servo(self):
        if self.start_servo_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.start_servo_client.call_async(req)
            self.get_logger().info('Servicio /servo_node/start_servo activado correctamente')
        else:
            self.get_logger().warn('Servicio /servo_node/start_servo no detectado aún')

    def publish_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.twist.linear.x = float(lx)
        msg.twist.linear.y = float(ly)
        msg.twist.linear.z = float(lz)
        msg.twist.angular.x = float(ax)
        msg.twist.angular.y = float(ay)
        msg.twist.angular.z = float(az)

        self.twist_pub.publish(msg)

    def trigger_gripper(self, close_gripper=True):
        if not self.gripper_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Servidor de acción del gripper no disponible')
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.0 if close_gripper else 0.07  # 0 m cerrado, 70 mm abierto
        goal.command.max_effort = 40.0
        self.gripper_client.send_goal_async(goal)
        self.gripper_closed = close_gripper
        state_str = "CERRADO / VACÍO ON" if close_gripper else "ABIERTO / VACÍO OFF"
        print(f"\r  >>> Gripper: {state_str}                          ", end='')


def get_key(settings, timeout=0.08):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    if not sys.stdin.isatty():
        print(INSTRUCTIONS)
        print("AVISO: servo_keyboard_teleop requiere una terminal TTY interactiva (ej. docker exec -it ... o terminal local).")
        return

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ServoKeyboardTeleop()

    print(INSTRUCTIONS)
    print(f"Estado inicial: Frame={node.frame_id} | Vel={node.linear_speed:.2f} m/s\n")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.05)
            lx = ly = lz = 0.0
            ax = ay = az = 0.0

            if key in ('w', 'W'):
                lx = node.linear_speed
            elif key in ('s', 'S'):
                lx = -node.linear_speed
            elif key in ('a', 'A'):
                ly = node.linear_speed
            elif key in ('d', 'D'):
                ly = -node.linear_speed
            elif key in ('r', 'R'):
                lz = node.linear_speed
            elif key in ('f', 'F'):
                lz = -node.linear_speed
            elif key in ('u', 'U'):
                ax = node.angular_speed
            elif key in ('o', 'O'):
                ax = -node.angular_speed
            elif key in ('i', 'I'):
                ay = node.angular_speed
            elif key in ('k', 'K'):
                ay = -node.angular_speed
            elif key in ('j', 'J'):
                az = node.angular_speed
            elif key in ('l', 'L'):
                az = -node.angular_speed
            elif key == '1':
                node.frame_id = 'base_link'
                print(f"\r  [Frame cambiado a: {node.frame_id}]                 ", end='')
            elif key == '2':
                node.frame_id = 'gripper_tcp'
                print(f"\r  [Frame cambiado a: {node.frame_id}]                 ", end='')
            elif key in ('+', '='):
                node.linear_speed = min(0.6, node.linear_speed + 0.05)
                print(f"\r  [Velocidad lineal: {node.linear_speed:.2f} m/s]         ", end='')
            elif key in ('-', '_'):
                node.linear_speed = max(0.02, node.linear_speed - 0.05)
                print(f"\r  [Velocidad lineal: {node.linear_speed:.2f} m/s]         ", end='')
            elif key == ' ':
                node.trigger_gripper(True)
            elif key in ('b', 'B'):
                node.trigger_gripper(False)
            elif key in ('q', 'Q', '\x03'):  # 'q' o Ctrl-C
                print("\nSaliendo de la teleoperación...")
                break

            # Publicar velocidad activa o cero
            node.publish_twist(lx, ly, lz, ax, ay, az)
            rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as e:
        print(f"\nError en teleoperación: {e}")
    finally:
        # Detener movimiento
        node.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
