#!/usr/bin/env python3
"""
Script para verificar calibración y diagnóstico del robot en Gazebo
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import TransformStamped as TFStamped
import numpy as np
import time


class CalibrationCheck(Node):
    def __init__(self):
        super().__init__('calibration_check')
        
        # Subscriber para estado de articulaciones
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Variables de estado
        self.current_joint_positions = {}
        self.joint_limits = {
            'shoulder_pan_joint': {'min': -np.pi, 'max': np.pi},
            'shoulder_lift_joint': {'min': -np.pi, 'max': np.pi},
            'elbow_joint': {'min': -np.pi, 'max': np.pi},
            'wrist_1_joint': {'min': -np.pi, 'max': np.pi},
            'wrist_2_joint': {'min': -np.pi, 'max': np.pi},
            'wrist_3_joint': {'min': -np.pi, 'max': np.pi},
            'onrobot2fg7_finger1_joint': {'min': 0.0, 'max': 0.03},
            'onrobot2fg7_finger2_joint': {'min': 0.0, 'max': 0.03}
        }
        
        self.get_logger().info('Calibration Check inicializado')
        
        # Timer para diagnóstico periódico
        self.timer = self.create_timer(5.0, self.diagnostic_callback)

    def joint_state_callback(self, msg):
        """Callback para actualizar posiciones articulares"""
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

    def check_joint_limits(self):
        """Verificar límites articulares"""
        violations = []
        
        for joint_name, position in self.current_joint_positions.items():
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                if position < limits['min'] or position > limits['max']:
                    violations.append({
                        'joint': joint_name,
                        'position': position,
                        'min': limits['min'],
                        'max': limits['max']
                    })
        
        return violations

    def calculate_end_effector_pose(self):
        """Calcular pose del end-effector (simplificado)"""
        # Esto es una aproximación simplificada
        # En un sistema real usarías cinemática directa
        
        if not all(joint in self.current_joint_positions for joint in [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]):
            return None
        
        # Posiciones de ejemplo (deberías implementar cinemática directa real)
        # Esta es solo una aproximación para diagnóstico
        q1 = self.current_joint_positions['shoulder_pan_joint']
        q2 = self.current_joint_positions['shoulder_lift_joint']
        q3 = self.current_joint_positions['elbow_joint']
        q4 = self.current_joint_positions['wrist_1_joint']
        q5 = self.current_joint_positions['wrist_2_joint']
        q6 = self.current_joint_positions['wrist_3_joint']
        
        # Cinemática directa simplificada para UR5e
        # Longitudes aproximadas en metros
        L1 = 0.1625  # Base to shoulder
        L2 = 0.425   # Shoulder to elbow
        L3 = 0.392   # Elbow to wrist1
        L4 = 0.094   # Wrist1 to wrist2
        L5 = 0.094   # Wrist2 to wrist3
        L6 = 0.082   # Wrist3 to flange
        
        # Cálculos simplificados (esto necesita la implementación real de DH)
        x = (L2 * np.cos(q2) + L3 * np.cos(q2 + q3) + L4 * np.cos(q2 + q3 + q4)) * np.cos(q1)
        y = (L2 * np.cos(q2) + L3 * np.cos(q2 + q3) + L4 * np.cos(q2 + q3 + q4)) * np.sin(q1)
        z = L1 + L2 * np.sin(q2) + L3 * np.sin(q2 + q3) + L4 * np.sin(q2 + q3 + q4)
        
        # Orientación simplificada
        roll = q4 + q5 + q6
        pitch = q5
        yaw = q6
        
        return {
            'position': [x, y, z],
            'orientation': [roll, pitch, yaw]
        }

    def publish_end_effector_tf(self):
        """Publicar TF del end-effector"""
        pose = self.calculate_end_effector_pose()
        if pose:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'calculated_eef'
            
            t.transform.translation.x = pose['position'][0]
            t.transform.translation.y = pose['position'][1]
            t.transform.translation.z = pose['position'][2]
            
            # Convertir RPY a cuaternión (simplificado)
            # En implementación real usarías la matriz de rotación completa
            from tf_transformations import quaternion_from_euler
            q = quaternion_from_euler(pose['orientation'][0], 
                                     pose['orientation'][1], 
                                     pose['orientation'][2])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(t)

    def diagnostic_callback(self):
        """Callback periódico para diagnóstico"""
        print("\n" + "="*60)
        print("DIAGNÓSTICO DEL ROBOT".center(60))
        print("="*60)
        
        # Mostrar posiciones actuales
        print("\nPosiciones articulares actuales:")
        for joint_name in sorted(self.current_joint_positions.keys()):
            if joint_name.startswith(('shoulder', 'elbow', 'wrist', 'onrobot')):
                position = self.current_joint_positions[joint_name]
                print(f"  {joint_name:30}: {position:8.4f} rad")
        
        # Verificar límites
        violations = self.check_joint_limits()
        if violations:
            print("\n⚠️  VIOLACIONES DE LÍMITES:")
            for v in violations:
                print(f"  {v['joint']:30}: {v['position']:8.4f} "
                      f"(límites: [{v['min']:6.3f}, {v['max']:6.3f}])")
        else:
            print("\n✅ Todos los límites articulares respetados")
        
        # Calcular pose del end-effector
        pose = self.calculate_end_effector_pose()
        if pose:
            print("\nPose calculada del end-effector:")
            print(f"  Posición: [{pose['position'][0]:6.3f}, "
                  f"{pose['position'][1]:6.3f}, {pose['position'][2]:6.3f}] m")
            print(f"  Orientación RPY: [{pose['orientation'][0]:6.3f}, "
                  f"{pose['orientation'][1]:6.3f}, {pose['orientation'][2]:6.3f}] rad")
        
        print("="*60)

    def run_calibration_check(self):
        """Ejecutar chequeo completo de calibración"""
        self.get_logger().info("Iniciando chequeo de calibración...")
        
        # Esperar a recibir datos de articulaciones
        time.sleep(2)
        
        # Ejecutar diagnóstico
        self.diagnostic_callback()
        
        # Publicar TF
        self.publish_end_effector_tf()
        
        self.get_logger().info("Chequeo de calibración completado")


def main(args=None):
    rclpy.init(args=args)
    
    calibration = CalibrationCheck()
    
    print("Iniciando chequeo de calibración...")
    print("Esperando datos de articulaciones...")
    
    try:
        # Ejecutar chequeo inicial
        calibration.run_calibration_check()
        
        # Mantener el nodo vivo para diagnóstico periódico
        print("\nDiagnóstico ejecutándose cada 5 segundos.")
        print("Presiona Ctrl+C para salir.\n")
        
        rclpy.spin(calibration)
        
    except KeyboardInterrupt:
        print("\nChequeo de calibración interrumpido")
    finally:
        calibration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()