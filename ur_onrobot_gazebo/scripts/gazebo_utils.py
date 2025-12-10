#!/usr/bin/env python3
"""
Utilidades para interacción con Gazebo
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetModelState, SetModelState, DeleteModel, GetWorldProperties
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
import numpy as np


class GazeboUtils(Node):
    def __init__(self):
        super().__init__('gazebo_utils')
        
        # Clientes para servicios de Gazebo
        self.get_state_client = self.create_client(GetModelState, '/gazebo/get_model_state')
        self.set_state_client = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.delete_model_client = self.create_client(DeleteModel, '/gazebo/delete_model')
        self.get_world_props_client = self.create_client(GetWorldProperties, '/gazebo/get_world_properties')
        
        # Esperar servicios
        self.wait_for_services()
        
        self.get_logger().info('Gazebo Utils inicializado')

    def wait_for_services(self):
        """Esperar a que los servicios estén disponibles"""
        services = [
            (self.get_state_client, '/gazebo/get_model_state'),
            (self.set_state_client, '/gazebo/set_model_state'),
            (self.delete_model_client, '/gazebo/delete_model'),
            (self.get_world_props_client, '/gazebo/get_world_properties')
        ]
        
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Esperando servicio {name}...')

    def get_model_state(self, model_name, relative_entity='world'):
        """Obtener estado de un modelo"""
        request = GetModelState.Request()
        request.model_name = model_name
        request.relative_entity_name = relative_entity
        
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result()
        return None

    def set_model_pose(self, model_name, position, orientation, relative_entity='world'):
        """Establecer pose de un modelo"""
        request = SetModelState.Request()
        request.model_state.model_name = model_name
        request.model_state.pose.position = Point(x=position[0], y=position[1], z=position[2])
        request.model_state.pose.orientation = Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
        )
        request.model_state.reference_frame = relative_entity
        
        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().success
        return False

    def delete_model(self, model_name):
        """Eliminar un modelo del mundo"""
        request = DeleteModel.Request()
        request.model_name = model_name
        
        future = self.delete_model_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().success
        return False

    def get_world_models(self):
        """Obtener lista de modelos en el mundo"""
        request = GetWorldProperties.Request()
        
        future = self.get_world_props_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().model_names
        return []

    def clear_test_objects(self):
        """Eliminar todos los objetos de prueba"""
        models = self.get_world_models()
        deleted = 0
        
        for model in models:
            if model.startswith('test_'):
                if self.delete_model(model):
                    self.get_logger().info(f'Eliminado: {model}')
                    deleted += 1
                else:
                    self.get_logger().warn(f'No se pudo eliminar: {model}')
        
        self.get_logger().info(f'Total eliminados: {deleted}')
        return deleted

    def list_models(self):
        """Listar todos los modelos en el mundo"""
        models = self.get_world_models()
        print(f"\n=== MODELOS EN EL MUNDO ({len(models)}) ===")
        for i, model in enumerate(models, 1):
            state = self.get_model_state(model)
            if state and state.success:
                pos = state.pose.position
                print(f"{i:2}. {model:30} Pos: [{pos.x:6.3f}, {pos.y:6.3f}, {pos.z:6.3f}]")
            else:
                print(f"{i:2}. {model:30} (estado no disponible)")
        print("="*50)


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    
    if len(sys.argv) < 2:
        print("Uso: ros2 run ur_onrobot_gazebo gazebo_utils.py [comando]")
        print("Comandos:")
        print("  list               - Listar modelos en el mundo")
        print("  clear              - Eliminar objetos de prueba")
        print("  get [modelo]       - Obtener estado de un modelo")
        print("  delete [modelo]    - Eliminar un modelo específico")
        return
    
    utils = GazeboUtils()
    command = sys.argv[1]
    
    try:
        if command == 'list':
            utils.list_models()
            
        elif command == 'clear':
            deleted = utils.clear_test_objects()
            print(f"Eliminados {deleted} objetos de prueba")
            
        elif command == 'get' and len(sys.argv) >= 3:
            model_name = sys.argv[2]
            state = utils.get_model_state(model_name)
            if state and state.success:
                pos = state.pose.position
                ori = state.pose.orientation
                print(f"\nEstado de {model_name}:")
                print(f"  Posición: [{pos.x}, {pos.y}, {pos.z}]")
                print(f"  Orientación: [{ori.x}, {ori.y}, {ori.z}, {ori.w}]")
                print(f"  Twist lin: [{state.twist.linear.x}, {state.twist.linear.y}, {state.twist.linear.z}]")
                print(f"  Twist ang: [{state.twist.angular.x}, {state.twist.angular.y}, {state.twist.angular.z}]")
            else:
                print(f"No se pudo obtener estado de {model_name}")
                
        elif command == 'delete' and len(sys.argv) >= 3:
            model_name = sys.argv[2]
            if utils.delete_model(model_name):
                print(f"Modelo {model_name} eliminado")
            else:
                print(f"No se pudo eliminar {model_name}")
                
        else:
            print(f"Comando desconocido: {command}")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        utils.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()