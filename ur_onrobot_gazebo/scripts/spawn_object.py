#!/usr/bin/env python3
"""
Script para spawnear objetos en Gazebo
Permite agregar objetos de prueba para manipulación
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import sys
import os


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /spawn_entity...')
        
        self.get_logger().info('Object Spawner listo')

    def spawn_cube(self, name, x=0.0, y=0.0, z=1.0, size=0.05, color='red'):
        """Spawnear un cubo en la posición especificada"""
        
        # Determinar color
        if color == 'red':
            material = 'Gazebo/Red'
            rgba = '1 0 0 1'
        elif color == 'blue':
            material = 'Gazebo/Blue'
            rgba = '0 0 1 1'
        elif color == 'green':
            material = 'Gazebo/Green'
            rgba = '0 1 0 1'
        else:
            material = 'Gazebo/Grey'
            rgba = '0.5 0.5 0.5 1'
        
        # Crear SDF del cubo
        cube_sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box>
                    <size>{size} {size} {size}</size>
                  </box>
                </geometry>
                <surface>
                  <contact>
                    <ode>
                      <kp>10000000</kp>
                      <kd>1</kd>
                      <max_vel>0.01</max_vel>
                      <min_depth>0.001</min_depth>
                    </ode>
                  </contact>
                  <friction>
                    <ode>
                      <mu>0.8</mu>
                      <mu2>0.8</mu2>
                    </ode>
                  </friction>
                </surface>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>{size} {size} {size}</size>
                  </box>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>{material}</name>
                  </script>
                </material>
              </visual>
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.0001</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>0.0001</iyy>
                  <iyz>0</iyz>
                  <izz>0.0001</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>"""
        
        return self.spawn_entity(name, cube_sdf)

    def spawn_cylinder(self, name, x=0.0, y=0.0, z=1.0, radius=0.025, height=0.1, color='blue'):
        """Spawnear un cilindro"""
        
        if color == 'red':
            material = 'Gazebo/Red'
        elif color == 'blue':
            material = 'Gazebo/Blue'
        elif color == 'green':
            material = 'Gazebo/Green'
        else:
            material = 'Gazebo/Grey'
        
        cylinder_sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>{radius}</radius>
                    <length>{height}</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>{radius}</radius>
                    <length>{height}</length>
                  </cylinder>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>{material}</name>
                  </script>
                </material>
              </visual>
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.0001</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>0.0001</iyy>
                  <iyz>0</iyz>
                  <izz>0.0001</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>"""
        
        return self.spawn_entity(name, cylinder_sdf)

    def spawn_sphere(self, name, x=0.0, y=0.0, z=1.0, radius=0.03, color='green'):
        """Spawnear una esfera"""
        
        if color == 'red':
            material = 'Gazebo/Red'
        elif color == 'blue':
            material = 'Gazebo/Blue'
        elif color == 'green':
            material = 'Gazebo/Green'
        else:
            material = 'Gazebo/Grey'
        
        sphere_sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <static>false</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <sphere>
                    <radius>{radius}</radius>
                  </sphere>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <sphere>
                    <radius>{radius}</radius>
                  </sphere>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>{material}</name>
                  </script>
                </material>
              </visual>
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.0001</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>0.0001</iyy>
                  <iyz>0</iyz>
                  <izz>0.0001</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>"""
        
        return self.spawn_entity(name, sphere_sdf)

    def spawn_entity(self, name, sdf):
        """Función genérica para spawnear entidades"""
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf
        request.robot_namespace = ''
        request.reference_frame = 'world'
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Objeto {name} spawnado exitosamente')
                return True
            else:
                self.get_logger().error(f'Error al spawnear {name}: {future.result().status_message}')
                return False
        else:
            self.get_logger().error('Servicio no respondió')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Uso: ros2 run ur_onrobot_gazebo spawn_object.py [tipo] [opciones]")
        print("Tipos: cube, cylinder, sphere")
        print("Ejemplo: ros2 run ur_onrobot_gazebo spawn_object.py cube --x 0.5 --y 0.0 --z 1.0 --color red")
        return
    
    spawner = ObjectSpawner()
    
    # Parsear argumentos simples
    obj_type = sys.argv[1]
    name = f"test_{obj_type}_{len(sys.argv)}"
    
    # Valores por defecto
    x, y, z = 0.0, 0.0, 1.0
    color = 'red'
    
    # Parsear argumentos (simple)
    for i in range(2, len(sys.argv)):
        if sys.argv[i] == '--x' and i+1 < len(sys.argv):
            x = float(sys.argv[i+1])
        elif sys.argv[i] == '--y' and i+1 < len(sys.argv):
            y = float(sys.argv[i+1])
        elif sys.argv[i] == '--z' and i+1 < len(sys.argv):
            z = float(sys.argv[i+1])
        elif sys.argv[i] == '--color' and i+1 < len(sys.argv):
            color = sys.argv[i+1]
        elif sys.argv[i] == '--name' and i+1 < len(sys.argv):
            name = sys.argv[i+1]
    
    success = False
    if obj_type == 'cube':
        success = spawner.spawn_cube(name, x, y, z, color=color)
    elif obj_type == 'cylinder':
        success = spawner.spawn_cylinder(name, x, y, z, color=color)
    elif obj_type == 'sphere':
        success = spawner.spawn_sphere(name, x, y, z, color=color)
    else:
        print(f"Tipo de objeto desconocido: {obj_type}")
    
    if success:
        print(f"Objeto {name} spawnado en ({x}, {y}, {z})")
    
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()