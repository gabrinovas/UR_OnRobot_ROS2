#!/usr/bin/env python3
"""
Script para spawn rápido de objetos de prueba
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import time


def spawn_test_scene():
    """Spawnear una escena de prueba completa"""
    from scripts.spawn_object import ObjectSpawner
    
    rclpy.init()
    spawner = ObjectSpawner()
    
    print("Spawnando escena de prueba...")
    
    # Spawnear objetos en posiciones específicas
    objects = [
        ('red_cube', 'cube', 0.5, 0.0, 1.0, 'red'),
        ('blue_cube', 'cube', 0.6, 0.2, 1.0, 'blue'),
        ('green_cube', 'cube', 0.4, -0.2, 1.0, 'green'),
        ('test_cylinder', 'cylinder', 0.3, 0.3, 1.0, 'blue'),
        ('test_sphere', 'sphere', 0.7, -0.1, 1.0, 'red'),
    ]
    
    for name, obj_type, x, y, z, color in objects:
        print(f"Spawnando {name}...")
        if obj_type == 'cube':
            spawner.spawn_cube(name, x, y, z, 0.05, color)
        elif obj_type == 'cylinder':
            spawner.spawn_cylinder(name, x, y, z, 0.025, 0.1, color)
        elif obj_type == 'sphere':
            spawner.spawn_sphere(name, x, y, z, 0.03, color)
        time.sleep(0.5)
    
    print("Escena de prueba lista!")
    spawner.destroy_node()
    rclpy.shutdown()


def main():
    print("Quick Spawn - Opciones:")
    print("1. Escena de prueba completa")
    print("2. Cubo rojo en posición central")
    print("3. Tres cubos en línea")
    print("4. Objetos para pick & place")
    
    choice = input("\nSelecciona opción (1-4): ").strip()
    
    rclpy.init()
    spawner = ObjectSpawner()
    
    try:
        if choice == '1':
            # Escena completa en un hilo separado
            thread = threading.Thread(target=spawn_test_scene)
            thread.start()
            thread.join()
            
        elif choice == '2':
            spawner.spawn_cube('test_cube', 0.5, 0.0, 1.0, 0.05, 'red')
            print("Cubo rojo spawnado en (0.5, 0.0, 1.0)")
            
        elif choice == '3':
            positions = [(0.4, 0.0), (0.5, 0.0), (0.6, 0.0)]
            colors = ['red', 'blue', 'green']
            for i, (x, y) in enumerate(positions):
                spawner.spawn_cube(f'cube_{i}', x, y, 1.0, 0.05, colors[i])
                time.sleep(0.3)
            print("Tres cubos spawnados en línea")
            
        elif choice == '4':
            # Objetos para demo de pick & place
            spawner.spawn_cube('pick_object', 0.5, 0.0, 1.0, 0.05, 'red')
            time.sleep(0.3)
            spawner.spawn_cylinder('place_target', 0.0, 0.5, 1.0, 0.03, 0.08, 'blue')
            print("Objetos para pick & place spawnados")
            
        else:
            print("Opción no válida")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()