#!/usr/bin/env python3
"""
Demo environment for 3D visualization of Ackermann robot navigation.
"""

import sys
import os
import time
import numpy as np

# Add project root (take2/src) to Python path for relative imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from src.env.env_3d import Renderer3D
from src.objects.generator import generate_environment
from src.robot.ackermann import AckermannRobot
from src.sensors.lidar import LidarSensor
from src.env.env_2d_map import MapVisualizer

def main():
    # Initialize environment with obstacles
    print("Generating environment with 12 obstacles...")
    obstacles = generate_environment(num_obs=12)

    # Initialize robot at position (2, 2) with 0° heading
    print("Initializing robot...")
    robot = AckermannRobot(x=2, y=2, theta=0, wheelbase=0.3, dt=0.1)
    

    # Initialize 3D renderer
    print("Initializing 3D renderer...")
    renderer = Renderer3D(obstacles, map_size=(20, 20))
        
    lidar = LidarSensor()
    mapviz = MapVisualizer()

    try:
        print("Starting simulation... (Press Ctrl+C to exit)")
        for t in range(200):
            # Sinusoidal steering pattern for smooth motion
            v = 0.3
            delta = np.sin(t/50) * 0.3
            
            robot.step(v, delta)
            ranges = lidar.scan(robot, obstacles)
            mapviz.update(robot, ranges, lidar.angles)
            renderer.render(robot)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        print("Cleaning up...")
        if hasattr(renderer, "close"):
            renderer.close()
        print("Renderer closed successfully.")


if __name__ == "__main__":
    main()
