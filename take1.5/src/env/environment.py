# src/env/environment.py

import numpy as np
from robot.ackermann import AckermannRobot
from map.occupancy_map import OccupancyMap
from sensors.lidar import LidarSensor

class RDPEnv:
    def __init__(self, map_size=(20, 20), resolution=0.05, visualize=False, mode='2d'):
        self.map = OccupancyMap(map_size, resolution)
        self.robot = AckermannRobot(x=2, y=2, theta=0, wheelbase=0.3)
        self.sensor = LidarSensor(self.map, fov=270, num_beams=108, max_range=5.0)
        self.visualize = visualize
        self.mode = mode
        self._init_renderer()
    
    def _init_renderer(self):
        if not self.visualize:
            return
        if self.mode == '2d':
            from env.env_2d import Renderer2D
            self.renderer = Renderer2D(self.map)
        else:
            from env.env_3d import Renderer3D
            self.renderer = Renderer3D(self.map)
    
    def step(self, v, delta):
        """Advance one timestep with control inputs"""
        self.robot.step(v, delta)
        ranges = self.sensor.scan(self.robot.x, self.robot.y, self.robot.theta)
        self.map.update_from_scan(self.robot.x, self.robot.y, ranges, self.sensor.angles)
        
        if self.visualize:
            self.renderer.render(self.robot, self.map)
        return ranges
    
    def reset(self):
        self.robot.reset(x=2, y=2, theta=0)
        self.map.reset()
        if self.visualize:
            self.renderer.reset()
