# src/sensors/lidar.py
import numpy as np
class LidarSensor:
    def __init__(self, fov=270, num_beams=108, max_range=6.0):
        self.fov = np.deg2rad(fov)
        self.num_beams = num_beams
        self.max_range = max_range
        self.angles = np.linspace(-self.fov/2, self.fov/2, num_beams)

    def scan(self, robot, obstacles):
        """Return range array for each beam."""
        ranges = []
        for rel_a in self.angles:
            theta = robot.theta + rel_a
            r = self.max_range
            for d in np.linspace(0, self.max_range, 150):
                x = robot.x + d*np.cos(theta)
                y = robot.y + d*np.sin(theta)
                for obj in obstacles:
                    if obj.contains(x, y, 0):
                        r = d; break
                else:
                    continue
                break
            ranges.append(r)
        return np.array(ranges)
