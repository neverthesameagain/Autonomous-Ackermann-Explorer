# src/env/env_2d_map.py
import numpy as np, matplotlib.pyplot as plt
class MapVisualizer:
    def __init__(self, map_size=(20,20), res=0.1):
        self.res = res
        self.grid = np.zeros((int(map_size[0]/res), int(map_size[1]/res)))
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(self.grid, cmap='gray', origin='lower')

    def update(self, robot, ranges, angles):
        for r,a in zip(ranges, angles):
            x = robot.x + r*np.cos(robot.theta + a)
            y = robot.y + r*np.sin(robot.theta + a)
            i,j = int(x/self.res), int(y/self.res)
            if 0<=i<self.grid.shape[0] and 0<=j<self.grid.shape[1]:
                self.grid[i,j] = 1
        self.im.set_data(self.grid)
        plt.draw(); plt.pause(0.001)
