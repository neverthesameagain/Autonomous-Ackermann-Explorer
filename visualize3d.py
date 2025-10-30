import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Visualizer3D:
    def __init__(self, grid_size, obstacle_height=1.0):
        """
        Initialize 3D visualization
        
        Args:
            grid_size: Size of the occupancy grid
            obstacle_height: Height of obstacle boxes in 3D view
        """
        self.grid_size = grid_size
        self.obstacle_height = obstacle_height
        
        # Setup 3D plot
        self.fig = plt.figure(figsize=(12, 5))
        
        # 2D subplot
        self.ax2d = self.fig.add_subplot(121)
        self.ax2d.set_aspect('equal')
        
        # 3D subplot
        self.ax3d = self.fig.add_subplot(122, projection='3d')
        self.ax3d.view_init(elev=30, azim=45)
        self.ax3d.set_box_aspect([1, 1, 0.5])
        
    def _create_box(self, x, y):
        """Create vertices and faces for a 3D box"""
        z = 0
        h = self.obstacle_height
        vertices = np.array([
            [x, y, z], [x+1, y, z], [x+1, y+1, z], [x, y+1, z],
            [x, y, z+h], [x+1, y, z+h], [x+1, y+1, z+h], [x, y+1, z+h]
        ])
        
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # left
            [vertices[1], vertices[2], vertices[6], vertices[5]]   # right
        ]
        return faces
        
    def update(self, grid, robot, path=None):
        """
        Update both 2D and 3D visualizations
        
        Args:
            grid: Occupancy grid (2D numpy array)
            robot: AckermannRobot instance
            path: Optional list of (x,y) path points
        """
        # Clear previous plots
        self.ax2d.clear()
        self.ax3d.clear()
        
        # 2D Plot
        self.ax2d.imshow(grid.T, cmap='gray_r', origin='lower')
        
        # Plot path if provided
        if path:
            px, py = zip(*path)
            self.ax2d.plot(px, py, 'b--', label='Planned Path')
            
        # Plot robot trajectory
        traj_x = robot.history['x']
        traj_y = robot.history['y']
        self.ax2d.plot(traj_x, traj_y, 'r-', label='Robot Trajectory')
        
        # Plot robot position
        self.ax2d.plot(robot.x, robot.y, 'go', label='Robot')
        
        self.ax2d.legend()
        self.ax2d.set_title('2D View')
        
        # 3D Plot
        # Add obstacle boxes
        obstacles = []
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if grid[i,j] == 1:
                    box_faces = self._create_box(i, j)
                    obstacles.append(box_faces)
        
        if obstacles:
            obstacle_collection = Poly3DCollection(
                np.concatenate(obstacles),
                facecolors='gray',
                alpha=0.25,
                edgecolors='k'
            )
            self.ax3d.add_collection3d(obstacle_collection)
        
        # Plot robot trajectory in 3D
        self.ax3d.plot(traj_x, traj_y, np.zeros_like(traj_x),
                      'r-', label='Robot Trajectory')
        
        # Plot path in 3D if provided
        if path:
            self.ax3d.plot(px, py, np.zeros_like(px),
                          'b--', label='Planned Path')
        
        # Set 3D plot limits and labels
        self.ax3d.set_xlim(0, self.grid_size)
        self.ax3d.set_ylim(0, self.grid_size)
        self.ax3d.set_zlim(0, self.obstacle_height)
        self.ax3d.set_title('3D View')
        
        plt.draw()
        plt.pause(0.01)  # Allow animation update