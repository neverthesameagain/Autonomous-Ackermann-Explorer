import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Visualizer3D:
    def __init__(self, grid_size, obstacle_height=1.0):
        """
        Initialize 3D visualization
        
        Args:
            grid_size: Tuple of (width, height) for the occupancy grid
            obstacle_height: Height of obstacle boxes in 3D view
        """
        if isinstance(grid_size, (list, tuple)) and len(grid_size) == 2:
            self.grid_width, self.grid_height = grid_size
        else:
            self.grid_width = self.grid_height = grid_size
            
        self.obstacle_height = obstacle_height
        
        # Setup 3D plot
        self.fig = plt.figure(figsize=(12, 5))
        
        # Store grid dimensions
        self.grid_width = grid_size[0]
        self.grid_height = grid_size[1]
        
        # 2D subplot
        self.ax2d = self.fig.add_subplot(121)
        self.ax2d.set_aspect('equal')
        
        # 3D subplot
        self.ax3d = self.fig.add_subplot(122, projection='3d')
        self.ax3d.view_init(elev=25, azim=0)  # Initial camera angle
        self.ax3d.set_box_aspect([1, 1, 0.5])
        
        # Set initial axis limits
        self.ax3d.set_xlim(0, self.grid_width)
        self.ax3d.set_ylim(0, self.grid_height)
        self.ax3d.set_zlim(0, self.obstacle_height + 2)
        
        # Camera parameters
        self.camera_distance = 5.0  # Distance behind robot
        self.camera_height = 3.0    # Height above ground
        self.camera_angle = 25      # Look-down angle in degrees
        
    def _calculate_camera_position(self, robot):
        """Calculate camera position and target based on robot pose"""
        # Get robot's heading angle
        heading = robot.theta
        
        # Calculate camera position behind robot (with boundary checks)
        cam_x = np.clip(robot.x - self.camera_distance * np.cos(heading), 0, self.grid_width)
        cam_y = np.clip(robot.y - self.camera_distance * np.sin(heading), 0, self.grid_height)
        cam_z = self.camera_height
        
        # Calculate look-at point (slightly ahead of robot, with boundary checks)
        look_x = np.clip(robot.x + 2.0 * np.cos(heading), 0, self.grid_width)
        look_y = np.clip(robot.y + 2.0 * np.sin(heading), 0, self.grid_height)
        look_z = 0.5  # Look slightly above ground
        
        return (cam_x, cam_y, cam_z), (look_x, look_y, look_z)
        
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
        
    def update(self, grid, robot, path=None, frontiers=None):
        """
        Update both 2D and 3D visualizations
        
        Args:
            grid: Occupancy grid (2D numpy array)
            robot: AckermannRobot instance
            path: Optional list of (x,y) path points
            frontiers: Optional list of frontier centroids
        """
        # Clear previous plots
        self.ax2d.clear()
        self.ax3d.clear()
        
        # 2D Plot - handle unknown cells
        cmap = plt.cm.gray_r
        if -1 in grid:  # If we have unknown cells
            grid_normalized = (grid + 1) / 2  # Scale from [-1,1] to [0,1]
            self.ax2d.imshow(grid_normalized.T, cmap=cmap, origin='lower',
                           vmin=0, vmax=1)
        else:
            self.ax2d.imshow(grid.T, cmap=cmap, origin='lower')
        
        # Plot frontiers if provided
        if frontiers:
            fx = [f[0] for f in frontiers]
            fy = [f[1] for f in frontiers]
            self.ax2d.scatter(fx, fy, c='magenta', marker='*',
                            label='Frontiers', s=100)
        
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
        # Update camera position
        cam_pos, look_at = self._calculate_camera_position(robot)
        self.ax3d.view_init(elev=self.camera_angle, azim=np.degrees(robot.theta) - 180)
        
        # Calculate view bounds with margin around robot
        margin = 5
        x_min = max(0, robot.x - margin)
        x_max = min(self.grid_width, robot.x + margin)
        y_min = max(0, robot.y - margin)
        y_max = min(self.grid_height, robot.y + margin)
        
        # Update view limits
        self.ax3d.set_xlim(x_min, x_max)
        self.ax3d.set_ylim(y_min, y_max)
        self.ax3d.set_zlim(0, self.obstacle_height + 2)
        
        # Update camera view based on robot position
        cam_pos, look_at = self._calculate_camera_position(robot)
        self.ax3d.view_init(elev=self.camera_angle, 
                          azim=np.degrees(robot.theta) - 180)
        
        # Clear previous 3D plot
        self.ax3d.clear()
        
        # Reset view limits
        self.ax3d.set_xlim(0, self.grid_width)
        self.ax3d.set_ylim(0, self.grid_height)
        self.ax3d.set_zlim(0, self.obstacle_height + 2)
        
        # 3D Plot - Add obstacle boxes
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
        self.ax3d.set_xlim(0, self.grid_width)
        self.ax3d.set_ylim(0, self.grid_height)
        self.ax3d.set_zlim(0, self.obstacle_height)
        self.ax3d.set_title('3D View')
        
        plt.draw()
        plt.pause(0.01)  # Allow animation update