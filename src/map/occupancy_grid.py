"""
Occupancy Grid Map for autonomous exploration.
Maintains a probabilistic grid of free/occupied/unknown space.
"""
import numpy as np
from typing import Tuple, List

class OccupancyGrid:
    """2D occupancy grid with probabilistic updates."""
    
    # Cell states
    UNKNOWN = 0.5
    FREE = 0.0
    OCCUPIED = 1.0
    
    def __init__(self, map_size: Tuple[float, float] = (20, 20), resolution: float = 0.1):
        """
        Initialize occupancy grid.
        
        Args:
            map_size: (width, height) in meters
            resolution: grid cell size in meters
        """
        self.map_size = map_size
        self.resolution = resolution
        
        # Grid dimensions
        self.width = int(map_size[0] / resolution)
        self.height = int(map_size[1] / resolution)
        
        # Initialize grid with unknown values
        self.grid = np.ones((self.height, self.width)) * self.UNKNOWN
        
        # Log-odds representation for probabilistic updates
        self.log_odds = np.zeros((self.height, self.width))
        
        # Parameters for log-odds updates
        self.l_occ = 0.85   # Log-odds for occupied
        self.l_free = -0.4  # Log-odds for free
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates (cell center)."""
        x = (grid_x + 0.5) * self.resolution
        y = (grid_y + 0.5) * self.resolution
        return x, y
    
    def is_valid(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are within bounds."""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def update_from_lidar(self, robot_x: float, robot_y: float, robot_theta: float,
                          ranges: np.ndarray, angles: np.ndarray):
        """
        Update occupancy grid from LiDAR scan using ray tracing.
        
        Args:
            robot_x, robot_y: Robot position in world coordinates
            robot_theta: Robot heading in radians
            ranges: Array of range measurements
            angles: Array of beam angles (relative to robot)
        """
        robot_gx, robot_gy = self.world_to_grid(robot_x, robot_y)
        
        if not self.is_valid(robot_gx, robot_gy):
            return
        
        for r, angle in zip(ranges, angles):
            # Absolute angle of the beam
            beam_angle = robot_theta + angle
            
            # End point of the beam
            end_x = robot_x + r * np.cos(beam_angle)
            end_y = robot_y + r * np.sin(beam_angle)
            end_gx, end_gy = self.world_to_grid(end_x, end_y)
            
            # Ray trace from robot to end point
            cells = self._bresenham(robot_gx, robot_gy, end_gx, end_gy)
            
            # Mark all cells along ray as free (except last)
            for i, (gx, gy) in enumerate(cells[:-1]):
                if self.is_valid(gx, gy):
                    self.log_odds[gy, gx] += self.l_free
                    self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], -5, 5)
            
            # Mark end cell as occupied (if within max range)
            if len(cells) > 0 and r < 5.9:  # Assuming max range ~6m
                gx, gy = cells[-1]
                if self.is_valid(gx, gy):
                    self.log_odds[gy, gx] += self.l_occ
                    self.log_odds[gy, gx] = np.clip(self.log_odds[gy, gx], -5, 5)
        
        # Convert log-odds to probabilities
        self._update_probabilities()
    
    def _bresenham(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham's line algorithm for ray tracing."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            cells.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return cells
    
    def _update_probabilities(self):
        """Convert log-odds to probabilities."""
        self.grid = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
    
    def is_occupied(self, grid_x: int, grid_y: int, threshold: float = 0.65) -> bool:
        """Check if a cell is occupied."""
        if not self.is_valid(grid_x, grid_y):
            return True  # Out of bounds treated as occupied
        return self.grid[grid_y, grid_x] > threshold
    
    def is_free(self, grid_x: int, grid_y: int, threshold: float = 0.35) -> bool:
        """Check if a cell is free."""
        if not self.is_valid(grid_x, grid_y):
            return False
        return self.grid[grid_y, grid_x] < threshold
    
    def is_unknown(self, grid_x: int, grid_y: int) -> bool:
        """Check if a cell is unknown."""
        if not self.is_valid(grid_x, grid_y):
            return False
        return 0.35 <= self.grid[grid_y, grid_x] <= 0.65
    
    def get_grid(self) -> np.ndarray:
        """Get the occupancy grid."""
        return self.grid.copy()
    
    def reset(self):
        """Reset the grid to unknown state."""
        self.grid = np.ones((self.height, self.width)) * self.UNKNOWN
        self.log_odds = np.zeros((self.height, self.width))
