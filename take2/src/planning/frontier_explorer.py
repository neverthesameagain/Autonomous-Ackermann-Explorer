"""
Frontier-based exploration for autonomous mapping.
Identifies and navigates to unexplored regions.
"""
import numpy as np
from typing import List, Tuple, Optional
from scipy.ndimage import label, center_of_mass

class FrontierExplorer:
    """Frontier-based exploration strategy."""
    
    def __init__(self, occupancy_grid, min_frontier_size: int = 10):
        """
        Initialize frontier explorer.
        
        Args:
            occupancy_grid: OccupancyGrid object
            min_frontier_size: Minimum number of cells to consider a frontier
        """
        self.grid = occupancy_grid
        self.min_frontier_size = min_frontier_size
    
    def find_frontiers(self) -> List[Tuple[float, float]]:
        """
        Find frontier cells (boundaries between free and unknown space).
        
        Returns:
            List of frontier centroids in world coordinates
        """
        frontiers = []
        frontier_map = np.zeros_like(self.grid.grid, dtype=bool)
        
        # Find cells that are free and adjacent to unknown cells
        for y in range(1, self.grid.height - 1):
            for x in range(1, self.grid.width - 1):
                if self.grid.is_free(x, y):
                    # Check 8-connected neighbors
                    has_unknown_neighbor = False
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            nx, ny = x + dx, y + dy
                            if self.grid.is_unknown(nx, ny):
                                has_unknown_neighbor = True
                                break
                        if has_unknown_neighbor:
                            break
                    
                    if has_unknown_neighbor:
                        frontier_map[y, x] = True
        
        # Group frontier cells into regions
        labeled_frontiers, num_frontiers = label(frontier_map)
        
        # Extract frontier centroids
        for frontier_id in range(1, num_frontiers + 1):
            frontier_cells = np.argwhere(labeled_frontiers == frontier_id)
            
            # Filter small frontiers
            if len(frontier_cells) < self.min_frontier_size:
                continue
            
            # Calculate centroid
            centroid_y, centroid_x = center_of_mass(labeled_frontiers == frontier_id)
            
            # Convert to world coordinates
            world_x, world_y = self.grid.grid_to_world(int(centroid_x), int(centroid_y))
            frontiers.append((world_x, world_y))
        
        return frontiers
    
    def select_best_frontier(self, robot_x: float, robot_y: float,
                            frontiers: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """
        Select the best frontier to explore based on distance and information gain.
        
        Args:
            robot_x, robot_y: Current robot position
            frontiers: List of frontier positions
            
        Returns:
            Best frontier position, or None if no frontiers
        """
        if not frontiers:
            return None
        
        best_frontier = None
        best_score = float('-inf')
        
        for fx, fy in frontiers:
            # Calculate distance
            dist = np.sqrt((fx - robot_x)**2 + (fy - robot_y)**2)
            
            # Calculate information gain (number of unknown cells nearby)
            info_gain = self._calculate_information_gain(fx, fy)
            
            # Score: balance between distance and information gain
            # Prefer closer frontiers with higher information gain
            score = info_gain / (1.0 + 0.5 * dist)
            
            if score > best_score:
                best_score = score
                best_frontier = (fx, fy)
        
        return best_frontier
    
    def _calculate_information_gain(self, x: float, y: float, radius: float = 2.0) -> float:
        """
        Calculate information gain at a position.
        
        Args:
            x, y: Position in world coordinates
            radius: Radius to check for unknown cells
            
        Returns:
            Information gain (number of unknown cells)
        """
        gx, gy = self.grid.world_to_grid(x, y)
        radius_cells = int(radius / self.grid.resolution)
        
        unknown_count = 0
        
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                nx, ny = gx + dx, gy + dy
                
                # Check if within radius
                if dx**2 + dy**2 > radius_cells**2:
                    continue
                
                if self.grid.is_valid(nx, ny) and self.grid.is_unknown(nx, ny):
                    unknown_count += 1
        
        return unknown_count
    
    def is_exploration_complete(self, threshold: float = 0.95) -> bool:
        """
        Check if exploration is complete.
        
        Args:
            threshold: Fraction of map that must be explored
            
        Returns:
            True if exploration is complete
        """
        total_cells = self.grid.width * self.grid.height
        known_cells = np.sum((self.grid.grid < 0.35) | (self.grid.grid > 0.65))
        
        exploration_ratio = known_cells / total_cells
        
        return exploration_ratio >= threshold
