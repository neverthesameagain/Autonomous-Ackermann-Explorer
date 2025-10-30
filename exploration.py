import numpy as np
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
import time
from math import atan2, pi, cos, sin
from utils import to_grid_coords, to_world_coords

class Explorer:
    def __init__(self, size=(30, 30), sensor_range=3, min_frontier_size=3):
        """
        Initialize the exploration module.
        
        Args:
            size: Tuple of (width, height) for the map
            sensor_range: Radius of robot's sensor view in cells
            min_frontier_size: Minimum cluster size to consider as frontier
        """
        self.size = size
        self.sensor_range = sensor_range
        self.min_frontier_size = min_frontier_size
        
        # Initialize maps
        self.known_map = np.full(size, -1)  # -1: unknown, 0: free, 1: obstacle
        self.frontier_map = np.zeros(size, dtype=bool)
        
        # Metrics and history
        self.frontiers_visited = 0
        self.total_distance = 0.0
        self.coverage_history = []
        self.start_time = None
        
        # Exploration history
        self.history = {
            'visited_frontiers': [],  # Track visited frontiers
            'coverage_progress': [],   # Track coverage over time
            'exploration_events': []   # Track significant events
        }
        
    def find_frontiers(self):
        """
        Find valid frontier cells (free cells adjacent to unknown space).
        Returns list of (x,y) frontier centroids, filtered and ranked by exploration potential.
        """
        # Parameters for frontier filtering
        min_dist_between_frontiers = 3  # Minimum distance between frontier centroids
        min_unknown_ratio = 0.3  # Minimum ratio of unknown cells in frontier neighborhood
        
        # Create kernels for neighbor checking
        kernel = np.ones((3, 3))
        observation_kernel = np.ones((5, 5))  # Larger kernel for unknown ratio check
        
        # Find free and unknown cells
        free_cells = (self.known_map == 0)
        unknown = (self.known_map == -1)
        obstacles = (self.known_map == 1)
        
        # Dilate obstacles slightly to avoid frontiers too close to walls
        obstacles_dilated = binary_dilation(obstacles, np.ones((3, 3)))
        valid_area = free_cells & ~obstacles_dilated
        
        # Find frontier candidates (free cells next to unknown)
        unknown_border = binary_dilation(unknown, kernel) & valid_area
        
        # Label connected components
        from scipy.ndimage import label
        labels, num_features = label(unknown_border)
        
        frontiers = []
        visited_positions = set()
        
        # Process each frontier cluster
        for i in range(1, num_features + 1):
            cluster = (labels == i)
            
            if np.sum(cluster) < self.min_frontier_size:
                continue
            
            # Get cluster points
            y_idx, x_idx = np.where(cluster)
            
            # Check each point in cluster for exploration potential
            for px, py in zip(x_idx, y_idx):
                # Extract local neighborhood
                x_min, x_max = max(0, px-2), min(self.size[0], px+3)
                y_min, y_max = max(0, py-2), min(self.size[1], py+3)
                neighborhood = unknown[y_min:y_max, x_min:x_max]
                
                # Calculate ratio of unknown cells in neighborhood
                unknown_ratio = np.sum(neighborhood) / neighborhood.size
                
                if unknown_ratio < min_unknown_ratio:
                    continue
                
                # Check if point is far enough from existing frontiers
                too_close = False
                for fx, fy in visited_positions:
                    if np.hypot(px - fx, py - fy) < min_dist_between_frontiers:
                        too_close = True
                        break
                
                if too_close:
                    continue
                
                # This is a valid frontier point
                frontiers.append((px, py))
                visited_positions.add((px, py))
        
        # Sort frontiers by exploration potential (distance from robot and unknown ratio)
        if frontiers:
            robot_pos = np.array([self.known_map.shape[0]//2, self.known_map.shape[1]//2])
            frontiers.sort(key=lambda f: (
                # Prefer points further from robot but not too far
                -0.7 * min(np.hypot(f[0] - robot_pos[0], f[1] - robot_pos[1]), 15.0) +
                # Prefer points with more unknown neighbors
                0.3 * np.sum(unknown[max(0, int(f[1])-2):min(self.size[1], int(f[1])+3),
                                   max(0, int(f[0])-2):min(self.size[0], int(f[0])+3)])
            ))
        
        self.frontier_map = unknown_border
        return frontiers
        
    def is_valid_frontier(self, point, planning_map):
        """Check if a frontier point is valid and reachable."""
        x, y = int(point[0]), int(point[1])
        
        # Check bounds
        if not (0 <= x < self.size[0] and 0 <= y < self.size[1]):
            return False
            
        # Check if point is in free space
        if planning_map[x, y] != 0:
            return False
            
        # Check neighborhood for unknown cells
        x_min, x_max = max(0, x-1), min(self.size[0], x+2)
        y_min, y_max = max(0, y-1), min(self.size[1], y+2)
        neighborhood = self.known_map[x_min:x_max, y_min:y_max]
        
        # Must have at least one unknown neighbor
        return np.any(neighborhood == -1)
        
    def select_frontier(self, frontiers, robot_pos, planning_map, planner):
        """
        Select best frontier point based on exploration potential and reachability.
        
        Args:
            frontiers: List of frontier points
            robot_pos: Current robot position
            planning_map: Map used for path planning
            planner: A* planner function
            
        Returns:
            tuple: Selected frontier point in world coordinates, or None if no valid frontier
        """
        if not frontiers:
            return None
            
        robot_grid_x, robot_grid_y = to_grid_coords(robot_pos[0], robot_pos[1])
        valid_frontiers = []
        
        # Filter frontiers by validity and reachability
        for frontier in frontiers:
            if not self.is_valid_frontier(frontier, planning_map):
                continue
                
            # Check if path exists to frontier
            start = (int(robot_grid_x), int(robot_grid_y))
            goal = (int(frontier[0]), int(frontier[1]))
            
            path = planner(planning_map, start, goal)
            if path is not None:
                # Calculate path length
                path_length = 0
                for i in range(len(path)-1):
                    dx = path[i+1][0] - path[i][0]
                    dy = path[i+1][1] - path[i][1]
                    path_length += np.hypot(dx, dy)
                    
                valid_frontiers.append({
                    'point': frontier,
                    'distance': np.hypot(frontier[0] - robot_grid_x,
                                       frontier[1] - robot_grid_y),
                    'path_length': path_length,
                    'direction': np.arctan2(frontier[1] - robot_grid_y,
                                          frontier[0] - robot_grid_x)
                })
        
        if not valid_frontiers:
            return None
            
        # Score frontiers based on multiple criteria
        for f in valid_frontiers:
            # Reward exploration in new directions
            angle_penalty = 0
            for visited in self.history.get('visited_frontiers', []):
                angle_diff = abs(f['direction'] - visited['direction'])
                angle_penalty += np.exp(-angle_diff)
            
            # Calculate score (higher is better)
            f['score'] = (
                1.0 * f['distance'] +  # Prefer distant frontiers
                -0.5 * f['path_length'] +  # But consider path length
                -0.3 * angle_penalty +  # Penalize similar directions
                0.2 * np.random.random()  # Add small randomness
            )
        
        # Select frontier with highest score
        selected = max(valid_frontiers, key=lambda x: x['score'])
        
            # Record visited frontier with timestamp
        self.history['visited_frontiers'].append({
            'direction': selected['direction'],
            'point': selected['point'],
            'time': time.time() - self.start_time if self.start_time else 0,
            'coverage': self.get_coverage_metrics()['coverage']
        })
        self.frontiers_visited += 1  # Increment frontier counter        # Convert to world coordinates
        world_x, world_y = to_world_coords(selected['point'][0], selected['point'][1])
        return (world_x, world_y)
        
    def update_map_with_sensor(self, true_map, robot_pos):
        """
        Update known_map based on robot's sensor reading at current position.
        """
        x, y = robot_pos[0], robot_pos[1]
        grid_x, grid_y = to_grid_coords(x, y)
        
        # Create circular mask for sensor range
        Y, X = np.ogrid[:self.size[0], :self.size[1]]
        dist = np.sqrt((X - grid_x)**2 + (Y - grid_y)**2)
        mask = dist <= self.sensor_range
        
        # Update known map within sensor range
        self.known_map[mask] = true_map[mask]
        
        # Update metrics
        total_cells = self.size[0] * self.size[1]
        known_cells = np.sum(self.known_map >= 0)
        coverage = known_cells / total_cells * 100
        
        if self.start_time is None:
            self.start_time = time.time()
            
        self.coverage_history.append({
            'time': time.time() - self.start_time,
            'coverage': coverage
        })
        
    def get_coverage_near_point(self, point, radius=5):
        """
        Calculate exploration coverage within a radius of a point
        
        Args:
            point: (x,y) coordinate
            radius: radius to check in grid cells
            
        Returns:
            float: Percentage of known cells in the region
        """
        x, y = int(point[0]), int(point[1])
        
        # Create circular mask
        Y, X = np.ogrid[:self.size[0], :self.size[1]]
        dist = np.sqrt((X - x)**2 + (Y - y)**2)
        mask = dist <= radius
        
        # Count known cells in region
        total_cells = np.sum(mask)
        if total_cells == 0:
            return 0.0
            
        known_cells = np.sum(self.known_map[mask] >= 0)
        return known_cells / total_cells
        
    def get_coverage_metrics(self):
        """Return exploration metrics."""
        total_cells = self.size[0] * self.size[1]
        known_cells = np.sum(self.known_map >= 0)
        coverage = known_cells / total_cells * 100
        
        return {
            'coverage': coverage,
            'frontiers_visited': self.frontiers_visited,
            'total_distance': self.total_distance,
            'coverage_history': self.coverage_history
        }
        
    def save_results(self, filepath_prefix):
        """Save exploration results and metrics."""
        # Save final map
        plt.figure(figsize=(8, 8))
        plt.imshow(self.known_map, cmap='gray')
        plt.colorbar(label='Cell State (-1: Unknown, 0: Free, 1: Obstacle)')
        plt.title('Final Explored Map')
        plt.savefig(f'{filepath_prefix}_final_map.png')
        
        # Save coverage history
        times = [x['time'] for x in self.coverage_history]
        coverages = [x['coverage'] for x in self.coverage_history]
        
        plt.figure(figsize=(8, 6))
        plt.plot(times, coverages)
        plt.xlabel('Time (s)')
        plt.ylabel('Coverage (%)')
        plt.title('Exploration Coverage vs Time')
        plt.grid(True)
        plt.savefig(f'{filepath_prefix}_coverage.png')
        
        # Save metrics as text
        metrics = self.get_coverage_metrics()
        with open(f'{filepath_prefix}_metrics.txt', 'w') as f:
            f.write(f"Final Coverage: {metrics['coverage']:.2f}%\n")
            f.write(f"Frontiers Visited: {metrics['frontiers_visited']}\n")
            f.write(f"Total Distance: {metrics['total_distance']:.2f}m\n")
        
        # Save raw data
        np.save(f'{filepath_prefix}_known_map.npy', self.known_map)