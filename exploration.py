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
        # Find free, unknown, and obstacle cells
        free_cells = (self.known_map == 0)
        unknown = (self.known_map == -1)
        obstacles = (self.known_map == 1)
        
        # Print debug info
        print(f"\nMap statistics:")
        print(f"Free cells: {np.sum(free_cells)}")
        print(f"Unknown cells: {np.sum(unknown)}")
        print(f"Obstacle cells: {np.sum(obstacles)}")
        
        # Create a temporary planning map for validation
        planning_map = np.where(self.known_map >= 0, self.known_map, 0)
        
        frontiers = []
        
        # Iterate through all free cells
        free_y, free_x = np.where(free_cells)
        for x, y in zip(free_x, free_y):
            # Skip if this point isn't valid in the planning map
            if planning_map[x, y] != 0:
                continue
                
            # Check 8-connected neighbors
            x_min, x_max = max(0, x-1), min(self.size[0], x+2)
            y_min, y_max = max(0, y-1), min(self.size[1], y+2)
            neighborhood = self.known_map[x_min:x_max, y_min:y_max]
            
            # Count unknown and obstacle neighbors
            unknown_count = np.sum(neighborhood == -1)
            obstacle_count = np.sum(neighborhood == 1)
            
            # A frontier cell must:
            # 1. Be free
            # 2. Have at least one unknown neighbor
            # 3. Not be completely surrounded by obstacles
            if unknown_count > 0 and obstacle_count < 7:  # Allow more freedom for frontiers
                # Check if this frontier is too close to existing ones
                min_dist = 2  # Minimum distance between frontiers
                too_close = False
                for fx, fy in frontiers:
                    if np.hypot(x - fx, y - fy) < min_dist:
                        too_close = True
                        break
                
                if not too_close and self.is_valid_frontier((x, y), planning_map):
                    frontiers.append((x, y))
                    print(f"Found frontier at ({x}, {y}) with {unknown_count} unknown neighbors")
        
        if not frontiers:
            return []
            
        print(f"\nFound {len(frontiers)} potential frontiers")
        
        # Sort frontiers by exploration potential
        scored_frontiers = []
        for fx, fy in frontiers:
            # Calculate the number of unknown neighbors again for scoring
            x_min, x_max = max(0, fx-1), min(self.size[0], fx+2)
            y_min, y_max = max(0, fy-1), min(self.size[1], fy+2)
            unknown_neighbors = np.sum(self.known_map[x_min:x_max, y_min:y_max] == -1)
            
            # Calculate distances from robot and current goal
            robot_x, robot_y = self.known_map.shape[0]//2, self.known_map.shape[1]//2
            dist_from_robot = np.hypot(fx - robot_x, fy - robot_y)
            
            # Score based on:
            # 1. Number of unknown neighbors (more is better)
            # 2. Distance from current position (closer is better)
            score = (2.0 * unknown_neighbors    # Weight unknown neighbors heavily
                    - 0.5 * dist_from_robot)    # Slight penalty for distance
            
            scored_frontiers.append(((fx, fy), score))
            
        # Sort by score (higher is better)
        scored_frontiers.sort(key=lambda x: -x[1])  # Negative for descending order
        
        # Print top frontiers with scores
        print(f"\nTop frontiers with scores:")
        for (fx, fy), score in scored_frontiers[:3]:
            print(f"Frontier at ({fx}, {fy}): score = {score:.2f}")
            
        # Update frontier map
        self.frontier_map = np.zeros(self.size, dtype=bool)
        for (fx, fy), _ in scored_frontiers:
            self.frontier_map[fx, fy] = True
            
        # Return the frontier positions only
        return [f[0] for f in scored_frontiers]
        
        # Label connected components
        from scipy.ndimage import label
        if not frontiers:
            return []
            
            # Get cluster points
            y_idx, x_idx = np.where(cluster)
            
            # Check each point in cluster for exploration potential
            for px, py in zip(x_idx, y_idx):
                # Extract local neighborhood
                x_min, x_max = max(0, px-2), min(self.size[0], px+3)
                y_min, y_max = max(0, py-2), min(self.size[1], py+3)
                neighborhood = unknown[y_min:y_max, x_min:x_max]
                
            if not frontiers:
                return []
            
        print(f"\nFound {len(frontiers)} potential frontiers")
        
        # Sort frontiers by exploration potential
        scored_frontiers = []
        for fx, fy in frontiers:
            # Calculate the number of unknown neighbors again for scoring
            x_min, x_max = max(0, fx-1), min(self.size[0], fx+2)
            y_min, y_max = max(0, fy-1), min(self.size[1], fy+2)
            unknown_neighbors = np.sum(self.known_map[x_min:x_max, y_min:y_max] == -1)
            
            # Calculate distances from robot and current goal
            robot_x, robot_y = self.known_map.shape[0]//2, self.known_map.shape[1]//2
            dist_from_robot = np.hypot(fx - robot_x, fy - robot_y)
            
            # Score based on:
            # 1. Number of unknown neighbors (more is better)
            # 2. Distance from current position (closer is better)
            score = (2.0 * unknown_neighbors    # Weight unknown neighbors heavily
                    - 0.5 * dist_from_robot)    # Slight penalty for distance
            
            scored_frontiers.append(((fx, fy), score))
            
        # Sort by score (higher is better)
        scored_frontiers.sort(key=lambda x: -x[1])  # Negative for descending order
        
        # Print top frontiers with scores
        print(f"\nTop frontiers with scores:")
        for (fx, fy), score in scored_frontiers[:3]:
            print(f"Frontier at ({fx}, {fy}): score = {score:.2f}")
            
        # Update frontier map
        self.frontier_map = np.zeros(self.size, dtype=bool)
        for (fx, fy), _ in scored_frontiers:
            self.frontier_map[fx, fy] = True
            
        # Return the frontier positions only
        return [f[0] for f in scored_frontiers]
        
    def is_valid_frontier(self, point, planning_map):
        """Check if a frontier point is valid and reachable."""
        x, y = int(point[0]), int(point[1])
        
        # Check bounds
        if not (0 <= x < self.size[0] and 0 <= y < self.size[1]):
            print(f"Frontier at ({x}, {y}) rejected: Out of bounds")
            return False
            
        # Check if point is known free space in both maps
        if self.known_map[x, y] != 0 or planning_map[x, y] != 0:
            print(f"Frontier at ({x}, {y}) rejected: Not in free space")
            return False
            
        # Check neighborhood for unknown cells
        x_min, x_max = max(0, x-1), min(self.size[0], x+2)
        y_min, y_max = max(0, y-1), min(self.size[1], y+2)
        neighborhood = self.known_map[x_min:x_max, y_min:y_max]
        
        # Must have at least one unknown neighbor and not be surrounded by obstacles
        has_unknown = np.any(neighborhood == -1)
        obstacle_count = np.sum(neighborhood == 1)
        
        if not has_unknown:
            print(f"Frontier at ({x}, {y}) rejected: No unknown neighbors")
            return False
            
        if obstacle_count >= 7:  # Leave at least one non-obstacle cell
            print(f"Frontier at ({x}, {y}) rejected: Too many obstacles nearby")
            return False
            
        return True
        
    def select_frontier(self, frontiers, robot_pos, planning_map, planner):
        """
        Select best frontier point based on exploration potential and reachability.
        
        Args:
            frontiers: List of frontier points
            robot_pos: Current robot position (which is also the goal)
            planning_map: Map used for path planning
            planner: A* planner function
            
        Returns:
            tuple: Selected frontier point in world coordinates, or None if no valid frontier
        """
        if not frontiers:
            return None
            
        robot_grid_x, robot_grid_y = to_grid_coords(robot_pos[0], robot_pos[1])
        goal_x, goal_y = robot_pos  # robot_pos is also the goal position
        
        # Create a list to store all candidates
        candidates = []
        
        # Calculate distance to goal in world coordinates
        dist_to_goal_world = np.hypot(robot_pos[0] - goal_x, robot_pos[1] - goal_y)
        near_goal = dist_to_goal_world < 5  # Check if we're near the goal
        
        # Process all frontiers first
        for frontier in frontiers:
            start = (int(robot_grid_x), int(robot_grid_y))
            frontier_point = (int(frontier[0]), int(frontier[1]))
            
            # Check if path exists to frontier
            path = planner(planning_map, start, frontier_point)
            if path is not None:
                path_length = sum(np.hypot(path[i+1][0] - path[i][0],
                                         path[i+1][1] - path[i][1])
                                for i in range(len(path)-1))
                
                # Calculate various metrics
                dist_to_frontier = np.hypot(frontier[0] - robot_grid_x, frontier[1] - robot_grid_y)
                dist_to_goal = np.hypot(frontier[0] - goal_x, frontier[1] - goal_y)
                
                if near_goal:
                    # Near goal mode: Prioritize getting to the goal
                    score = (-0.7 * dist_to_goal     # Heavy weight on goal distance
                            -0.2 * path_length       # Small penalty for path length
                            -0.1 * dist_to_frontier) # Minimal consideration for frontier distance
                else:
                    # Exploration mode: Focus on efficient frontier exploration
                    score = (-0.5 * dist_to_frontier # Prioritize closer frontiers
                            -0.3 * path_length       # Consider path efficiency
                            -0.2 * dist_to_goal)     # Small consideration for goal direction
                
                candidates.append({
                    'point': frontier,
                    'score': score,
                    'dist_to_goal': dist_to_goal,
                    'path_length': path_length
                })

        if not candidates:
            return None
            
        # Print all candidates and their scores for debugging
        print("\nCandidate evaluation:")
        for c in sorted(candidates, key=lambda x: -x['score']):
            print(f"Point {c['point']}: score={c['score']:.2f}, "
                  f"dist_to_goal={c['dist_to_goal']:.1f}, "
                  f"path_length={c['path_length']:.1f}")
        
        # Select the best candidate
        best_candidate = max(candidates, key=lambda x: x['score'])
        print(f"\nSelected point {best_candidate['point']} with score {best_candidate['score']:.2f}")
        
        # Record this frontier in history
        self.history['visited_frontiers'].append({
            'point': best_candidate['point'],
            'time': time.time() - self.start_time if self.start_time else 0,
            'coverage': self.get_coverage_metrics()['coverage']
        })
        
        # Return the selected point
        return best_candidate['point']
        
        if not valid_frontiers:
            return None
            
        # Score frontiers based on multiple criteria
        for f in valid_frontiers:
            # Reward exploration in new directions
            angle_penalty = 0
            for visited in self.history.get('visited_frontiers', []):
                angle_diff = abs(f['direction'] - visited['direction'])
                angle_penalty += np.exp(-angle_diff)
            
            # Calculate distances to goal and frontier
            goal_x, goal_y = robot_pos  # robot_pos is already the goal position passed from main.py
            dist_to_goal = np.hypot(f['point'][0] - goal_x, f['point'][1] - goal_y)
            
            # If goal is very close (within 5 units), strongly prefer frontiers near the goal
            near_goal = dist_to_goal < 5
            
            # Calculate score (higher is better)
            if near_goal:
                # When near goal, heavily prioritize frontiers closer to goal
                f['score'] = (-0.7 * dist_to_goal           # Strong preference for goal-ward frontiers
                            -0.2 * f['path_length']         # Less emphasis on path length
                            -0.1 * angle_penalty)           # Minimal direction penalty
            else:
                # Normal exploration mode
                f['score'] = (-0.4 * f['path_length']       # Path efficiency
                            -0.3 * dist_to_goal            # Goal direction
                            -0.2 * angle_penalty           # Avoid revisiting
                            -0.1 * f['distance'])          # Prefer closer frontiers
                
            # Add tiny randomness to break ties
            f['score'] += 0.05 * np.random.random()
        
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