# ================================================================
#   COMPILED PROJECT FILE
#   Automatically generated for unified analysis
# ================================================================



# ================================================================
# ==== FILE: compiled_project.py ====
# ================================================================





# ================================================================
# ==== FILE: knowit.py ====
# ================================================================

import os
from pathlib import Path

def compile_project_to_single_file(root_dir=".", output_file="compiled_project.py", exclude=None):
    """
    Combines all Python source files in the project into a single file.
    Each file is wrapped with header markers showing its path.
    """
    if exclude is None:
        exclude = ["venv", "__pycache__", "site-packages"]

    root_dir = Path(root_dir).resolve()
    output_path = root_dir / output_file

    with open(output_path, "w", encoding="utf-8") as outfile:
        outfile.write("# ================================================================\n")
        outfile.write("#   COMPILED PROJECT FILE\n")
        outfile.write("#   Automatically generated for unified analysis\n")
        outfile.write("# ================================================================\n\n")

        for folder, subdirs, files in os.walk(root_dir):
            # Skip unwanted directories
            if any(skip in folder for skip in exclude):
                continue

            for file in files:
                if not file.endswith(".py"):
                    continue
                file_path = Path(folder) / file

                outfile.write(f"\n\n# ================================================================\n")
                outfile.write(f"# ==== FILE: {file_path.relative_to(root_dir)} ====\n")
                outfile.write(f"# ================================================================\n\n")

                try:
                    with open(file_path, "r", encoding="utf-8") as infile:
                        content = infile.read()
                        outfile.write(content)
                        outfile.write("\n\n")
                except Exception as e:
                    outfile.write(f"# ⚠️ Could not read this file due to error: {e}\n\n")

        outfile.write("# ================================================================\n")
        outfile.write("# END OF COMPILED PROJECT\n")
        outfile.write("# ================================================================\n")

    print(f"✅ Project successfully compiled to: {output_path}")
    return str(output_path)


if __name__ == "__main__":
    # Change '.' to your project folder path if needed
    compile_project_to_single_file(root_dir="/Users/aryanmathur/Desktop/RDCP/take2")




# ================================================================
# ==== FILE: src/autonomous_explorer.py ====
# ================================================================

"""
Autonomous Exploration System
Integrates mapping, planning, and control for autonomous navigation.
"""
import numpy as np
import time
from typing import Optional, List, Tuple

from src.robot.ackermann import AckermannRobot
from src.sensors.lidar import LidarSensor
from src.map.occupancy_grid import OccupancyGrid
from src.planning.astar import AStarPlanner
from src.planning.frontier_explorer import FrontierExplorer
from src.control.velocity_controller import PathFollowingController, TrapezoidalVelocityController
from src.env.env_3d import Renderer3D
from src.objects.generator import generate_environment


class AutonomousExplorer:
    """Main autonomous exploration system."""
    
    def __init__(self, map_size: Tuple[float, float] = (20, 20), 
                 resolution: float = 0.1,
                 visualize: bool = True):
        """
        Initialize autonomous exploration system.
        
        Args:
            map_size: Environment size (width, height) in meters
            resolution: Grid resolution in meters
            visualize: Enable 3D visualization
        """
        self.map_size = map_size
        self.resolution = resolution
        self.visualize = visualize
        
        # Initialize components
        print("Initializing autonomous exploration system...")
        
        # Robot
        self.robot = AckermannRobot(x=2, y=2, theta=0, wheelbase=0.3, dt=0.1)
        print("✓ Robot initialized")
        
        # Sensors
        self.lidar = LidarSensor(fov=270, num_beams=108, max_range=6.0)
        print("✓ LiDAR sensor initialized")
        
        # Mapping
        self.occupancy_grid = OccupancyGrid(map_size=map_size, resolution=resolution)
        print("✓ Occupancy grid initialized")
        
        # Planning
        self.path_planner = AStarPlanner(self.occupancy_grid)
        self.frontier_explorer = FrontierExplorer(self.occupancy_grid, min_frontier_size=5)
        print("✓ Path planner and frontier explorer initialized")
        
        # Control
        velocity_controller = TrapezoidalVelocityController(
            max_velocity=0.5,
            max_acceleration=0.3,
            max_angular_velocity=0.8
        )
        self.path_controller = PathFollowingController(
            lookahead_distance=0.5,
            velocity_controller=velocity_controller
        )
        print("✓ Motion controller initialized")
        
        # Environment
        self.obstacles = generate_environment(num_obs=12, bounds=map_size)
        print(f"✓ Environment generated with {len(self.obstacles)} obstacles")
        
        # Visualization
        if self.visualize:
            self.renderer = Renderer3D(self.obstacles, map_size=map_size)
            print("✓ 3D renderer initialized")
        
        # State
        self.current_path = None
        self.current_goal = None
        self.exploration_complete = False
        self.total_distance = 0.0
        self.start_time = None
        
    def run_exploration(self, max_iterations: int = 2000, 
                       exploration_threshold: float = 0.85):
        """
        Run autonomous exploration.
        
        Args:
            max_iterations: Maximum number of simulation steps
            exploration_threshold: Fraction of map to explore before stopping
        """
        print("\n" + "="*60)
        print("STARTING AUTONOMOUS EXPLORATION")
        print("="*60)
        
        self.start_time = time.time()
        iteration = 0
        
        try:
            while iteration < max_iterations and not self.exploration_complete:
                iteration += 1
                
                # 1. Sense: Get LiDAR scan
                ranges = self.lidar.scan(self.robot, self.obstacles)
                
                # 2. Map: Update occupancy grid
                self.occupancy_grid.update_from_lidar(
                    self.robot.x, self.robot.y, self.robot.theta,
                    ranges, self.lidar.angles
                )
                
                # 3. Plan: Check if we need a new goal
                if self.current_goal is None or self._is_goal_reached():
                    # Find frontiers
                    frontiers = self.frontier_explorer.find_frontiers()
                    
                    if not frontiers:
                        print("\n✓ No more frontiers - Exploration complete!")
                        self.exploration_complete = True
                        break
                    
                    # Select best frontier
                    self.current_goal = self.frontier_explorer.select_best_frontier(
                        self.robot.x, self.robot.y, frontiers
                    )
                    
                    if self.current_goal is None:
                        print("\n✓ No reachable frontiers - Exploration complete!")
                        self.exploration_complete = True
                        break
                    
                    # Plan path to frontier
                    print(f"\n[Iteration {iteration}] Planning path to frontier at ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
                    self.current_path = self.path_planner.plan(
                        self.robot.x, self.robot.y,
                        self.current_goal[0], self.current_goal[1],
                        inflation_radius=3
                    )
                    
                    if self.current_path is None:
                        print("  ✗ No path found, selecting new frontier")
                        self.current_goal = None
                        continue
                    
                    print(f"  ✓ Path found with {len(self.current_path)} waypoints")
                
                # 4. Control: Follow path
                if self.current_path:
                    v, delta = self.path_controller.compute_control(
                        self.robot.x, self.robot.y, self.robot.theta,
                        self.current_path, self.robot.dt
                    )
                    
                    # Move robot
                    prev_x, prev_y = self.robot.x, self.robot.y
                    self.robot.step(v, delta)
                    
                    # Track distance
                    dist = np.sqrt((self.robot.x - prev_x)**2 + (self.robot.y - prev_y)**2)
                    self.total_distance += dist
                
                # 5. Visualize
                if self.visualize and iteration % 1 == 0:
                    self.renderer.render(self.robot)
                    time.sleep(0.01)
                
                # 6. Check exploration progress
                if iteration % 50 == 0:
                    self._print_status(iteration)
                
                # Check if exploration is complete
                if self.frontier_explorer.is_exploration_complete(exploration_threshold):
                    print(f"\n✓ Exploration threshold reached ({exploration_threshold*100:.1f}%)")
                    self.exploration_complete = True
                    break
        
        except KeyboardInterrupt:
            print("\n\n⚠ Exploration interrupted by user")
        
        finally:
            self._print_final_stats(iteration)
            if self.visualize:
                print("\nClose the visualization window to exit...")
                time.sleep(5)
                self.renderer.close()
    
    def _is_goal_reached(self) -> bool:
        """Check if current goal is reached."""
        if self.current_goal is None:
            return True
        
        return self.path_controller.is_goal_reached(
            self.robot.x, self.robot.y,
            self.current_goal[0], self.current_goal[1],
            tolerance=0.3
        )
    
    def _print_status(self, iteration: int):
        """Print current exploration status."""
        total_cells = self.occupancy_grid.width * self.occupancy_grid.height
        known_cells = np.sum(
            (self.occupancy_grid.grid < 0.35) | (self.occupancy_grid.grid > 0.65)
        )
        exploration_ratio = known_cells / total_cells
        
        elapsed_time = time.time() - self.start_time
        
        print(f"\n[Status - Iteration {iteration}]")
        print(f"  Position: ({self.robot.x:.2f}, {self.robot.y:.2f})")
        print(f"  Heading: {np.degrees(self.robot.theta):.1f}°")
        print(f"  Exploration: {exploration_ratio*100:.1f}%")
        print(f"  Distance traveled: {self.total_distance:.2f}m")
        print(f"  Time elapsed: {elapsed_time:.1f}s")
    
    def _print_final_stats(self, iterations: int):
        """Print final exploration statistics."""
        total_cells = self.occupancy_grid.width * self.occupancy_grid.height
        known_cells = np.sum(
            (self.occupancy_grid.grid < 0.35) | (self.occupancy_grid.grid > 0.65)
        )
        exploration_ratio = known_cells / total_cells
        
        elapsed_time = time.time() - self.start_time
        
        print("\n" + "="*60)
        print("EXPLORATION COMPLETE")
        print("="*60)
        print(f"Total iterations: {iterations}")
        print(f"Total distance traveled: {self.total_distance:.2f}m")
        print(f"Total time: {elapsed_time:.1f}s")
        print(f"Average speed: {self.total_distance/elapsed_time:.2f}m/s")
        print(f"Map explored: {exploration_ratio*100:.1f}%")
        print(f"Final position: ({self.robot.x:.2f}, {self.robot.y:.2f})")
        print("="*60)


def main():
    """Main entry point."""
    explorer = AutonomousExplorer(
        map_size=(20, 20),
        resolution=0.1,
        visualize=True
    )
    
    explorer.run_exploration(
        max_iterations=2000,
        exploration_threshold=0.85
    )


if __name__ == "__main__":
    main()




# ================================================================
# ==== FILE: src/goal_directed_explorer.py ====
# ================================================================

"""
Goal-Directed Autonomous Exploration
Navigate from start to goal while exploring and mapping the environment.
"""
import numpy as np
import time
import matplotlib.pyplot as plt
from typing import Optional, List, Tuple

from src.robot.ackermann import AckermannRobot
from src.sensors.lidar import LidarSensor
from src.map.occupancy_grid import OccupancyGrid
from src.planning.astar import AStarPlanner
from src.planning.frontier_explorer import FrontierExplorer
from src.control.velocity_controller import PathFollowingController, TrapezoidalVelocityController
from src.env.env_3d import Renderer3D
from src.objects.generator import generate_environment


class GoalDirectedExplorer:
    """Goal-directed exploration with 2D and 3D visualization."""
    
    def __init__(self, start_pos: Tuple[float, float], goal_pos: Tuple[float, float],
                 map_size: Tuple[float, float] = (20, 20), 
                 resolution: float = 0.1):
        """
        Initialize goal-directed exploration system.
        
        Args:
            start_pos: (x, y) starting position
            goal_pos: (x, y) goal position
            map_size: Environment size (width, height) in meters
            resolution: Grid resolution in meters
        """
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.map_size = map_size
        self.resolution = resolution
        
        # Initialize components
        print("="*70)
        print("GOAL-DIRECTED AUTONOMOUS EXPLORATION SYSTEM")
        print("="*70)
        print(f"Start Position: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")
        print(f"Goal Position:  ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})")
        print("="*70)
        
        # Robot
        self.robot = AckermannRobot(x=start_pos[0], y=start_pos[1], theta=0, 
                                    wheelbase=0.3, dt=0.1)
        print("✓ Robot initialized at start position")
        
        # Sensors
        self.lidar = LidarSensor(fov=270, num_beams=108, max_range=6.0)
        print("✓ LiDAR sensor initialized")
        
        # Mapping
        self.occupancy_grid = OccupancyGrid(map_size=map_size, resolution=resolution)
        print("✓ Occupancy grid initialized")
        
        # Planning
        self.path_planner = AStarPlanner(self.occupancy_grid)
        self.frontier_explorer = FrontierExplorer(self.occupancy_grid, min_frontier_size=5)
        print("✓ Path planner and frontier explorer initialized")
        
        # Control
        velocity_controller = TrapezoidalVelocityController(
            max_velocity=0.5, max_acceleration=0.3, max_angular_velocity=0.8
        )
        self.path_controller = PathFollowingController(
            lookahead_distance=0.5, velocity_controller=velocity_controller
        )
        print("✓ Motion controller initialized")
        
        # Environment (with obstacles between start and goal)
        self.obstacles = generate_environment(num_obs=12, bounds=map_size,
                                             start_pos=start_pos, goal_pos=goal_pos)
        print(f"✓ Environment generated with {len(self.obstacles)} obstacles")
        
        # Visualization
        self._setup_visualization()
        
        # State
        self.current_path = None
        self.current_goal = None
        self.goal_reached = False
        self.total_distance = 0.0
        self.start_time = None
        self.visited_positions = []
        self.all_frontiers = []  # Current active frontiers
        self.frontiers_visited = []  # History of visited frontiers
        self.frontier_count = 0
        self.show_all_frontiers = False  # Only show current target
        
        # Loop detection
        self.position_history = []  # Last 60 positions
        self.last_progress_check = 0
        self.best_distance_to_goal = float('inf')
        self.iterations_without_progress = 0
        
        # Frontier tracking (visit log and blacklist)
        self.visit_log = {}  # frontier_idx -> attempts
        self.blacklist = set()  # frontier indices to temporarily exclude
        self.blacklist_ttl = {}  # frontier_idx -> iterations remaining
        self.frontier_history = []  # Track visited frontier positions
        self.last_10_positions = []  # For loop detection
        
        # Compute proper inflation based on robot footprint
        # Use a reasonable safety margin for navigation
        safety_margin = 0.4  # meters clearance
        self.inflation_radius_m = safety_margin
        self.inflation_radius_cells = int(round(self.inflation_radius_m / resolution))  # 4 cells
        
        print(f"✓ Computed inflation: {self.inflation_radius_m:.2f}m ({self.inflation_radius_cells} cells)")
        
        # Scoring weights
        self.W_PATH = 0.45
        self.W_GOAL = 0.25
        self.W_INFO = 0.20
        self.W_ALIGN = 0.06
        self.W_VIS = 0.04
        
    def _setup_visualization(self):
        """Setup 2D and 3D visualization."""
        plt.ion()
        
        # Create figure with 2 subplots
        self.fig = plt.figure(figsize=(16, 7))
        
        # 2D Map (Left)
        self.ax_2d = self.fig.add_subplot(121)
        self.ax_2d.set_xlim(0, self.map_size[0])
        self.ax_2d.set_ylim(0, self.map_size[1])
        self.ax_2d.set_xlabel('X (m)', fontsize=12)
        self.ax_2d.set_ylabel('Y (m)', fontsize=12)
        self.ax_2d.set_title('2D Exploration Map', fontsize=14, fontweight='bold')
        self.ax_2d.grid(True, alpha=0.3)
        self.ax_2d.set_aspect('equal')
        
        # 3D Environment (Right)
        self.ax_3d = self.fig.add_subplot(122, projection='3d')
        self.ax_3d.set_xlim(0, self.map_size[0])
        self.ax_3d.set_ylim(0, self.map_size[1])
        self.ax_3d.set_zlim(0, 2)
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        self.ax_3d.set_title('3D Environment View', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        print("✓ 2D + 3D visualization initialized")
    
    def run_exploration(self, max_iterations: int = 3000):
        """
        Run goal-directed exploration.
        
        Args:
            max_iterations: Maximum number of simulation steps
        """
        print("\n" + "="*70)
        print("STARTING EXPLORATION")
        print("="*70)
        
        # Initial scan to identify frontiers
        print("\n🔍 INITIAL ENVIRONMENT SCAN")
        print("-" * 70)
        
        # Do initial LiDAR scans from start position
        for _ in range(5):
            ranges = self.lidar.scan(self.robot, self.obstacles)
            self.occupancy_grid.update_from_lidar(
                self.robot.x, self.robot.y, self.robot.theta,
                ranges, self.lidar.angles
            )
        
        # Identify initial frontiers
        initial_frontiers = self.frontier_explorer.find_frontiers()
        self.all_frontiers = initial_frontiers
        
        print(f"\n📍 INITIAL FRONTIERS DETECTED: {len(initial_frontiers)}")
        print("-" * 70)
        
        if initial_frontiers:
            for i, (fx, fy) in enumerate(initial_frontiers, 1):
                dist_to_frontier = np.sqrt((fx - self.robot.x)**2 + (fy - self.robot.y)**2)
                dist_to_goal = np.sqrt((self.goal_pos[0] - fx)**2 + (self.goal_pos[1] - fy)**2)
                info_gain = self.frontier_explorer._calculate_information_gain(fx, fy)
                
                print(f"  F{i}: Position ({fx:.2f}, {fy:.2f})")
                print(f"      Distance from robot: {dist_to_frontier:.2f}m")
                print(f"      Distance to goal: {dist_to_goal:.2f}m")
                print(f"      Information gain: {info_gain:.0f} cells")
                print()
        
        # Select best initial frontier
        if initial_frontiers:
            best_frontier = self._select_frontier_towards_goal(initial_frontiers)
            if best_frontier:
                frontier_idx = None
                for i, (fx, fy) in enumerate(initial_frontiers):
                    if abs(fx - best_frontier[0]) < 0.1 and abs(fy - best_frontier[1]) < 0.1:
                        frontier_idx = i + 1
                        break
                
                print(f"🎯 SELECTED BEST FRONTIER: F{frontier_idx} at ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})")
                print(f"   Strategy: Moving towards goal while exploring")
                print("-" * 70)
        
        self.start_time = time.time()
        iteration = 0
        
        try:
            while iteration < max_iterations and not self.goal_reached:
                iteration += 1
                
                # 1. Sense: Get LiDAR scan
                ranges = self.lidar.scan(self.robot, self.obstacles)
                
                # 2. Map: Update occupancy grid
                self.occupancy_grid.update_from_lidar(
                    self.robot.x, self.robot.y, self.robot.theta,
                    ranges, self.lidar.angles
                )
                
                # Track visited positions
                self.visited_positions.append((self.robot.x, self.robot.y))
                
                # 3. Plan: Decide next goal
                if self.current_goal is None or self._is_goal_reached():
                    # First, try to reach the final goal directly
                    direct_path = self.path_planner.plan(
                        self.robot.x, self.robot.y,
                        self.goal_pos[0], self.goal_pos[1],
                        inflation_radius=self.inflation_radius_cells
                    )
                    
                    if direct_path is not None:
                        print(f"\n[Iteration {iteration}] 🎯 Direct path to GOAL found!")
                        print(f"  Going straight to goal - no more frontiers needed")
                        self.current_goal = self.goal_pos
                        self.current_path = direct_path
                        print(f"  ✓ Final path with {len(self.current_path)} waypoints")
                    else:
                        # Find frontiers and explore
                        frontiers = self.frontier_explorer.find_frontiers()
                        self.all_frontiers = frontiers  # Store for visualization
                        
                        if not frontiers:
                            print("\n⚠ No frontiers found - trying direct path to goal")
                            self.current_goal = self.goal_pos
                            self.current_path = self.path_planner.plan(
                                self.robot.x, self.robot.y,
                                self.goal_pos[0], self.goal_pos[1],
                                inflation_radius=self.inflation_radius_cells
                            )
                            if self.current_path is None:
                                print("✗ Cannot reach goal - exploration failed")
                                break
                        else:
                            # Select nearest frontier towards goal
                            self.current_goal = self._select_frontier_towards_goal(frontiers)
                            
                            if self.current_goal is None:
                                print("\n⚠ No reachable frontiers")
                                break
                            
                            # Find which frontier number this is
                            frontier_idx = None
                            for i, (fx, fy) in enumerate(frontiers):
                                if abs(fx - self.current_goal[0]) < 0.1 and abs(fy - self.current_goal[1]) < 0.1:
                                    frontier_idx = i + 1
                                    break
                            
                            frontier_label = f"F{frontier_idx}" if frontier_idx else "Frontier"
                            print(f"\n[Iteration {iteration}] 🎯 Targeting {frontier_label} at ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
                            print(f"  Total frontiers detected: {len(frontiers)}")
                            
                            self.current_path = self.path_planner.plan(
                                self.robot.x, self.robot.y,
                                self.current_goal[0], self.current_goal[1],
                                inflation_radius=self.inflation_radius_cells
                            )
                            
                            if self.current_path is None:
                                print("  ✗ No path found, selecting new frontier")
                                self.current_goal = None
                                continue
                            
                            print(f"  ✓ Path planned with {len(self.current_path)} waypoints")
                            
                            # Calculate path length
                            path_length = 0
                            for i in range(len(self.current_path) - 1):
                                dx = self.current_path[i+1][0] - self.current_path[i][0]
                                dy = self.current_path[i+1][1] - self.current_path[i][1]
                                path_length += np.sqrt(dx**2 + dy**2)
                            print(f"  📏 Path length: {path_length:.2f}m")
                
                # 4. Control: Follow path
                if self.current_path:
                    v, delta = self.path_controller.compute_control(
                        self.robot.x, self.robot.y, self.robot.theta,
                        self.current_path, self.robot.dt
                    )
                    
                    # Move robot
                    prev_x, prev_y = self.robot.x, self.robot.y
                    self.robot.step(v, delta)
                    
                    # Track distance
                    dist = np.sqrt((self.robot.x - prev_x)**2 + (self.robot.y - prev_y)**2)
                    self.total_distance += dist
                
                # 5. Check if intermediate frontier reached
                if self.current_goal and self.current_goal != self.goal_pos and self._is_goal_reached():
                    self.frontiers_visited.append(self.current_goal)
                    self.frontier_count += 1
                    
                    print(f"\n{'='*70}")
                    print(f"✅ FRONTIER #{self.frontier_count} REACHED!")
                    print(f"   Position: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")
                    print(f"   Total distance traveled: {self.total_distance:.2f}m")
                    print(f"{'='*70}")
                    
                    # CLEAR old frontiers - they're no longer relevant
                    self.all_frontiers = []
                    self.current_goal = None
                    
                    # Re-scan environment from new position
                    print(f"\n🔍 SCANNING FROM NEW POSITION...")
                    for _ in range(5):  # More scans for better mapping
                        ranges = self.lidar.scan(self.robot, self.obstacles)
                        self.occupancy_grid.update_from_lidar(
                            self.robot.x, self.robot.y, self.robot.theta,
                            ranges, self.lidar.angles
                        )
                    
                    # Find NEW frontiers from this position
                    new_frontiers = self.frontier_explorer.find_frontiers()
                    
                    print(f"\n📍 NEW FRONTIERS DETECTED: {len(new_frontiers)}")
                    
                    if new_frontiers:
                        # Select BEST frontier using Hybrid A* strategy
                        best_frontier = self._select_frontier_towards_goal(new_frontiers)
                        
                        if best_frontier:
                            print(f"\n🎯 SELECTED BEST FRONTIER: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})")
                            print(f"   This will be Frontier #{self.frontier_count + 1}")
                            print("-" * 70)
                    else:
                        print("  No new frontiers - checking if goal is reachable...")
                    
                    print()
                
                # 6. Check if final goal reached
                if self._is_at_final_goal():
                    print(f"\n{'='*70}")
                    print("🎯 FINAL GOAL REACHED!")
                    print(f"{'='*70}")
                    self.goal_reached = True
                    break
                
                # 7. Visualize
                if iteration % 2 == 0:  # Update every 2 iterations for performance
                    self._update_visualization()
                    plt.pause(0.01)
                
                # 8. Loop detection and progress monitoring
                current_dist = np.sqrt((self.robot.x - self.goal_pos[0])**2 + 
                                      (self.robot.y - self.goal_pos[1])**2)
                
                # Track position history for loop detection
                self.position_history.append((self.robot.x, self.robot.y))
                if len(self.position_history) > 60:
                    self.position_history.pop(0)
                
                # Track last 10 positions for immediate loop detection
                self.last_10_positions.append((self.robot.x, self.robot.y))
                if len(self.last_10_positions) > 10:
                    self.last_10_positions.pop(0)
                
                # Immediate loop detection (every 10 iterations)
                if len(self.last_10_positions) >= 10 and iteration % 10 == 0:
                    # Calculate average displacement
                    avg_disp = np.mean([
                        np.hypot(self.last_10_positions[i][0] - self.last_10_positions[i-1][0],
                                self.last_10_positions[i][1] - self.last_10_positions[i-1][1])
                        for i in range(1, len(self.last_10_positions))
                    ])
                    
                    if avg_disp < 0.3:  # Moving less than 0.3m on average
                        print(f"\n⚠️  LOOP DETECTED! Average displacement: {avg_disp:.3f}m")
                        print(f"   Robot stuck in local area")
                        self.blacklist_recent_frontiers()
                        self.current_goal = None
                        self.current_path = None
                        self.last_10_positions.clear()
                        
                        # Boost exploration weight temporarily
                        self.W_INFO = 0.40  # Increase from 0.20
                        self.W_GOAL = 0.15  # Decrease from 0.25
                        print(f"   🔄 Boosting exploration (W_INFO={self.W_INFO}, W_GOAL={self.W_GOAL})")
                
                # Check for progress every 50 iterations
                if iteration % 50 == 0:
                    if current_dist < self.best_distance_to_goal - 0.5:
                        # Good progress!
                        self.best_distance_to_goal = current_dist
                        self.iterations_without_progress = 0
                        
                        # Reset weights to normal
                        self.W_INFO = 0.20
                        self.W_GOAL = 0.25
                    else:
                        # No progress
                        self.iterations_without_progress += 50
                    
                    # Detect if stuck for long time
                    if self.iterations_without_progress >= 150:
                        print(f"\n⚠️  WARNING: No progress for {self.iterations_without_progress} iterations!")
                        print(f"   Current distance: {current_dist:.2f}m")
                        print(f"   Best distance: {self.best_distance_to_goal:.2f}m")
                        print(f"   Forcing complete re-planning...")
                        
                        # Clear blacklist and try again
                        self.blacklist.clear()
                        self.blacklist_ttl.clear()
                        self.current_goal = None
                        self.current_path = None
                        self.iterations_without_progress = 0
                    
                    self._print_status(iteration)
                
                # Decay blacklist TTL
                expired = []
                for idx in self.blacklist_ttl:
                    self.blacklist_ttl[idx] -= 1
                    if self.blacklist_ttl[idx] <= 0:
                        expired.append(idx)
                for idx in expired:
                    self.blacklist.discard(idx)
                    del self.blacklist_ttl[idx]
        
        except KeyboardInterrupt:
            print("\n\n⚠ Exploration interrupted by user")
        
        finally:
            self._print_final_stats(iteration)
            print("\nKeep visualization window open to view results...")
            plt.ioff()
            plt.show()
    
    
 
        """Compute information gain at frontier."""
        return self.frontier_explorer._calculate_information_gain(fx, fy)
    
    def _direction_alignment(self, fx: float, fy: float) -> float:
        """
        Compute directional alignment score (0-1).
        1.0 = perfectly aligned with goal direction
        0.0 = perpendicular or opposite
        """
        # Vector from robot to frontier
        dx_rf = fx - self.robot.x
        dy_rf = fy - self.robot.y
        
        # Vector from robot to goal
        dx_rg = self.goal_pos[0] - self.robot.x
        dy_rg = self.goal_pos[1] - self.robot.y
        
        # Normalize
        mag_rf = np.sqrt(dx_rf**2 + dy_rf**2) + 1e-6
        mag_rg = np.sqrt(dx_rg**2 + dy_rg**2) + 1e-6
        
        # Dot product (cosine similarity)
        cos_sim = (dx_rf * dx_rg + dy_rf * dy_rg) / (mag_rf * mag_rg)
        
        # Convert to 0-1 range (1 = aligned, 0 = opposite)
        return (cos_sim + 1.0) / 2.0
    
    def _recent_visit_penalty(self, fx: float, fy: float) -> float:
        """
        Penalize frontiers visited recently.
        Returns 0-1 (0 = never visited, 1 = visited many times)
        """
        # Check if this frontier is close to any recently visited
        for hist_fx, hist_fy in self.frontier_history[-10:]:
            dist = np.sqrt((fx - hist_fx)**2 + (fy - hist_fy)**2)
            if dist < 1.0:  # Within 1m of previous frontier
                return 1.0
        return 0.0
    
    def _record_frontier_visit(self, fx: float, fy: float):
        """Record that we visited this frontier."""
        self.frontier_history.append((fx, fy))
        if len(self.frontier_history) > 50:
            self.frontier_history.pop(0)
    
    def blacklist_recent_frontiers(self):
        """Blacklist recently visited frontiers to force exploration."""
        print("🚫 Blacklisting recent frontiers to escape loop")
        # Add last 5 visited frontiers to blacklist
        for fx, fy in self.frontier_history[-5:]:
            # Find matching frontier index
            for idx, (f_x, f_y) in enumerate(self.all_frontiers):
                if abs(fx - f_x) < 0.5 and abs(fy - f_y) < 0.5:
                    self.blacklist.add(idx)
                    self.blacklist_ttl[idx] = 100  # Blacklist for 100 iterations
    
    def _select_frontier_towards_goal(self, frontiers: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """
        Select frontier using expert-recommended scoring function.
        
        Scoring combines:
        - Path cost (A* distance to frontier)
        - Goal cost (distance from frontier to goal)
        - Info gain (unknown cells near frontier)
        - Angle alignment (towards goal direction)
        - Visit penalty (avoid repeated attempts)
        """
        if not frontiers:
            return None
        
        EPS = 1e-6
        current_dist_to_goal = np.sqrt((self.goal_pos[0] - self.robot.x)**2 + 
                                       (self.goal_pos[1] - self.robot.y)**2)
        
        goal_angle = np.arctan2(self.goal_pos[1] - self.robot.y, 
                               self.goal_pos[0] - self.robot.x)
        
        print(f"\n🔍 EVALUATING {len(frontiers)} FRONTIERS (Expert Scoring):")
        print(f"   Current distance to goal: {current_dist_to_goal:.2f}m")
        print("-" * 70)
        
        # Phase 1: Quick filter with coarse checks
        candidates = []
        for idx, (fx, fy) in enumerate(frontiers):
            # Skip blacklisted
            if idx in self.blacklist:
                continue
            
            dist_to_frontier = np.sqrt((fx - self.robot.x)**2 + (fy - self.robot.y)**2)
            
            # Filter: must be at least 0.5m away
            if dist_to_frontier < 0.5:
                continue
            
            dist_to_goal = np.sqrt((self.goal_pos[0] - fx)**2 + (self.goal_pos[1] - fy)**2)
            info_gain = self.frontier_explorer._calculate_information_gain(fx, fy)
            
            # Filter: must have some info gain
            if info_gain < 3:
                continue
            
            candidates.append((idx, fx, fy, dist_to_frontier, dist_to_goal, info_gain))
        
        if not candidates:
            print("❌ No valid candidates after filtering")
            return None
        
        # Phase 2: Score top-K with A* path costs
        # Sort by cheap heuristic first
        candidates.sort(key=lambda c: c[4] - 0.2 * c[5])  # goal_dist - 0.2*info_gain
        top_k = candidates[:min(8, len(candidates))]
        
        scored = []
        path_costs = []
        
        for idx, fx, fy, d_frontier, d_goal, info_gain in top_k:
            # Plan path to frontier
            path = self.path_planner.plan(
                self.robot.x, self.robot.y,
                fx, fy,
                inflation_radius=self.inflation_radius_cells
            )
            
            if path is None:
                continue
            
            # Calculate path cost (length)
            path_cost = sum(np.sqrt((path[i+1][0] - path[i][0])**2 + 
                                   (path[i+1][1] - path[i][1])**2)
                           for i in range(len(path) - 1))
            
            path_costs.append(path_cost)
            
            # Angle alignment
            frontier_angle = np.arctan2(fy - self.robot.y, fx - self.robot.x)
            angle_diff = abs(np.arctan2(np.sin(frontier_angle - goal_angle),
                                       np.cos(frontier_angle - goal_angle)))
            
            # Visit penalty
            visit_count = self.visit_log.get(idx, 0)
            visit_penalty = min(1.0, visit_count / 5.0)
            
            scored.append((idx, fx, fy, path_cost, d_goal, info_gain, angle_diff, visit_penalty, path))
        
        if not scored:
            print("❌ No feasible paths found to any frontier")
            return None
        
        # Phase 3: Normalize and compute final scores
        costs = [s[3] for s in scored]
        goals = [s[4] for s in scored]
        infos = [s[5] for s in scored]
        
        min_cost, max_cost = min(costs), max(costs)
        min_goal, max_goal = min(goals), max(goals)
        min_info, max_info = min(infos), max(infos)
        
        final_scored = []
        for idx, fx, fy, path_cost, d_goal, info_gain, angle_diff, visit_pen, path in scored:
            # Normalize
            nc = (path_cost - min_cost) / (max_cost - min_cost + EPS)
            ng = (d_goal - min_goal) / (max_goal - min_goal + EPS)
            ni = (info_gain - min_info) / (max_info - min_info + EPS)
            nang = abs(angle_diff) / np.pi
            
            # Compute score (lower is better)
            score = (self.W_PATH * nc + 
                    self.W_GOAL * ng + 
                    self.W_INFO * (1.0 - ni) +  # Higher info gain = lower score
                    self.W_ALIGN * nang + 
                    self.W_VIS * visit_pen)
            
            final_scored.append((score, idx, fx, fy, path_cost, d_goal, info_gain, angle_diff, visit_pen))
        
        # Sort by score (ascending - lower is better)
        final_scored.sort(key=lambda x: x[0])
        
        print(f"✅ {len(final_scored)} frontiers scored")
        print("\nTop 5 Candidates:")
        for i, (score, idx, fx, fy, pc, dg, ig, ang, vp) in enumerate(final_scored[:5], 1):
            marker = "👉" if i == 1 else "  "
            print(f"{marker} {i}. ({fx:.2f}, {fy:.2f}) - Score: {score:.3f}")
            print(f"      Path cost: {pc:.2f}m | Goal dist: {dg:.2f}m")
            print(f"      Info gain: {ig:.0f} cells | Angle: {np.degrees(ang):.1f}°")
            print(f"      Visit penalty: {vp:.2f}")
        
        # Select best
        best_score, best_idx, best_fx, best_fy = final_scored[0][:4]
        
        # Update visit log and record visit
        self.visit_log[best_idx] = self.visit_log.get(best_idx, 0) + 1
        self._record_frontier_visit(best_fx, best_fy)
        
        print("-" * 70)
        print(f"✅ SELECTED: ({best_fx:.2f}, {best_fy:.2f}) with score {best_score:.3f}")
        print(f"   Visit count: {self.visit_log[best_idx]}")
        print("-" * 70)
        
        return (best_fx, best_fy)
    
    def _is_goal_reached(self) -> bool:
        """Check if current intermediate goal is reached."""
        if self.current_goal is None:
            return True
        
        return self.path_controller.is_goal_reached(
            self.robot.x, self.robot.y,
            self.current_goal[0], self.current_goal[1],
            tolerance=0.8  # Increased from 0.3 to avoid jittering
        )
    
    def _is_at_final_goal(self) -> bool:
        """Check if robot reached the final goal."""
        dist = np.sqrt((self.robot.x - self.goal_pos[0])**2 + 
                      (self.robot.y - self.goal_pos[1])**2)
        return dist < 0.4
    
    def _update_visualization(self):
        """Update both 2D and 3D visualizations."""
        # Clear axes
        self.ax_2d.clear()
        self.ax_3d.clear()
        
        # === 2D MAP ===
        self.ax_2d.set_xlim(0, self.map_size[0])
        self.ax_2d.set_ylim(0, self.map_size[1])
        self.ax_2d.set_xlabel('X (m)', fontsize=12)
        self.ax_2d.set_ylabel('Y (m)', fontsize=12)
        self.ax_2d.set_title('2D Exploration Map', fontsize=14, fontweight='bold')
        self.ax_2d.grid(True, alpha=0.3)
        self.ax_2d.set_aspect('equal')
        
        # Draw occupancy grid
        grid_display = self.occupancy_grid.grid.copy()
        
        # Color map: Unknown=gray, Free=white, Occupied=black, Explored=light blue
        colored_grid = np.zeros((*grid_display.shape, 3))
        
        # Unknown areas (gray)
        unknown_mask = (grid_display >= 0.35) & (grid_display <= 0.65)
        colored_grid[unknown_mask] = [0.7, 0.7, 0.7]
        
        # Free areas (light blue - explored)
        free_mask = grid_display < 0.35
        colored_grid[free_mask] = [0.8, 0.9, 1.0]
        
        # Occupied areas (dark red)
        occupied_mask = grid_display > 0.65
        colored_grid[occupied_mask] = [0.3, 0.1, 0.1]
        
        self.ax_2d.imshow(colored_grid, origin='lower', 
                         extent=[0, self.map_size[0], 0, self.map_size[1]],
                         alpha=0.8)
        
        # Draw obstacles (ground truth)
        for obj in self.obstacles:
            if obj.__class__.__name__ == "Box":
                rect = plt.Rectangle((obj.x - obj.width/2, obj.y - obj.length/2),
                                    obj.width, obj.length, 
                                    fill=False, edgecolor='red', linewidth=1.5, 
                                    linestyle='--', alpha=0.5)
                self.ax_2d.add_patch(rect)
            elif obj.__class__.__name__ == "Cylinder":
                circle = plt.Circle((obj.x, obj.y), obj.radius, 
                                   fill=False, edgecolor='red', linewidth=1.5,
                                   linestyle='--', alpha=0.5)
                self.ax_2d.add_patch(circle)
        
        # Draw robot trajectory
        if len(self.visited_positions) > 1:
            traj = np.array(self.visited_positions)
            self.ax_2d.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2, 
                           alpha=0.6, label='Robot Path')
        
        # Draw current path
        if self.current_path and len(self.current_path) > 1:
            path_array = np.array(self.current_path)
            self.ax_2d.plot(path_array[:, 0], path_array[:, 1], 'g--', 
                           linewidth=2, alpha=0.7, label='Planned Path')
        
        # Draw visited frontiers (history trail)
        if self.frontiers_visited:
            visited_array = np.array(self.frontiers_visited)
            self.ax_2d.scatter(visited_array[:, 0], visited_array[:, 1], 
                              c='lightgreen', s=200, marker='o', 
                              edgecolors='darkgreen', linewidths=2,
                              label=f'Visited Frontiers ({len(self.frontiers_visited)})', 
                              zorder=6, alpha=0.7)
            
            # Number the visited frontiers
            for i, (fx, fy) in enumerate(self.frontiers_visited, 1):
                self.ax_2d.text(fx, fy, f'{i}', 
                               fontsize=10, color='white', fontweight='bold',
                               ha='center', va='center',
                               bbox=dict(boxstyle='circle,pad=0.3', 
                                       facecolor='darkgreen', alpha=0.9))
        
        # Highlight CURRENT FRONTIER target (only the one being pursued)
        if self.current_goal and self.current_goal != self.goal_pos:
            self.ax_2d.scatter(self.current_goal[0], self.current_goal[1], 
                              c='yellow', s=500, marker='*', 
                              edgecolors='red', linewidths=4,
                              label='Current Target Frontier', zorder=20)
            self.ax_2d.text(self.current_goal[0], self.current_goal[1] - 0.7, 
                           f'TARGET\nF{self.frontier_count + 1}',
                           ha='center', fontsize=10, fontweight='bold', 
                           color='red',
                           bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', 
                                   edgecolor='red', linewidth=3, alpha=0.95))
        
        # Draw START position
        self.ax_2d.scatter(self.start_pos[0], self.start_pos[1], 
                          c='green', s=300, marker='o', 
                          edgecolors='black', linewidths=2,
                          label='START', zorder=10)
        self.ax_2d.text(self.start_pos[0], self.start_pos[1] + 0.5, 'START',
                       ha='center', fontsize=10, fontweight='bold', color='green')
        
        # Draw GOAL position
        self.ax_2d.scatter(self.goal_pos[0], self.goal_pos[1], 
                          c='red', s=300, marker='X', 
                          edgecolors='black', linewidths=2,
                          label='GOAL', zorder=10)
        self.ax_2d.text(self.goal_pos[0], self.goal_pos[1] + 0.5, 'GOAL',
                       ha='center', fontsize=10, fontweight='bold', color='red')
        
        # Draw current robot position
        self.ax_2d.scatter(self.robot.x, self.robot.y, 
                          c='blue', s=200, marker='o',
                          edgecolors='yellow', linewidths=2,
                          label='Robot', zorder=15)
        
        # Draw robot heading
        arrow_length = 0.5
        dx = arrow_length * np.cos(self.robot.theta)
        dy = arrow_length * np.sin(self.robot.theta)
        self.ax_2d.arrow(self.robot.x, self.robot.y, dx, dy,
                        head_width=0.3, head_length=0.2, fc='blue', ec='blue')
        
        self.ax_2d.legend(loc='upper right', fontsize=9)
        
        # === 3D ENVIRONMENT ===
        self.ax_3d.set_xlim(0, self.map_size[0])
        self.ax_3d.set_ylim(0, self.map_size[1])
        self.ax_3d.set_zlim(0, 2)
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        
        # Draw obstacles in 3D
        for obj in self.obstacles:
            cls_name = obj.__class__.__name__
            if cls_name == "Box":
                self._draw_box_3d(obj)
            elif cls_name == "Cylinder":
                self._draw_cylinder_3d(obj)
            elif cls_name == "Cone":
                self._draw_cone_3d(obj)
            elif cls_name == "Hemisphere":
                self._draw_hemisphere_3d(obj)
        
        # Draw robot in 3D
        self._draw_robot_3d()
        
        # Draw trajectory in 3D
        if len(self.visited_positions) > 1:
            traj = np.array(self.visited_positions)
            self.ax_3d.plot(traj[:, 0], traj[:, 1], np.zeros(len(traj)) + 0.1,
                           'b-', linewidth=2, alpha=0.6)
        
        # Draw start and goal in 3D
        self.ax_3d.scatter([self.start_pos[0]], [self.start_pos[1]], [0.5],
                          c='green', s=200, marker='o', edgecolors='black', linewidths=2)
        self.ax_3d.scatter([self.goal_pos[0]], [self.goal_pos[1]], [0.5],
                          c='red', s=200, marker='X', edgecolors='black', linewidths=2)
        
        plt.draw()
    
    def _draw_box_3d(self, obj):
        """Draw box in 3D."""
        x = [obj.x - obj.width/2, obj.x + obj.width/2]
        y = [obj.y - obj.length/2, obj.y + obj.length/2]
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)
        self.ax_3d.plot_surface(X, Y, Z, color=obj.color or "gray", alpha=0.7)
        self.ax_3d.plot_surface(X, Y, Z + obj.height, color=obj.color or "gray", alpha=0.7)
    
    def _draw_cylinder_3d(self, obj):
        """Draw cylinder in 3D."""
        theta = np.linspace(0, 2 * np.pi, 30)
        z = np.linspace(0, obj.height, 10)
        theta, z = np.meshgrid(theta, z)
        X = obj.x + obj.radius * np.cos(theta)
        Y = obj.y + obj.radius * np.sin(theta)
        Z = z
        self.ax_3d.plot_surface(X, Y, Z, color=obj.color or "gray", alpha=0.7)
    
    def _draw_cone_3d(self, obj):
        """Draw cone in 3D."""
        theta = np.linspace(0, 2 * np.pi, 30)
        r = np.linspace(0, obj.radius, 30)
        R, T = np.meshgrid(r, theta)
        X = obj.x + R * np.cos(T)
        Y = obj.y + R * np.sin(T)
        Z = obj.height - (obj.height / obj.radius) * R
        self.ax_3d.plot_surface(X, Y, Z, color=obj.color or "gray", alpha=0.7)
    
    def _draw_hemisphere_3d(self, obj):
        """Draw hemisphere in 3D."""
        u = np.linspace(0, np.pi / 2, 30)
        v = np.linspace(0, 2 * np.pi, 30)
        U, V = np.meshgrid(u, v)
        X = obj.x + obj.radius * np.sin(U) * np.cos(V)
        Y = obj.y + obj.radius * np.sin(U) * np.sin(V)
        Z = obj.radius * np.cos(U)
        self.ax_3d.plot_surface(X, Y, Z, color=obj.color or "gray", alpha=0.7)
    
    def _draw_robot_3d(self):
        """Draw robot in 3D."""
        # Simple representation
        self.ax_3d.scatter([self.robot.x], [self.robot.y], [0.2],
                          c='blue', s=100, marker='o')
    
    def _print_status(self, iteration: int):
        """Print current status."""
        total_cells = self.occupancy_grid.width * self.occupancy_grid.height
        known_cells = np.sum(
            (self.occupancy_grid.grid < 0.35) | (self.occupancy_grid.grid > 0.65)
        )
        exploration_ratio = known_cells / total_cells
        
        dist_to_goal = np.sqrt((self.robot.x - self.goal_pos[0])**2 + 
                              (self.robot.y - self.goal_pos[1])**2)
        
        print(f"\n[Status - Iteration {iteration}]")
        print(f"  Position: ({self.robot.x:.2f}, {self.robot.y:.2f})")
        print(f"  Distance to goal: {dist_to_goal:.2f}m")
        print(f"  Exploration: {exploration_ratio*100:.1f}%")
        print(f"  Distance traveled: {self.total_distance:.2f}m")
    
    def _print_final_stats(self, iterations: int):
        """Print final statistics."""
        elapsed_time = time.time() - self.start_time
        total_cells = self.occupancy_grid.width * self.occupancy_grid.height
        known_cells = np.sum(
            (self.occupancy_grid.grid < 0.35) | (self.occupancy_grid.grid > 0.65)
        )
        exploration_ratio = known_cells / total_cells
        
        print("\n" + "="*70)
        print("EXPLORATION SUMMARY")
        print("="*70)
        print(f"Goal Reached: {'YES ✓' if self.goal_reached else 'NO ✗'}")
        print(f"Total iterations: {iterations}")
        print(f"Frontiers visited: {self.frontier_count}")
        print(f"Total distance: {self.total_distance:.2f}m")
        print(f"Total time: {elapsed_time:.1f}s")
        print(f"Average speed: {self.total_distance/elapsed_time:.2f}m/s")
        print(f"Map explored: {exploration_ratio*100:.1f}%")
        print(f"Final position: ({self.robot.x:.2f}, {self.robot.y:.2f})")
        
        if self.frontiers_visited:
            print(f"\n📍 FRONTIER EXPLORATION PATH:")
            print("-" * 70)
            for i, (fx, fy) in enumerate(self.frontiers_visited, 1):
                print(f"  {i}. Frontier at ({fx:.2f}, {fy:.2f})")
        
        print("="*70)


def main():
    """Main entry point."""
    # Define start and goal positions (closer with obstacles in between)
    START = (4.0, 6.0)
    GOAL = (10.0, 10.0)  # Closer goal to see more exploration behavior
    
    explorer = GoalDirectedExplorer(
        start_pos=START,
        goal_pos=GOAL,
        map_size=(20, 20),
        resolution=0.1
    )
    
    explorer.run_exploration(max_iterations=3000)


if __name__ == "__main__":
    main()




# ================================================================
# ==== FILE: src/sensors/lidar.py ====
# ================================================================

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




# ================================================================
# ==== FILE: src/planning/__init__.py ====
# ================================================================

"""Path planning modules."""




# ================================================================
# ==== FILE: src/planning/frontier_explorer.py ====
# ================================================================

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




# ================================================================
# ==== FILE: src/planning/astar.py ====
# ================================================================

"""
A* Path Planner for grid-based navigation.
Finds optimal collision-free paths considering robot constraints.
"""
import numpy as np
import heapq
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

@dataclass(order=True)
class Node:
    """Node for A* search."""
    f: float
    g: float = field(compare=False)
    pos: Tuple[int, int] = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)

class AStarPlanner:
    """A* path planner for occupancy grids."""
    
    def __init__(self, occupancy_grid):
        """
        Initialize A* planner.
        
        Args:
            occupancy_grid: OccupancyGrid object
        """
        self.grid = occupancy_grid
        
        # 8-connected grid movements (dx, dy, cost)
        self.motions = [
            (1, 0, 1.0),      # Right
            (0, 1, 1.0),      # Up
            (-1, 0, 1.0),     # Left
            (0, -1, 1.0),     # Down
            (1, 1, 1.414),    # Diagonal
            (-1, 1, 1.414),
            (1, -1, 1.414),
            (-1, -1, 1.414)
        ]
    
    def plan(self, start_x: float, start_y: float, 
             goal_x: float, goal_y: float,
             inflation_radius: int = 2) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path from start to goal.
        
        Args:
            start_x, start_y: Start position in world coordinates
            goal_x, goal_y: Goal position in world coordinates
            inflation_radius: Safety margin around obstacles (in grid cells)
            
        Returns:
            List of (x, y) waypoints in world coordinates, or None if no path found
        """
        # Convert to grid coordinates
        start_gx, start_gy = self.grid.world_to_grid(start_x, start_y)
        goal_gx, goal_gy = self.grid.world_to_grid(goal_x, goal_y)
        
        # Validate start and goal
        if not self.grid.is_valid(start_gx, start_gy):
            print(f"Start position ({start_x}, {start_y}) is out of bounds")
            return None
        
        if not self.grid.is_valid(goal_gx, goal_gy):
            print(f"Goal position ({goal_x}, {goal_y}) is out of bounds")
            return None
        
        if self.grid.is_occupied(goal_gx, goal_gy):
            print(f"Goal position is occupied")
            return None
        
        # Inflate obstacles for safety
        inflated_grid = self._inflate_obstacles(inflation_radius)
        
        # A* search
        open_set = []
        closed_set = set()
        
        start_node = Node(
            f=self._heuristic(start_gx, start_gy, goal_gx, goal_gy),
            g=0,
            pos=(start_gx, start_gy),
            parent=None
        )
        
        heapq.heappush(open_set, start_node)
        came_from = {}
        g_score = {(start_gx, start_gy): 0}
        
        while open_set:
            current = heapq.heappop(open_set)
            
            if current.pos in closed_set:
                continue
            
            # Goal reached
            if current.pos == (goal_gx, goal_gy):
                return self._reconstruct_path(current)
            
            closed_set.add(current.pos)
            
            # Explore neighbors
            for dx, dy, cost in self.motions:
                nx = current.pos[0] + dx
                ny = current.pos[1] + dy
                neighbor_pos = (nx, ny)
                
                # Check validity
                if not self.grid.is_valid(nx, ny):
                    continue
                
                if neighbor_pos in closed_set:
                    continue
                
                # Check if occupied (using inflated grid)
                if inflated_grid[ny, nx]:
                    continue
                
                # Calculate tentative g score
                tentative_g = current.g + cost
                
                # Check if this path is better
                if neighbor_pos not in g_score or tentative_g < g_score[neighbor_pos]:
                    g_score[neighbor_pos] = tentative_g
                    h = self._heuristic(nx, ny, goal_gx, goal_gy)
                    f = tentative_g + h
                    
                    neighbor_node = Node(
                        f=f,
                        g=tentative_g,
                        pos=neighbor_pos,
                        parent=current
                    )
                    
                    heapq.heappush(open_set, neighbor_node)
        
        print("No path found!")
        return None
    
    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """Euclidean distance heuristic."""
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def _reconstruct_path(self, node: Node) -> List[Tuple[float, float]]:
        """Reconstruct path from goal node to start."""
        path = []
        current = node
        
        while current is not None:
            # Convert grid coordinates to world coordinates
            x, y = self.grid.grid_to_world(current.pos[0], current.pos[1])
            path.append((x, y))
            current = current.parent
        
        path.reverse()
        
        # Smooth the path
        path = self._smooth_path(path)
        
        return path
    
    def _smooth_path(self, path: List[Tuple[float, float]], 
                     max_iterations: int = 100) -> List[Tuple[float, float]]:
        """Smooth path by removing unnecessary waypoints."""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Try to connect to furthest visible point
            for j in range(len(path) - 1, i, -1):
                if self._is_line_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
        
        return smoothed
    
    def _is_line_free(self, p1: Tuple[float, float], 
                      p2: Tuple[float, float]) -> bool:
        """Check if line between two points is collision-free."""
        x1, y1 = self.grid.world_to_grid(p1[0], p1[1])
        x2, y2 = self.grid.world_to_grid(p2[0], p2[1])
        
        # Bresenham's line algorithm
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        x, y = x1, y1
        
        while True:
            if self.grid.is_occupied(x, y):
                return False
            
            if x == x2 and y == y2:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True
    
    def _inflate_obstacles(self, radius: int) -> np.ndarray:
        """Inflate obstacles for safety margin."""
        from scipy.ndimage import binary_dilation
        
        # Create binary obstacle map
        obstacle_map = self.grid.grid > 0.65
        
        # Inflate
        if radius > 0:
            structure = np.ones((2*radius+1, 2*radius+1))
            inflated = binary_dilation(obstacle_map, structure=structure)
        else:
            inflated = obstacle_map
        
        return inflated




# ================================================================
# ==== FILE: src/objects/primitives.py ====
# ================================================================

"""
Primitive 3D shapes for the simulation environment.
Supports: Box, Cylinder, Cone, Hemisphere.
Each primitive provides geometric vertices for rendering
and a `contains(x, y, z)` method for spatial queries.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Optional


# ==========================================================
# Base Class
# ==========================================================
@dataclass
class Primitive:
    """Base class for all primitive shapes."""
    x: float                  # x-coordinate of the center
    y: float                  # y-coordinate of the center
    height: float             # height of the shape along z-axis
    color: Optional[Tuple[float, float, float]] = None  # RGB color (0-1)

    def get_vertices(self) -> List[Tuple[float, float, float]]:
        """Return vertices for rendering."""
        raise NotImplementedError("Subclasses must implement get_vertices()")

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if a point (x, y, z) is inside the shape."""
        raise NotImplementedError("Subclasses must implement contains()")


# ==========================================================
# Box (Rectangular Prism)
# ==========================================================
@dataclass
class Box(Primitive):
    width: float = 1.0     # size along x-axis
    length: float = 1.0    # size along y-axis
    rotation: float = 0.0  # rotation in radians around z-axis

    def get_vertices(self) -> List[Tuple[float, float, float]]:
        """Get the 8 vertices of the box for rendering."""
        hw, hl = self.width / 2, self.length / 2

        # Local vertices (bottom + top faces)
        local_vertices = [
            (-hw, -hl, 0), (hw, -hl, 0), (hw, hl, 0), (-hw, hl, 0),
            (-hw, -hl, self.height), (hw, -hl, self.height),
            (hw, hl, self.height), (-hw, hl, self.height)
        ]

        # Apply rotation + translation
        cos_t, sin_t = np.cos(self.rotation), np.sin(self.rotation)
        world_vertices = []
        for lx, ly, lz in local_vertices:
            wx = self.x + (lx * cos_t - ly * sin_t)
            wy = self.y + (lx * sin_t + ly * cos_t)
            world_vertices.append((wx, wy, lz))
        return world_vertices

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point (x,y,z) lies inside the box."""
        if not (0 <= z <= self.height):
            return False

        # Transform to local coordinates
        dx, dy = x - self.x, y - self.y
        cos_t, sin_t = np.cos(-self.rotation), np.sin(-self.rotation)
        local_x = dx * cos_t - dy * sin_t
        local_y = dx * sin_t + dy * cos_t

        return (abs(local_x) <= self.width / 2) and (abs(local_y) <= self.length / 2)


# ==========================================================
# Cylinder
# ==========================================================
@dataclass
class Cylinder(Primitive):
    radius: float = 0.5

    def get_vertices(self, segments: int = 24) -> List[Tuple[float, float, float]]:
        """Approximate the cylinder surface using circular segments."""
        verts = []
        angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
        for theta in angles:
            cx = self.x + self.radius * np.cos(theta)
            cy = self.y + self.radius * np.sin(theta)
            verts.append((cx, cy, 0))
            verts.append((cx, cy, self.height))
        return verts

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point is inside the cylinder."""
        if not (0 <= z <= self.height):
            return False
        dx, dy = x - self.x, y - self.y
        return (dx**2 + dy**2) <= (self.radius**2)


# ==========================================================
# Cone
# ==========================================================
@dataclass
class Cone(Primitive):
    radius: float = 0.5

    def get_vertices(self, segments: int = 24) -> List[Tuple[float, float, float]]:
        """Generate vertices for cone surface."""
        verts = []
        apex = (self.x, self.y, self.height)
        verts.append(apex)

        # Base circle
        angles = np.linspace(0, 2 * np.pi, segments + 1)
        for theta in angles:
            bx = self.x + self.radius * np.cos(theta)
            by = self.y + self.radius * np.sin(theta)
            verts.append((bx, by, 0))
        return verts

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point is inside the cone volume."""
        if not (0 <= z <= self.height):
            return False
        dx, dy = x - self.x, y - self.y
        r_at_z = self.radius * (1 - z / self.height)
        return (dx**2 + dy**2) <= r_at_z**2


# ==========================================================
# Hemisphere
# ==========================================================
@dataclass
class Hemisphere(Primitive):
    radius: float = 0.5

    def get_vertices(self, rings: int = 10, segments: int = 24) -> List[Tuple[float, float, float]]:
        """Generate hemisphere surface points."""
        verts = []
        for j in range(rings + 1):
            phi = (np.pi / 2) * (j / rings)   # 0 → π/2
            r = self.radius * np.sin(phi)
            z = self.radius * np.cos(phi)
            for i in range(segments):
                theta = 2 * np.pi * i / segments
                x = self.x + r * np.cos(theta)
                y = self.y + r * np.sin(theta)
                verts.append((x, y, z))
        return verts

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point lies within the hemisphere."""
        if not (0 <= z <= self.radius):
            return False
        dx, dy, dz = x - self.x, y - self.y, z
        return (dx**2 + dy**2 + dz**2) <= (self.radius**2)




# ================================================================
# ==== FILE: src/objects/generator.py ====
# ================================================================

"""
Environment generator for creating random 3D environments with various obstacles.
"""
import random
import numpy as np
from typing import List, Tuple
from .primitives import Box, Cylinder, Cone, Hemisphere

def generate_environment(num_obs: int = 15, bounds: Tuple[float, float] = (20, 20),
                       start_pos: Tuple[float, float] = (4, 6),
                       goal_pos: Tuple[float, float] = (16, 16)) -> List:
    """
    Generate a challenging environment with no direct path from start to goal.
    """
    obstacles = []
    
    # Create a maze-like structure that forces exploration
    mid_x = (start_pos[0] + goal_pos[0]) / 2
    mid_y = (start_pos[1] + goal_pos[1]) / 2
    
    # Create a more complex obstacle course
    # Main walls to force navigation
    walls = [
        # Horizontal wall with gap
        Box(x=mid_x - 4, y=mid_y, width=6, length=0.5, height=2.0, rotation=0.0, color=(0.6, 0.2, 0.2)),
        Box(x=mid_x + 4, y=mid_y, width=6, length=0.5, height=2.0, rotation=0.0, color=(0.6, 0.2, 0.2)),
        
        # Vertical walls to create path
        Box(x=mid_x, y=mid_y + 3, width=0.5, length=4.0, height=2.0, rotation=0.0, color=(0.2, 0.6, 0.2)),
        Box(x=mid_x + 3, y=mid_y - 3, width=0.5, length=4.0, height=2.0, rotation=0.0, color=(0.2, 0.6, 0.2)),
        Box(x=mid_x - 3, y=mid_y + 6, width=0.5, length=4.0, height=2.0, rotation=0.0, color=(0.2, 0.6, 0.2)),
        
        # Block direct path
        Box(x=mid_x, y=mid_y, width=2.0, length=2.0, height=2.0, rotation=0.0, color=(0.8, 0.2, 0.2)),
    ]
    
    #Add some random obstacles
    for _ in range(num_obs):
        while True:
            x = np.random.uniform(2, bounds[0] - 2)
            y = np.random.uniform(2, bounds[1] - 2)
            
            # Don't place obstacles too close to start or goal
            if (np.hypot(x - start_pos[0], y - start_pos[1]) < 3.0 or
                np.hypot(x - goal_pos[0], y - goal_pos[1]) < 3.0):
                continue
                
            # Don't block the main path
            if (mid_x - 6 < x < mid_x + 6 and mid_y - 6 < y < mid_y + 6):
                continue
                
            shape = np.random.choice(['box', 'cylinder', 'cone'])
            height = np.random.uniform(1.0, 2.5)
            color = (np.random.uniform(0.2, 0.8),
                    np.random.uniform(0.2, 0.8),
                    np.random.uniform(0.2, 0.8))
            
            if shape == 'box':
                width = np.random.uniform(0.8, 2.5)
                length = np.random.uniform(0.8, 2.5)
                rotation = np.random.uniform(0, np.pi)
                obstacles.append(Box(x, y, height, width, length, rotation, color=color))
            elif shape == 'cylinder':
                radius = np.random.uniform(0.5, 1.5)
                obstacles.append(Cylinder(x, y, height, radius, color=color))
            else:  # cone
                radius = np.random.uniform(0.5, 1.5)
                obstacles.append(Cone(x, y, height, radius, color=color))
            break
    
    return walls + obstacles
 



# ================================================================
# ==== FILE: src/robot/ackermann.py ====
# ================================================================

class AckermannRobot:
    def __init__(self, x, y, theta, wheelbase=0.3, dt=0.1):
        self.x = x
        self.y = y
        self.theta = theta
        self.wheelbase = wheelbase
        self.dt = dt

        # Visual dimensions (in meters)
        self.body_length = 2.4
        self.body_width = 1.5
        self.wheel_radius = 0.3
        self.wheel_width = 0.12
        self.delta = 0.0  # steering angle
        self.v = 0.0  # current velocity

    def step(self, v, delta):
        """Discrete update using bicycle kinematics."""
        self.v = v  # store velocity
        self.delta = delta  # store steering for visualization
        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += (v / self.wheelbase) * np.tan(delta) * self.dt



# ================================================================
# ==== FILE: src/env/demo_env.py ====
# ================================================================

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




# ================================================================
# ==== FILE: src/env/env_3d.py ====
# ================================================================

"""
3D Renderer for 2-wheel Ackermann visualization + telemetry graphs.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Renderer3D:
    def __init__(self, obstacles, map_size=(20, 20)):
        self.obstacles = obstacles
        self.map_size = map_size

        # --- 3D Figure ---
        plt.ion()
        self.fig = plt.figure(figsize=(10, 5))
        self.ax3d = self.fig.add_subplot(121, projection="3d")
        self.ax3d.set_xlim(0, map_size[0])
        self.ax3d.set_ylim(0, map_size[1])
        self.ax3d.set_zlim(0, 2.5)
        self.ax3d.set_xlabel("X (m)", fontsize=10, fontweight='bold')
        self.ax3d.set_ylabel("Y (m)", fontsize=10, fontweight='bold')
        self.ax3d.set_zlabel("Z (m)", fontsize=10, fontweight='bold')
        self.ax3d.set_title('3D Environment - Ackermann Robot', fontsize=12, fontweight='bold')
        
        # Better viewing angle
        self.ax3d.view_init(elev=25, azim=45)
        
        # Grid and background
        self.ax3d.grid(True, alpha=0.3)
        self.ax3d.set_facecolor((0.95, 0.95, 0.95))

        # --- Telemetry Figure ---
        self.fig_telemetry, (self.ax_wheels, self.ax_ctrl) = plt.subplots(2, 1, figsize=(6, 6))
        self.time_data = []
        self.left_wheel = []
        self.right_wheel = []
        self.v_data = []
        self.delta_data = []

        # Ground plane mesh
        Xg, Yg = np.meshgrid(np.linspace(0, map_size[0], 10),
                             np.linspace(0, map_size[1], 10))
        self.Zg = np.zeros_like(Xg)
        self.Xg, self.Yg = Xg, Yg

    # ------------------------------------------------------------------
    def render(self, robot, t):
        """Render robot + update telemetry plots."""
        self.ax3d.clear()
        self.ax3d.set_xlim(0, self.map_size[0])
        self.ax3d.set_ylim(0, self.map_size[1])
        self.ax3d.set_zlim(0, 2.5)
        self.ax3d.view_init(elev=25, azim=45)
        self.ax3d.grid(True, alpha=0.3)
        self.ax3d.plot_surface(self.Xg, self.Yg, self.Zg, color="lightgray", alpha=0.3)

        # Draw obstacles
        for obj in self.obstacles:
            if obj.__class__.__name__ == "Box":
                self._draw_box(obj)
            elif obj.__class__.__name__ == "Cylinder":
                self._draw_cylinder(obj)

        # Draw the simplified 2-wheel robot
        self._draw_two_wheel_robot(robot)

        # --- Telemetry update ---
        self.time_data.append(t)
        wl, wr = robot.get_wheel_speeds()
        self.left_wheel.append(wl)
        self.right_wheel.append(wr)
        self.v_data.append(robot.v)
        self.delta_data.append(robot.delta)

        self._update_telemetry()
        plt.pause(0.001)

    # ------------------------------------------------------------------
    def _draw_robot(self, robot):
        """Draw the robot as a 4-wheel Ackermann vehicle with turning front wheels."""
        body_length = robot.body_length
        body_width = robot.body_width
        wheel_radius = robot.wheel_radius
        wheel_width = robot.wheel_width
        
        x, y, theta = robot.x, robot.y, robot.theta
        steering_angle = robot.delta  # Current steering angle
        
        # === DRAW BODY ===
        # Body corners
        corners_x = [-body_length/2, body_length/2, body_length/2, -body_length/2, -body_length/2]
        corners_y = [-body_width/2, -body_width/2, body_width/2, body_width/2, -body_width/2]
        
        # Rotate and translate
        rotated_x = []
        rotated_y = []
        for cx, cy in zip(corners_x, corners_y):
            rx = x + cx * np.cos(theta) - cy * np.sin(theta)
            ry = y + cx * np.sin(theta) + cy * np.cos(theta)
            rotated_x.append(rx)
            rotated_y.append(ry)
        
        # Draw body (blue box)
        z_bottom = [0.1] * 5
        z_top = [0.4] * 5
        self.ax3d.plot(rotated_x, rotated_y, z_bottom, 'b-', linewidth=3, alpha=0.8)
        self.ax3d.plot(rotated_x, rotated_y, z_top, 'b-', linewidth=3, alpha=0.8)
        
        # Connect bottom to top
        for i in range(4):
            self.ax3d.plot([rotated_x[i], rotated_x[i]], 
                        [rotated_y[i], rotated_y[i]], 
                        [z_bottom[i], z_top[i]], 'b-', linewidth=2, alpha=0.6)
        
        # === DRAW 4 WHEELS ===
        # Wheel positions in robot frame
        wheelbase = robot.wheelbase
        rear_axle_offset = -body_length/3
        front_axle_offset = rear_axle_offset + wheelbase
        
        wheel_positions = [
            (rear_axle_offset, -body_width/2, 0),      # Rear left (no steering)
            (rear_axle_offset, body_width/2, 0),       # Rear right (no steering)
            (front_axle_offset, -body_width/2, steering_angle),  # Front left (steered)
            (front_axle_offset, body_width/2, steering_angle),   # Front right (steered)
        ]
        
        for i, (wx, wy, wheel_angle) in enumerate(wheel_positions):
            # Rotate wheel position to world frame
            rwx = x + wx * np.cos(theta) - wy * np.sin(theta)
            rwy = y + wx * np.sin(theta) + wy * np.cos(theta)
            
            # Total wheel orientation (robot heading + steering)
            total_angle = theta + wheel_angle
            
            # Draw wheel as a small box
            wheel_half_length = wheel_radius
            wheel_half_width = wheel_width / 2
            
            # Wheel corners in wheel frame
            w_corners_x = [-wheel_half_length, wheel_half_length, wheel_half_length, -wheel_half_length, -wheel_half_length]
            w_corners_y = [-wheel_half_width, -wheel_half_width, wheel_half_width, wheel_half_width, -wheel_half_width]
            
            # Rotate and translate wheel corners
            w_rotated_x = []
            w_rotated_y = []
            for wcx, wcy in zip(w_corners_x, w_corners_y):
                wrx = rwx + wcx * np.cos(total_angle) - wcy * np.sin(total_angle)
                wry = rwy + wcx * np.sin(total_angle) + wcy * np.cos(total_angle)
                w_rotated_x.append(wrx)
                w_rotated_y.append(wry)
            
            # Draw wheel (black for rear, red for front to show steering)
            wheel_color = 'red' if i >= 2 else 'black'
            wheel_z = [0.05] * 5
            self.ax3d.plot(w_rotated_x, w_rotated_y, wheel_z, 
                        color=wheel_color, linewidth=4, alpha=0.9)
            
            # Draw wheel top
            wheel_z_top = [0.15] * 5
            self.ax3d.plot(w_rotated_x, w_rotated_y, wheel_z_top, 
                        color=wheel_color, linewidth=3, alpha=0.7)
        
        # === DRAW HEADING INDICATOR ===
        arrow_length = 0.6
        arrow_x = x + arrow_length * np.cos(theta)
        arrow_y = y + arrow_length * np.sin(theta)
        self.ax3d.plot([x, arrow_x], [y, arrow_y], [0.4, 0.4], 
                    'yellow', linewidth=3, alpha=0.9)

    # ------------------------------------------------------------------
    def _update_telemetry(self):
        """Live-update telemetry subplots."""
        self.ax_wheels.clear()
        self.ax_wheels.plot(self.time_data, self.left_wheel, label="Left Wheel ω")
        self.ax_wheels.plot(self.time_data, self.right_wheel, label="Right Wheel ω")
        self.ax_wheels.set_ylabel("Angular Velocity (rad/s)")
        self.ax_wheels.legend()
        self.ax_wheels.grid(True)

        self.ax_ctrl.clear()
        self.ax_ctrl.plot(self.time_data, self.v_data, label="v (m/s)")
        self.ax_ctrl.plot(self.time_data, self.delta_data, label="δ (rad)")
        self.ax_ctrl.set_xlabel("Time (s)")
        self.ax_ctrl.legend()
        self.ax_ctrl.grid(True)

    # ------------------------------------------------------------------
    def _draw_box(self, obj):
        X = [obj.x - obj.width/2, obj.x + obj.width/2]
        Y = [obj.y - obj.length/2, obj.y + obj.length/2]
        X, Y = np.meshgrid(X, Y)
        Z = np.zeros_like(X)
        self.ax3d.plot_surface(X, Y, Z + obj.height, color=obj.color or "gray", alpha=0.5)

    def _draw_cylinder(self, obj):
        theta = np.linspace(0, 2*np.pi, 30)
        z = np.linspace(0, obj.height, 5)
        theta, z = np.meshgrid(theta, z)
        X = obj.x + obj.radius*np.cos(theta)
        Y = obj.y + obj.radius*np.sin(theta)
        self.ax3d.plot_surface(X, Y, z, color=obj.color or "gray", alpha=0.6)

    def close(self):
        plt.close(self.fig)
        plt.close(self.fig_telemetry)




# ================================================================
# ==== FILE: src/env/env_2d_map.py ====
# ================================================================

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




# ================================================================
# ==== FILE: src/env/environment.py ====
# ================================================================

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




# ================================================================
# ==== FILE: src/env/env_2d.py ====
# ================================================================

# In env_2d.py
import numpy as np
from matplotlib.patches import Rectangle, Circle, Polygon, FancyArrow

class Renderer2D:
    def __init__(self, obstacles, map_size):
        self.obstacles = obstacles
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(0, map_size[0])
        self.ax.set_ylim(0, map_size[1])
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('2D Ackermann Robot Visualization')

    def draw_robot(self, robot):
        """Draw the 4-wheel Ackermann robot with proper steering"""
        # Robot body
        body = Rectangle((robot.x - robot.body_length/2, robot.y - robot.body_width/2),
                        robot.body_length, robot.body_width,
                        angle=np.degrees(robot.theta), 
                        color='skyblue', alpha=0.8)
        self.ax.add_patch(body)
        
        # Wheels
        wheel_positions = [
            (-robot.body_length/3, -robot.body_width/2, 0),  # Rear left
            (-robot.body_length/3, robot.body_width/2, 0),   # Rear right
            (robot.body_length/3, -robot.body_width/2, robot.delta),  # Front left
            (robot.body_length/3, robot.body_width/2, robot.delta)    # Front right
        ]
        
        for wx, wy, delta in wheel_positions:
            # Transform wheel position to world frame
            x = robot.x + wx * np.cos(robot.theta) - wy * np.sin(robot.theta)
            y = robot.y + wx * np.sin(robot.theta) + wy * np.cos(robot.theta)
            
            # Draw wheel
            wheel = Rectangle((x - robot.wheel_radius/2, y - robot.wheel_width/2),
                            robot.wheel_radius, robot.wheel_width,
                            angle=np.degrees(robot.theta + delta),
                            color='black', alpha=0.8)
            self.ax.add_patch(wheel)
            
        # Heading indicator
        arrow_len = robot.body_length/2
        dx = arrow_len * np.cos(robot.theta)
        dy = arrow_len * np.sin(robot.theta)
        self.ax.arrow(robot.x, robot.y, dx, dy, 
                     head_width=0.2, head_length=0.3, fc='red', ec='red')

    def render(self, robot):
        self.ax.clear()
        
        # Draw obstacles
        for obj in self.obstacles:
            if obj.__class__.__name__ == "Box":
                self.ax.add_patch(Rectangle(
                    (obj.x - obj.width/2, obj.y - obj.length/2),
                    obj.width, obj.length,
                    angle=np.degrees(obj.rotation),
                    color='gray', alpha=0.7
                ))
            else:  # Cylinder
                self.ax.add_patch(Circle(
                    (obj.x, obj.y), obj.radius,
                    color='gray', alpha=0.7
                ))
        
        # Draw robot
        self.draw_robot(robot)
        
        # Update plot
        self.ax.set_xlim(0, self.ax.get_xlim()[1])  # Maintain limits
        self.ax.set_ylim(0, self.ax.get_ylim()[1])
        plt.pause(0.01)



# ================================================================
# ==== FILE: src/map/occupancy_grid.py ====
# ================================================================

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




# ================================================================
# ==== FILE: src/map/occupancy_map.py ====
# ================================================================

# src/map/occupancy_map.py
import numpy as np

class OccupancyMap:
    def __init__(self, size=(20,20), resolution=0.05):
        self.size = size
        self.res = resolution
        self.grid = np.zeros((int(size[0]/resolution), int(size[1]/resolution)))

    def from_obstacles(self, obstacles):
        self.grid[:] = 0
        xs = np.arange(0, self.size[0], self.res)
        ys = np.arange(0, self.size[1], self.res)
        for ox in xs:
            for oy in ys:
                for obj in obstacles:
                    if obj.contains(ox, oy):
                        i, j = int(ox/self.res), int(oy/self.res)
                        self.grid[i, j] = 1
                        break




# ================================================================
# ==== FILE: src/control/__init__.py ====
# ================================================================

"""Motion control modules."""




# ================================================================
# ==== FILE: src/control/velocity_controller.py ====
# ================================================================

"""
Trapezoidal velocity profile controller for smooth motion.
Ensures smooth acceleration and deceleration.
"""
import numpy as np
from typing import Tuple

class TrapezoidalVelocityController:
    """Trapezoidal velocity profile for smooth motion control."""
    
    def __init__(self, max_velocity: float = 0.5, max_acceleration: float = 0.3,
                 max_angular_velocity: float = 0.8):
        """
        Initialize velocity controller.
        
        Args:
            max_velocity: Maximum linear velocity (m/s)
            max_acceleration: Maximum acceleration (m/s²)
            max_angular_velocity: Maximum angular velocity (rad/s)
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_angular_velocity = max_angular_velocity
        
        self.current_velocity = 0.0
        self.target_velocity = 0.0
    
    def update(self, target_velocity: float, dt: float) -> float:
        """
        Update velocity using trapezoidal profile.
        
        Args:
            target_velocity: Desired velocity
            dt: Time step
            
        Returns:
            Commanded velocity
        """
        self.target_velocity = np.clip(target_velocity, -self.max_velocity, self.max_velocity)
        
        # Calculate velocity difference
        velocity_error = self.target_velocity - self.current_velocity
        
        # Apply acceleration limits
        max_delta_v = self.max_acceleration * dt
        
        if abs(velocity_error) > max_delta_v:
            # Accelerate/decelerate at max rate
            delta_v = np.sign(velocity_error) * max_delta_v
        else:
            # Reach target velocity
            delta_v = velocity_error
        
        self.current_velocity += delta_v
        
        return self.current_velocity
    
    def reset(self):
        """Reset controller state."""
        self.current_velocity = 0.0
        self.target_velocity = 0.0


class PathFollowingController:
    """Pure pursuit controller for path following."""
    
    def __init__(self, lookahead_distance: float = 0.5, 
                 velocity_controller: TrapezoidalVelocityController = None):
        """
        Initialize path following controller.
        
        Args:
            lookahead_distance: Distance to look ahead on path
            velocity_controller: Velocity profile controller
        """
        self.lookahead_distance = lookahead_distance
        self.velocity_controller = velocity_controller or TrapezoidalVelocityController()
    
    def compute_control(self, robot_x: float, robot_y: float, robot_theta: float,
                       path: list, dt: float) -> Tuple[float, float]:
        """
        Compute velocity and steering commands to follow path.
        
        Args:
            robot_x, robot_y, robot_theta: Robot pose
            path: List of (x, y) waypoints
            dt: Time step
            
        Returns:
            (velocity, steering_angle) commands
        """
        if not path or len(path) < 2:
            return 0.0, 0.0
        
        # Find lookahead point
        lookahead_point = self._find_lookahead_point(robot_x, robot_y, path)
        
        if lookahead_point is None:
            # Reached end of path
            return 0.0, 0.0
        
        # Calculate steering angle using pure pursuit
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        
        # Angle to lookahead point
        alpha = np.arctan2(dy, dx) - robot_theta
        
        # Normalize angle to [-pi, pi]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        # Pure pursuit steering law
        # delta = arctan(2 * L * sin(alpha) / lookahead_distance)
        # where L is wheelbase (handled in robot model)
        steering_angle = alpha  # Simplified for now
        
        # Limit steering angle
        max_steer = np.pi / 4  # 45 degrees
        steering_angle = np.clip(steering_angle, -max_steer, max_steer)
        
        # Calculate target velocity based on path curvature
        curvature = abs(steering_angle)
        if curvature > 0.5:
            target_velocity = 0.2  # Slow down for sharp turns
        elif curvature > 0.3:
            target_velocity = 0.3
        else:
            target_velocity = 0.5  # Full speed for straight paths
        
        # Apply trapezoidal velocity profile
        velocity = self.velocity_controller.update(target_velocity, dt)
        
        return velocity, steering_angle
    
    def _find_lookahead_point(self, robot_x: float, robot_y: float, 
                             path: list) -> Tuple[float, float]:
        """Find the lookahead point on the path."""
        min_dist = float('inf')
        closest_idx = 0
        
        # Find closest point on path
        for i, (px, py) in enumerate(path):
            dist = np.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Find lookahead point
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            dist = np.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            
            if dist >= self.lookahead_distance:
                return (px, py)
        
        # Return last point if no lookahead point found
        if path:
            return path[-1]
        
        return None
    
    def is_goal_reached(self, robot_x: float, robot_y: float, 
                       goal_x: float, goal_y: float, 
                       tolerance: float = 0.2) -> bool:
        """Check if robot has reached the goal."""
        dist = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        return dist < tolerance


# ================================================================
# END OF COMPILED PROJECT
# ================================================================
