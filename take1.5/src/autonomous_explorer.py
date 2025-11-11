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
