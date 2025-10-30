import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation
from ackermann_vehicle import AckermannVehicle
from planner import astar
from controller import PurePursuitController
from visualize import VehicleVisualizer
from exploration import Explorer
import time

def create_test_environment(size=30):
    """Create a test environment with some obstacles and goals"""
    grid = np.zeros((size, size))
    
    # Add some obstacles
    grid[10:15, 10:12] = 1  # vertical wall
    grid[1:3, 2:4] = 1   # second wall
    grid[15:20, 15:17] = 1  # third wall
    
    # Define multiple goals for testing
    goals = [
        (7, 7),  # far corner
        (15, 25),  # mid-right
        (5, 25),   # near-right
        (25, 5)    # far-left
    ]
    return grid, goals

def inflate_obstacles(grid, radius=1):
    """Inflate obstacles by safety radius"""
    mask = np.ones((2*radius+1, 2*radius+1))
    return binary_dilation(grid, structure=mask).astype(int)

def check_collision(x, y, grid):
    """Check if position collides with obstacles"""
    ix, iy = int(round(x)), int(round(y))
    if 0 <= ix < grid.shape[0] and 0 <= iy < grid.shape[1]:
        return grid[ix, iy] == 1
    return True  # out of bounds is considered collision

def exploration_loop(vehicle, true_map, explorer, planner, controller, viz, goals, max_iterations=1000):
    """
    Main exploration loop with multiple goals and improved frontier selection
    
    Args:
        vehicle: AckermannVehicle instance
        true_map: Complete environment map (ground truth)
        explorer: Explorer instance for frontier detection
        planner: Path planner (A*)
        controller: Pure Pursuit controller
        viz: VehicleVisualizer instance
        goals: List of goal positions to visit
    """
    iteration = 0
    last_pos = (vehicle.x, vehicle.y)
    current_goal_idx = 0
    replan_countdown = 0
    stuck_count = 0
    last_coverage = 0
    goal_timeout = 200  # Maximum iterations per goal
    goal_timer = 0
    
    print("\nStarting exploration with multiple goals...")
    print(f"Total goals to visit: {len(goals)}")
    
    while iteration < max_iterations:
        # Update known map with current sensor data
        explorer.update_map_with_sensor(true_map, (vehicle.x, vehicle.y))
        
        # Check coverage progress
        current_coverage = explorer.get_coverage_metrics()['coverage']
        if abs(current_coverage - last_coverage) < 0.1:
            stuck_count += 1
        else:
            stuck_count = 0
        last_coverage = current_coverage
        
        # Current goal handling
        current_goal = goals[current_goal_idx]
        goal_timer += 1
        
        # Print current status
        print(f"\rGoal {current_goal_idx + 1}/{len(goals)} at {current_goal} | "
              f"Coverage: {current_coverage:.1f}% | "
              f"Frontiers visited: {explorer.frontiers_visited}", end="")
        
        # Check if we should move to next goal
        dist_to_goal = np.hypot(vehicle.x - current_goal[0],
                               vehicle.y - current_goal[1])
        if dist_to_goal < 2.0 or goal_timer >= goal_timeout or stuck_count > 50:
            print(f"\nCompleted or timeout at goal {current_goal_idx + 1}")
            current_goal_idx = (current_goal_idx + 1) % len(goals)
            goal_timer = 0
            stuck_count = 0
            if current_goal_idx == 0:  # Completed full cycle
                if current_coverage > 60:  # Reasonable coverage achieved
                    print("\nExploration complete! Good coverage achieved.")
                    break
        
        # Find frontiers
        frontiers = explorer.find_frontiers()
        
        if not frontiers:
            print("\nNo frontiers found, moving to next goal...")
            current_goal_idx = (current_goal_idx + 1) % len(goals)
            goal_timer = 0
            continue
        
        # Use planning map for frontier selection
        planning_map = np.where(explorer.known_map >= 0, explorer.known_map, 0)
        planning_map = inflate_obstacles(planning_map, radius=1)
        
        # Select best frontier considering current goal
        target = explorer.select_frontier(frontiers, current_goal, 
                                       planning_map, planner)
                                       
        if target:
            print(f"\nMoving to frontier at ({target[0]:.1f}, {target[1]:.1f}) towards goal {current_goal}")
            
        # If no valid frontier found, try moving directly to goal
        if target is None:
            print("\nNo valid frontier found, attempting direct goal approach...")
            target = current_goal
            
        # Plan path to frontier
        start = (int(vehicle.x), int(vehicle.y))
        goal = (int(target[0]), int(target[1]))
        
        # Use known map for planning
        planning_map = np.where(explorer.known_map >= 0, explorer.known_map, 0)
        planning_map = inflate_obstacles(planning_map, radius=1)
        
        path = planner(planning_map, start, goal)
        if not path:
            print(f"No path found to frontier at {goal}")
            continue
            
        # Follow path to frontier
        reached_target = False
        replanning_cooldown = 0
        
        while not reached_target and iteration < max_iterations:
            current_target = controller.find_target((vehicle.x, vehicle.y), path)
            v, delta = controller.compute_control(
                (vehicle.x, vehicle.y, vehicle.theta),
                current_target,
                vehicle.L
            )
            
            # Update vehicle state with full wheel dynamics
            vehicle.update(v, delta)
            
            # Update explorer metrics
            dx = vehicle.x - last_pos[0]
            dy = vehicle.y - last_pos[1]
            explorer.total_distance += np.hypot(dx, dy)
            last_pos = (vehicle.x, vehicle.y)
            
            # Update visualization with vehicle geometry
            viz.update(vehicle, explorer.known_map, path, frontiers=frontiers)
            
            # Check if target reached
            dist_to_target = np.hypot(vehicle.x - goal[0], vehicle.y - goal[1])
            reached_target = dist_to_target < 0.5
            
            # Update map as we move
            explorer.update_map_with_sensor(true_map, (vehicle.x, vehicle.y))
            
            # Check for obstacles in front of the robot
            heading_x = vehicle.x + 2.0 * np.cos(vehicle.theta)
            heading_y = vehicle.y + 2.0 * np.sin(vehicle.theta)
            
            # Check points along the robot's heading
            obstacle_detected = False
            for d in np.linspace(0, 2.0, 5):  # Check 5 points up to 2 units ahead
                check_x = vehicle.x + d * np.cos(vehicle.theta)
                check_y = vehicle.y + d * np.sin(vehicle.theta)
                grid_x, grid_y = int(check_x), int(check_y)
                
                if (0 <= grid_x < planning_map.shape[0] and 
                    0 <= grid_y < planning_map.shape[1] and 
                    planning_map[grid_x, grid_y] == 1):
                    obstacle_detected = True
                    break
            
            # Adjust velocity based on obstacles
            obstacle_factor = 0.0 if obstacle_detected else 1.0
            
            # Check for collisions and replan if needed
            if replanning_cooldown == 0:
                current_pos = (int(vehicle.x), int(vehicle.y))
                # Check if current path is still valid
                if obstacle_detected or any(planning_map[int(x), int(y)] == 1 for x, y in path):
                    new_path = planner(planning_map, current_pos, goal)
                    if new_path:
                        path = new_path
                        replanning_cooldown = 10
                    else:
                        # If no path found, try to rotate in place to find new path
                        v = 0
                        delta = 0.5  # Rotate in place
                        
            # Modify controls based on obstacle detection
            if obstacle_detected:
                v *= 0.0  # Stop forward motion
                if abs(delta) < 0.1:  # If not already turning
                    delta = 0.5  # Turn to avoid obstacle
            
            # Update iteration and cooldown
            iteration += 1
            replanning_cooldown = max(0, replanning_cooldown - 1)
        
        if reached_target:
            explorer.frontiers_visited += 1
            
        # Check if we've explored enough around current goal
        coverage_near_goal = explorer.get_coverage_near_point(current_goal, radius=5)
        if coverage_near_goal > 0.9:  # 90% explored
            current_goal_idx += 1
            
    return explorer.get_coverage_metrics()

def main():
    # Environment setup
    size = 30
    true_map, goals = create_test_environment(size)
    
    # Initialize vehicle at starting position
    start = (0, 0)
    vehicle = AckermannVehicle(x=start[0], y=start[1], theta=0)
    
    # Sort goals by distance from start position
    goals = sorted(goals, key=lambda g: np.hypot(g[0] - start[0], g[1] - start[1]))
    
    # Initialize exploration components
    explorer = Explorer(size=(size, size), sensor_range=5)  # Increased sensor range for faster speed
    controller = PurePursuitController(
        lookahead=3.0,     # Increased lookahead for higher speeds
        v_ref=2.0,         # Increased reference velocity
        Kp=2.0,           # Increased steering responsiveness
        dist_threshold=1.0 # Increased threshold for smoother path following
    )
    viz = VehicleVisualizer(map_size=size)
    
    # Run exploration
    print("Starting autonomous exploration...")
    metrics = exploration_loop(
        vehicle=vehicle,
        true_map=true_map,
        explorer=explorer,
        planner=astar,
        controller=controller,
        viz=viz,
        goals=goals
    )
    
    # Print and save results
    print("\nExploration Results:")
    print(f"Coverage: {metrics['coverage']:.2f}%")
    print(f"Frontiers Visited: {metrics['frontiers_visited']}")
    print(f"Total Distance: {metrics['total_distance']:.2f}m")
    
    # Save comprehensive simulation results
    from metrics_logger import save_simulation_metrics
    save_simulation_metrics(vehicle, explorer)
    
    # Keep final visualization open
    plt.show()

if __name__ == "__main__":
    main()
