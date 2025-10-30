import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import binary_dilation
from robot import AckermannRobot
from planner import astar
from controller import PurePursuitController
from visualize3d import Visualizer3D

def create_test_environment(size=30):
    """Create a test environment with some obstacles"""
    grid = np.zeros((size, size))
    
    # Add some obstacles
    grid[10:15, 10:12] = 1  # vertical wall
    grid[5:10, 20:22] = 1   # second wall
    grid[15:20, 15:17] = 1  # third wall
    return grid

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

def main():
    # Environment setup
    size = 30
    grid = create_test_environment(size)
    grid_inflated = inflate_obstacles(grid, radius=1)
    
    # Start and goal positions
    start = (2, 2)
    goal = (25, 25)
    
    # Plan initial path
    path = astar(grid_inflated, start, goal)
    if not path:
        print("No valid path found!")
        return
        
    # Initialize visualization
    viz = Visualizer3D(size)
    
    # Initialize robot
    initial_heading = np.arctan2(path[1][1] - path[0][1],
                                path[1][0] - path[0][0])
    robot = AckermannRobot(x=start[0], y=start[1], theta=initial_heading)
    
    # Initialize controller
    controller = PurePursuitController(lookahead=2.0, v_ref=0.3)
    
    # Simulation loop
    reached_goal = False
    replanning_cooldown = 0
    
    while not reached_goal:
        # Get current target point
        target = controller.find_target((robot.x, robot.y), path)
        
        # Compute control inputs
        v, delta = controller.compute_control(
            (robot.x, robot.y, robot.theta), 
            target,
            robot.L
        )
        
        # Predict trajectory for collision checking
        pred_x, pred_y = robot.predict_trajectory(v, delta, steps=5)
        
        # Check for potential collisions
        collision_predicted = any(
            check_collision(px, py, grid_inflated)
            for px, py in zip(pred_x, pred_y)
        )
        
        if collision_predicted and replanning_cooldown == 0:
            # Replan path from current position
            current_pos = (int(robot.x), int(robot.y))
            new_path = astar(grid_inflated, current_pos, goal)
            
            if new_path:
                path = new_path
                print("Path replanned due to predicted collision")
                replanning_cooldown = 10  # Avoid constant replanning
            else:
                print("Warning: No alternative path found")
                v = 0  # Stop if no alternative
        
        # Update robot state
        robot.move(v, delta)
        
        # Update visualization
        viz.update(grid_inflated, robot, path)
        
        # Check if goal reached
        dist_to_goal = np.hypot(robot.x - goal[0], robot.y - goal[1])
        reached_goal = dist_to_goal < 0.5
        
        # Update cooldown
        replanning_cooldown = max(0, replanning_cooldown - 1)
        
    plt.show()  # Keep final plot open

if __name__ == "__main__":
    main()
