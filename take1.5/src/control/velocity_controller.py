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
