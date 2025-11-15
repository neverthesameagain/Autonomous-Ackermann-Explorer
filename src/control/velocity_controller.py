import math
import numpy as np

class PurePursuitController:
    def __init__(self, lookahead=1.0, Kp=1.5, v_ref=0.3, dist_threshold=0.5):
        """
        Initialize Pure Pursuit controller
        
        Args:
            lookahead: Lookahead distance for path following
            Kp: Proportional gain for steering control
            v_ref: Reference velocity
            dist_threshold: Distance threshold for path point switching
        """
        self.lookahead = lookahead
        self.Kp = Kp
        self.v_ref = v_ref
        self.dist_threshold = dist_threshold
        self.current_idx = 0
        
    def find_target(self, robot_pos, path):
        """
        Find target point on path that is lookahead distance away
        
        Args:
            robot_pos: Current (x,y) position of robot
            path: List of (x,y) path points
        """
        x, y = robot_pos
        
        # Look for a point that's lookahead distance away
        best_dist = float('inf')
        target_point = None
        
        # Start searching from current index
        for i in range(self.current_idx, len(path)):
            px, py = path[i]
            dist = math.hypot(px - x, py - y)
            
            # Update closest point index
            if dist < self.dist_threshold:
                self.current_idx = i
                
            # Find point closest to lookahead distance
            dist_diff = abs(dist - self.lookahead)
            if dist_diff < best_dist:
                best_dist = dist_diff
                target_point = (px, py)
                
            # If we're past lookahead distance, no need to search further
            if dist > self.lookahead * 1.5:
                break
                
        # If no point found, use the last path point
        if target_point is None:
            target_point = path[-1]
            
        return target_point
        
    def compute_control(self, robot_state, target, L):
        """
        Compute control inputs using pure pursuit algorithm
        
        Args:
            robot_state: Tuple of (x, y, theta)
            target: Target point (x, y)
            L: Robot wheelbase
            
        Returns:
            v: Linear velocity command
            delta: Steering angle command
        """
        x, y, theta = robot_state
        tx, ty = target
        
        # Transform target to robot frame
        dx = tx - x
        dy = ty - y
        target_dist = math.hypot(dx, dy)
        
        # Compute target angle and cross track error
        target_angle = math.atan2(dy, dx)
        alpha = target_angle - theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # wrap angle
        
        # Pure pursuit steering control
        delta = math.atan2(2 * L * math.sin(alpha), max(0.1, self.lookahead))
        
        # Adjust velocity based on multiple factors
        turning_factor = 1 - abs(delta) / math.pi  # reduce speed in sharp turns
        obstacle_factor = 1.0  # will be set by exploration loop
        
        # Calculate base velocity
        v = self.v_ref * turning_factor * obstacle_factor
        
        # Allow zero velocity for obstacle avoidance while maintaining steering
        v = max(0.0, v)  # can be zero for pure rotation
        
        # Limit maximum steering angle
        delta = np.clip(delta, -math.pi/3, math.pi/3)  # limit to ±60 degrees
        
        return v, delta
        
    def reset(self):
        """Reset controller state"""
        self.current_idx = 0
"""
Trapezoidal velocity profile controller for smooth motion.
Ensures smooth acceleration and deceleration.
"""
import numpy as np
from typing import Tuple

class TrapezoidalVelocityController:
    """Trapezoidal velocity profile for smooth motion control."""
    
    def __init__(self, max_velocity: float = 2.5, max_acceleration: float = 2.3,
                 max_angular_velocity: float = 1.8):
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
    
    def __init__(self, lookahead_distance: float = 0.8,
                 velocity_controller: TrapezoidalVelocityController = None,
                 wheelbase: float = 0.3,
                 min_lookahead: float = 0.25,
                 max_lookahead: float = 1.5):
        """
        Initialize path following controller.
        
        Args:
            lookahead_distance: Nominal lookahead distance on the path
            velocity_controller: Velocity profile controller
            wheelbase: Vehicle wheelbase used by pure pursuit law
            min_lookahead: Minimum admissible lookahead
            max_lookahead: Maximum admissible lookahead
        """
        self.lookahead_distance = lookahead_distance
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.wheelbase = wheelbase
        self.velocity_controller = velocity_controller or TrapezoidalVelocityController()
        self._last_target_idx = 0
    
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
            self.reset()
            return 0.0, 0.0
        
        lookahead_point, lookahead_dist, idx = self._find_lookahead_point(robot_x, robot_y, path)
        if lookahead_point is None:
            self.reset()
            return 0.0, 0.0
        self._last_target_idx = idx
        
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        alpha = np.arctan2(dy, dx) - robot_theta
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        ld = max(self.min_lookahead, min(lookahead_dist, self.max_lookahead))
        max_steer = np.deg2rad(40.0)
        steering_angle = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), ld)
        steering_angle = np.clip(steering_angle, -max_steer, max_steer)
        
        curvature = abs(np.tan(steering_angle) / max(self.wheelbase, 1e-3))
        distance_to_goal = np.hypot(path[-1][0] - robot_x, path[-1][1] - robot_y)
        base_speed = self.velocity_controller.max_velocity
        curvature_factor = 1.0 / (1.0 + 3.5 * curvature)
        target_velocity = base_speed * curvature_factor
        if distance_to_goal < 1.0:
            target_velocity = min(target_velocity, 0.8)
        if distance_to_goal < 0.4:
            target_velocity = min(target_velocity, 0.25)
        target_velocity = max(0.0, target_velocity)
        
        velocity = self.velocity_controller.update(target_velocity, dt)
        return velocity, steering_angle
    
    def _find_lookahead_point(self, robot_x: float, robot_y: float, 
                             path: list) -> Tuple[Tuple[float, float], float, int]:
        """Find the lookahead point on the path."""
        if not path:
            return None, 0.0, 0
        
        desired_ld = np.clip(self.lookahead_distance, self.min_lookahead, self.max_lookahead)
        search_start = max(0, self._last_target_idx - 5)
        best_idx = search_start
        closest_dist = float('inf')
        lookahead_point = None
        
        for i in range(search_start, len(path)):
            px, py = path[i]
            dist = np.hypot(px - robot_x, py - robot_y)
            if dist < closest_dist:
                closest_dist = dist
                best_idx = i
            if dist >= desired_ld:
                lookahead_point = (px, py)
                best_idx = i
                break
        
        if lookahead_point is None:
            lookahead_point = path[-1]
            best_idx = len(path) - 1
            closest_dist = np.hypot(lookahead_point[0] - robot_x, lookahead_point[1] - robot_y)
        
        lookahead_dist = max(closest_dist, desired_ld)
        return lookahead_point, lookahead_dist, best_idx
    
    def is_goal_reached(self, robot_x: float, robot_y: float, 
                       goal_x: float, goal_y: float, 
                       tolerance: float = 0.2) -> bool:
        """Check if robot has reached the goal."""
        dist = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        return dist < tolerance
    
    def reset(self):
        """Reset controller history."""
        self._last_target_idx = 0
        if self.velocity_controller:
            self.velocity_controller.reset()
