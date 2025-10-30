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
        delta = math.atan2(2 * L * math.sin(alpha), self.lookahead)
        
        # Adjust velocity based on curvature
        v = self.v_ref * (1 - abs(delta) / math.pi)  # slow down for turns
        v = max(0.1, v)  # maintain minimum velocity
        
        return v, delta
        
    def reset(self):
        """Reset controller state"""
        self.current_idx = 0
