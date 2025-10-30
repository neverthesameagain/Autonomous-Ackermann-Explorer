import math
import numpy as np

class AckermannRobot:
    def __init__(self, x=0.0, y=0.0, theta=0.0, L=0.5, v_max=0.5, a_lin=0.5, dt=0.05):
        # Robot physical parameters
        self.x, self.y, self.theta = x, y, theta
        self.L = L  # wheelbase
        self.v = 0.0  # current velocity
        self.delta = 0.0  # current steering angle
        
        # Motion constraints
        self.v_max = v_max  # maximum velocity
        self.a_lin = a_lin  # linear acceleration limit
        self.max_delta = 0.7  # ~40 degrees max steering
        self.delta_rate = 1.0  # steering rate limit (rad/s)
        self.dt = dt
        
        # Trajectory history for visualization
        self.history = {'x': [x], 'y': [y], 'theta': [theta]}
        
    def move(self, v_target, delta_target):
        """
        Update robot state using Ackermann kinematics with realistic constraints
        """
        # Apply motion limits
        delta_target = np.clip(delta_target, -self.max_delta, self.max_delta)
        v_target = np.clip(v_target, -self.v_max, self.v_max)
        
        # Smooth velocity profile (trapezoidal)
        if abs(v_target - self.v) > self.a_lin * self.dt:
            self.v += self.a_lin * self.dt * (1 if v_target > self.v else -1)
        else:
            self.v = v_target
            
        # Smooth steering dynamics
        delta_change = np.clip(
            delta_target - self.delta,
            -self.delta_rate * self.dt,
            self.delta_rate * self.dt
        )
        self.delta += delta_change

        # Update kinematics
        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt
        self.theta += (self.v / self.L) * math.tan(self.delta) * self.dt
        
        # Normalize angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Record history
        self.history['x'].append(self.x)
        self.history['y'].append(self.y)
        self.history['theta'].append(self.theta)

    def state(self):
        """Return current robot state"""
        return self.x, self.y, self.theta
        
    def predict_trajectory(self, v_cmd, delta_cmd, steps=10):
        """
        Predict future trajectory for collision checking
        """
        x, y, theta = self.x, self.y, self.theta
        v, delta = self.v, self.delta
        
        pred_x, pred_y = [], []
        
        for _ in range(steps):
            # Apply motion constraints
            if abs(v_cmd - v) > self.a_lin * self.dt:
                v += self.a_lin * self.dt * (1 if v_cmd > v else -1)
            else:
                v = v_cmd
                
            delta_change = np.clip(
                delta_cmd - delta,
                -self.delta_rate * self.dt,
                self.delta_rate * self.dt
            )
            delta += delta_change
            
            # Forward propagate
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += (v / self.L) * math.tan(delta) * self.dt
            
            pred_x.append(x)
            pred_y.append(y)
            
        return pred_x, pred_y
        
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """Reset robot state"""
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0
        self.delta = 0.0
        self.history = {'x': [x], 'y': [y], 'theta': [theta]}
