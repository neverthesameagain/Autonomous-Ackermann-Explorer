import numpy as np
from math import sin, cos, tan, atan, pi

class AckermannVehicle:
    """Full 4-wheel Ackermann-steered vehicle model with wheel geometry."""
    
    def __init__(self, x=0.0, y=0.0, theta=0.0, wheelbase=0.55, track_width=0.3, wheel_radius=0.06, dt=0.01):
        # Vehicle geometry
        self.L = wheelbase        # 0.55m wheelbase for better stability
        self.T = track_width     # 0.3m track width
        self.r_wheel = wheel_radius
        self.dt = dt
        
        # State variables
        self.x = x  # global x position
        self.y = y  # global y position
        self.theta = theta  # heading angle
        self.v = 0.0  # linear velocity
        self.omega = 0.0  # angular velocity
        
        # Wheel states
        self.delta_left = 0.0   # left front steering angle
        self.delta_right = 0.0  # right front steering angle
        self.omega_LF = 0.0     # left front wheel angular velocity
        self.omega_RF = 0.0     # right front wheel angular velocity
        self.omega_LR = 0.0     # left rear wheel angular velocity
        self.omega_RR = 0.0     # right rear wheel angular velocity
        
        # Motion limits - increased for better performance
        self.max_steering = 0.7  # ~40 degrees
        self.max_velocity = 2.0  # Increased from 1.0 to 2.0 m/s
        self.max_acceleration = 1.0  # Increased from 0.5 to 1.0 m/s²
        
        # History for plotting
        self.history = {
            'time': [],
            'x': [], 'y': [], 'theta': [],
            'v': [], 'omega': [],
            'delta_left': [], 'delta_right': [],
            'omega_LF': [], 'omega_RF': [], 
            'omega_LR': [], 'omega_RR': [],
            'v_LF': [], 'v_RF': [], 'v_LR': [], 'v_RR': []
        }
        self.t = 0.0
        
    def get_wheel_positions(self):
        """Calculate current positions of all four wheels."""
        # Center positions
        front_center = (
            self.x + self.L/2 * cos(self.theta),
            self.y + self.L/2 * sin(self.theta)
        )
        rear_center = (
            self.x - self.L/2 * cos(self.theta),
            self.y - self.L/2 * sin(self.theta)
        )
        
        # Wheel positions relative to centers
        wheels = {
            'FL': (  # Front Left
                front_center[0] - (self.T/2) * sin(self.theta),
                front_center[1] + (self.T/2) * cos(self.theta),
                self.delta_left + self.theta
            ),
            'FR': (  # Front Right
                front_center[0] + (self.T/2) * sin(self.theta),
                front_center[1] - (self.T/2) * cos(self.theta),
                self.delta_right + self.theta
            ),
            'RL': (  # Rear Left
                rear_center[0] - (self.T/2) * sin(self.theta),
                rear_center[1] + (self.T/2) * cos(self.theta),
                self.theta
            ),
            'RR': (  # Rear Right
                rear_center[0] + (self.T/2) * sin(self.theta),
                rear_center[1] - (self.T/2) * cos(self.theta),
                self.theta
            )
        }
        return wheels
        
    def compute_steering_angles(self, delta_center):
        """
        Compute individual wheel steering angles using Ackermann geometry.
        
        Args:
            delta_center: Center steering angle command
        """
        if abs(delta_center) < 1e-4:  # Straight line case
            self.delta_left = 0
            self.delta_right = 0
            return
            
        # Compute turning radius from center steering angle
        R = self.L / tan(abs(delta_center))
        
        # Compute individual steering angles
        self.delta_left = atan(self.L / (R - np.sign(delta_center) * self.T/2))
        self.delta_right = atan(self.L / (R + np.sign(delta_center) * self.T/2))
        
        if delta_center < 0:
            self.delta_left, self.delta_right = -self.delta_right, -self.delta_left
            
    def compute_wheel_velocities(self):
        """Compute individual wheel velocities based on vehicle motion."""
        if abs(self.omega) < 1e-4:  # Straight line motion
            v_wheels = self.v
            self.omega_LF = self.omega_RF = self.omega_LR = self.omega_RR = self.v / self.r_wheel
        else:
            # Compute turning radius and wheel paths
            R = self.v / self.omega  # Instantaneous turning radius
            
            # Compute wheel velocities based on their turning radii
            R_LF = R - self.T/2
            R_RF = R + self.T/2
            R_LR = R - self.T/2
            R_RR = R + self.T/2
            
            # Convert to angular velocities
            self.omega_LF = self.v * R_LF / (R * self.r_wheel)
            self.omega_RF = self.v * R_RF / (R * self.r_wheel)
            self.omega_LR = self.v * R_LR / (R * self.r_wheel)
            self.omega_RR = self.v * R_RR / (R * self.r_wheel)
        
    def update(self, v_cmd, delta_cmd):
        """
        Update vehicle state for one timestep.
        
        Args:
            v_cmd: Commanded linear velocity
            delta_cmd: Commanded center steering angle
        """
        # Apply motion limits
        v_cmd = np.clip(v_cmd, -self.max_velocity, self.max_velocity)
        delta_cmd = np.clip(delta_cmd, -self.max_steering, self.max_steering)
        
        # Smooth acceleration
        dv = np.clip(v_cmd - self.v, 
                    -self.max_acceleration * self.dt,
                    self.max_acceleration * self.dt)
        self.v += dv
        
        # Update steering geometry
        self.compute_steering_angles(delta_cmd)
        
        # Update position and heading (bicycle model approximation)
        self.x += self.v * cos(self.theta) * self.dt
        self.y += self.v * sin(self.theta) * self.dt
        self.omega = (self.v / self.L) * tan(delta_cmd)
        self.theta += self.omega * self.dt
        
        # Compute wheel velocities
        self.compute_wheel_velocities()
        
        # Record history
        self.t += self.dt
        self.history['time'].append(self.t)
        self.history['x'].append(self.x)
        self.history['y'].append(self.y)
        self.history['theta'].append(self.theta)
        self.history['v'].append(self.v)
        self.history['omega'].append(self.omega)
        self.history['delta_left'].append(self.delta_left)
        self.history['delta_right'].append(self.delta_right)
        self.history['omega_LF'].append(self.omega_LF)
        self.history['omega_RF'].append(self.omega_RF)
        self.history['omega_LR'].append(self.omega_LR)
        self.history['omega_RR'].append(self.omega_RR)
        self.history['v_LF'].append(self.omega_LF * self.r_wheel)
        self.history['v_RF'].append(self.omega_RF * self.r_wheel)
        self.history['v_LR'].append(self.omega_LR * self.r_wheel)
        self.history['v_RR'].append(self.omega_RR * self.r_wheel)
        
    def get_state(self):
        """Return current vehicle state."""
        return {
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'v': self.v,
            'omega': self.omega
        }
        
    def reset(self, x=0.0, y=0.0, theta=0.0):
        """Reset vehicle state."""
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.delta_left = 0.0
        self.delta_right = 0.0
        self.omega_LF = self.omega_RF = self.omega_LR = self.omega_RR = 0.0
        
        # Clear history
        for key in self.history:
            self.history[key] = []
        self.t = 0.0