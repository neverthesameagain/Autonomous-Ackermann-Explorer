import numpy as np

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
        self.delta = delta  # store steering for visualization
        self.v = v  # store velocity
        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += (v / self.wheelbase) * np.tan(delta) * self.dt
        
    def get_wheel_speeds(self):
        """Return left/right angular velocities based on kinematics."""
        track = self.body_width
        v_left = self.v - (self.v / self.wheelbase) * np.tan(self.delta) * (track / 2)
        v_right = self.v + (self.v / self.wheelbase) * np.tan(self.delta) * (track / 2)
        ω_left = v_left / self.wheel_radius
        ω_right = v_right / self.wheel_radius
        return ω_left, ω_right
    
    def get_pose(self):
        """Get the current pose of the robot."""
        return self.x, self.y, self.theta
    
    def reset(self, x=0, y=0, theta=0):
        """Reset the robot to a specific pose."""
        self.x = x
        self.y = y
        self.theta = theta
        self.path = [(x, y, theta)]
        return self.get_pose()