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

# In env_2d.py
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