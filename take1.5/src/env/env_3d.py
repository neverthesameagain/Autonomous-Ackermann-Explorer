"""
3D Renderer for 2-wheel Ackermann visualization + telemetry graphs.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Renderer3D:
    def __init__(self, obstacles, map_size=(20, 20)):
        self.obstacles = obstacles
        self.map_size = map_size

        # --- 3D Figure ---
        plt.ion()
        self.fig = plt.figure(figsize=(10, 5))
        self.ax3d = self.fig.add_subplot(121, projection="3d")
        self.ax3d.set_xlim(0, map_size[0])
        self.ax3d.set_ylim(0, map_size[1])
        self.ax3d.set_zlim(0, 2.5)
        self.ax3d.set_xlabel("X (m)", fontsize=10, fontweight='bold')
        self.ax3d.set_ylabel("Y (m)", fontsize=10, fontweight='bold')
        self.ax3d.set_zlabel("Z (m)", fontsize=10, fontweight='bold')
        self.ax3d.set_title('3D Environment - Ackermann Robot', fontsize=12, fontweight='bold')
        
        # Better viewing angle
        self.ax3d.view_init(elev=25, azim=45)
        
        # Grid and background
        self.ax3d.grid(True, alpha=0.3)
        self.ax3d.set_facecolor((0.95, 0.95, 0.95))

        # --- Telemetry Figure ---
        self.fig_telemetry, (self.ax_wheels, self.ax_ctrl) = plt.subplots(2, 1, figsize=(6, 6))
        self.time_data = []
        self.left_wheel = []
        self.right_wheel = []
        self.v_data = []
        self.delta_data = []

        # Ground plane mesh
        Xg, Yg = np.meshgrid(np.linspace(0, map_size[0], 10),
                             np.linspace(0, map_size[1], 10))
        self.Zg = np.zeros_like(Xg)
        self.Xg, self.Yg = Xg, Yg

    # ------------------------------------------------------------------
    def render(self, robot, t):
        """Render robot + update telemetry plots."""
        self.ax3d.clear()
        self.ax3d.set_xlim(0, self.map_size[0])
        self.ax3d.set_ylim(0, self.map_size[1])
        self.ax3d.set_zlim(0, 2.5)
        self.ax3d.view_init(elev=25, azim=45)
        self.ax3d.grid(True, alpha=0.3)
        self.ax3d.plot_surface(self.Xg, self.Yg, self.Zg, color="lightgray", alpha=0.3)

        # Draw obstacles
        for obj in self.obstacles:
            if obj.__class__.__name__ == "Box":
                self._draw_box(obj)
            elif obj.__class__.__name__ == "Cylinder":
                self._draw_cylinder(obj)

        # Draw the simplified 2-wheel robot
        self._draw_two_wheel_robot(robot)

        # --- Telemetry update ---
        self.time_data.append(t)
        wl, wr = robot.get_wheel_speeds()
        self.left_wheel.append(wl)
        self.right_wheel.append(wr)
        self.v_data.append(robot.v)
        self.delta_data.append(robot.delta)

        self._update_telemetry()
        plt.pause(0.001)

    # ------------------------------------------------------------------
    def _draw_robot(self, robot):
        """Draw the robot as a 4-wheel Ackermann vehicle with turning front wheels."""
        body_length = robot.body_length
        body_width = robot.body_width
        wheel_radius = robot.wheel_radius
        wheel_width = robot.wheel_width
        
        x, y, theta = robot.x, robot.y, robot.theta
        steering_angle = robot.delta  # Current steering angle
        
        # === DRAW BODY ===
        # Body corners
        corners_x = [-body_length/2, body_length/2, body_length/2, -body_length/2, -body_length/2]
        corners_y = [-body_width/2, -body_width/2, body_width/2, body_width/2, -body_width/2]
        
        # Rotate and translate
        rotated_x = []
        rotated_y = []
        for cx, cy in zip(corners_x, corners_y):
            rx = x + cx * np.cos(theta) - cy * np.sin(theta)
            ry = y + cx * np.sin(theta) + cy * np.cos(theta)
            rotated_x.append(rx)
            rotated_y.append(ry)
        
        # Draw body (blue box)
        z_bottom = [0.1] * 5
        z_top = [0.4] * 5
        self.ax3d.plot(rotated_x, rotated_y, z_bottom, 'b-', linewidth=3, alpha=0.8)
        self.ax3d.plot(rotated_x, rotated_y, z_top, 'b-', linewidth=3, alpha=0.8)
        
        # Connect bottom to top
        for i in range(4):
            self.ax3d.plot([rotated_x[i], rotated_x[i]], 
                        [rotated_y[i], rotated_y[i]], 
                        [z_bottom[i], z_top[i]], 'b-', linewidth=2, alpha=0.6)
        
        # === DRAW 4 WHEELS ===
        # Wheel positions in robot frame
        wheelbase = robot.wheelbase
        rear_axle_offset = -body_length/3
        front_axle_offset = rear_axle_offset + wheelbase
        
        wheel_positions = [
            (rear_axle_offset, -body_width/2, 0),      # Rear left (no steering)
            (rear_axle_offset, body_width/2, 0),       # Rear right (no steering)
            (front_axle_offset, -body_width/2, steering_angle),  # Front left (steered)
            (front_axle_offset, body_width/2, steering_angle),   # Front right (steered)
        ]
        
        for i, (wx, wy, wheel_angle) in enumerate(wheel_positions):
            # Rotate wheel position to world frame
            rwx = x + wx * np.cos(theta) - wy * np.sin(theta)
            rwy = y + wx * np.sin(theta) + wy * np.cos(theta)
            
            # Total wheel orientation (robot heading + steering)
            total_angle = theta + wheel_angle
            
            # Draw wheel as a small box
            wheel_half_length = wheel_radius
            wheel_half_width = wheel_width / 2
            
            # Wheel corners in wheel frame
            w_corners_x = [-wheel_half_length, wheel_half_length, wheel_half_length, -wheel_half_length, -wheel_half_length]
            w_corners_y = [-wheel_half_width, -wheel_half_width, wheel_half_width, wheel_half_width, -wheel_half_width]
            
            # Rotate and translate wheel corners
            w_rotated_x = []
            w_rotated_y = []
            for wcx, wcy in zip(w_corners_x, w_corners_y):
                wrx = rwx + wcx * np.cos(total_angle) - wcy * np.sin(total_angle)
                wry = rwy + wcx * np.sin(total_angle) + wcy * np.cos(total_angle)
                w_rotated_x.append(wrx)
                w_rotated_y.append(wry)
            
            # Draw wheel (black for rear, red for front to show steering)
            wheel_color = 'red' if i >= 2 else 'black'
            wheel_z = [0.05] * 5
            self.ax3d.plot(w_rotated_x, w_rotated_y, wheel_z, 
                        color=wheel_color, linewidth=4, alpha=0.9)
            
            # Draw wheel top
            wheel_z_top = [0.15] * 5
            self.ax3d.plot(w_rotated_x, w_rotated_y, wheel_z_top, 
                        color=wheel_color, linewidth=3, alpha=0.7)
        
        # === DRAW HEADING INDICATOR ===
        arrow_length = 0.6
        arrow_x = x + arrow_length * np.cos(theta)
        arrow_y = y + arrow_length * np.sin(theta)
        self.ax3d.plot([x, arrow_x], [y, arrow_y], [0.4, 0.4], 
                    'yellow', linewidth=3, alpha=0.9)

    # ------------------------------------------------------------------
    def _update_telemetry(self):
        """Live-update telemetry subplots."""
        self.ax_wheels.clear()
        self.ax_wheels.plot(self.time_data, self.left_wheel, label="Left Wheel ω")
        self.ax_wheels.plot(self.time_data, self.right_wheel, label="Right Wheel ω")
        self.ax_wheels.set_ylabel("Angular Velocity (rad/s)")
        self.ax_wheels.legend()
        self.ax_wheels.grid(True)

        self.ax_ctrl.clear()
        self.ax_ctrl.plot(self.time_data, self.v_data, label="v (m/s)")
        self.ax_ctrl.plot(self.time_data, self.delta_data, label="δ (rad)")
        self.ax_ctrl.set_xlabel("Time (s)")
        self.ax_ctrl.legend()
        self.ax_ctrl.grid(True)

    # ------------------------------------------------------------------
    def _draw_box(self, obj):
        X = [obj.x - obj.width/2, obj.x + obj.width/2]
        Y = [obj.y - obj.length/2, obj.y + obj.length/2]
        X, Y = np.meshgrid(X, Y)
        Z = np.zeros_like(X)
        self.ax3d.plot_surface(X, Y, Z + obj.height, color=obj.color or "gray", alpha=0.5)

    def _draw_cylinder(self, obj):
        theta = np.linspace(0, 2*np.pi, 30)
        z = np.linspace(0, obj.height, 5)
        theta, z = np.meshgrid(theta, z)
        X = obj.x + obj.radius*np.cos(theta)
        Y = obj.y + obj.radius*np.sin(theta)
        self.ax3d.plot_surface(X, Y, z, color=obj.color or "gray", alpha=0.6)

    def close(self):
        plt.close(self.fig)
        plt.close(self.fig_telemetry)
