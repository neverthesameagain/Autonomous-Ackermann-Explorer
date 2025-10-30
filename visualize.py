import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.transforms as transforms

class VehicleVisualizer:
    def __init__(self, map_size=30, vehicle_color='blue', 
                 wheel_color='black', trail_color='red'):
        """Initialize visualization with both 2D map view and motion plots."""
        
        # Create figure with subplots
        self.fig = plt.figure(figsize=(15, 10))
        self.gs = self.fig.add_gridspec(3, 3)
        
        # Main 2D view
        self.ax_main = self.fig.add_subplot(self.gs[:2, :2])
        self.ax_main.set_aspect('equal')
        self.ax_main.set_xlim(-1, map_size+1)
        self.ax_main.set_ylim(-1, map_size+1)
        
        # Add title and project info
        self.fig.suptitle('Autonomous Obstacle-Aware Exploration and Path Planning\n' +
                         'Four-Wheel Ackermann Robot in 3D Environment',
                         fontsize=14, y=0.98)
        plt.figtext(0.02, 0.02, 'IIT Palakkad - EE5608 Project\nAryan Mathur (122201017)',
                   fontsize=8, ha='left')
        
        # Vehicle visualization parameters
        self.vehicle_color = vehicle_color
        self.wheel_color = wheel_color
        self.trail_color = trail_color
        
        # Motion plots
        self.ax_v = self.fig.add_subplot(self.gs[0, 2])
        self.ax_omega = self.fig.add_subplot(self.gs[1, 2])
        self.ax_steering = self.fig.add_subplot(self.gs[2, 0])
        self.ax_wheel_v = self.fig.add_subplot(self.gs[2, 1:])
        
        self._setup_plot_styles()
        
    def _setup_plot_styles(self):
        """Configure plot styles and labels."""
        # Velocity plot
        self.ax_v.set_title('Linear Velocity')
        self.ax_v.set_xlabel('Time [s]')
        self.ax_v.set_ylabel('v [m/s]')
        self.ax_v.grid(True)
        
        # Angular velocity plot
        self.ax_omega.set_title('Angular Velocity')
        self.ax_omega.set_xlabel('Time [s]')
        self.ax_omega.set_ylabel('ω [rad/s]')
        self.ax_omega.grid(True)
        
        # Steering angles plot
        self.ax_steering.set_title('Steering Angles')
        self.ax_steering.set_xlabel('Time [s]')
        self.ax_steering.set_ylabel('δ [rad]')
        self.ax_steering.grid(True)
        
        # Wheel velocities plot
        self.ax_wheel_v.set_title('Wheel Angular Velocities')
        self.ax_wheel_v.set_xlabel('Time [s]')
        self.ax_wheel_v.set_ylabel('ω [rad/s]')
        self.ax_wheel_v.grid(True)
        
    def _draw_vehicle(self, vehicle, map_data=None):
        """Draw vehicle chassis and wheels."""
        self.ax_main.clear()
        
        # Draw occupancy grid if provided
        if map_data is not None:
            if -1 in map_data:  # Handle unknown cells
                grid_normalized = (map_data + 1) / 2
                self.ax_main.imshow(grid_normalized.T, cmap='gray_r', 
                                  origin='lower', vmin=0, vmax=1)
            else:
                self.ax_main.imshow(map_data.T, cmap='gray_r', origin='lower')
        
        # Get vehicle dimensions
        L = vehicle.L  # wheelbase
        T = vehicle.T  # track width
        r = vehicle.r_wheel  # wheel radius
        
        # Draw chassis
        chassis = Rectangle(
            (vehicle.x - L/2, vehicle.y - T/2),
            L, T,
            angle=np.degrees(vehicle.theta),
            color=self.vehicle_color,
            alpha=0.5
        )
        self.ax_main.add_patch(chassis)
        
        # Draw wheels
        wheel_positions = vehicle.get_wheel_positions()
        wheel_width = 0.04
        wheel_length = 0.08
        
        for pos_key, (wx, wy, wtheta) in wheel_positions.items():
            wheel = Rectangle(
                (wx - wheel_length/2, wy - wheel_width/2),
                wheel_length, wheel_width,
                angle=np.degrees(wtheta),
                color=self.wheel_color
            )
            self.ax_main.add_patch(wheel)
        
        # Plot trajectory
        if len(vehicle.history['x']) > 0:
            self.ax_main.plot(vehicle.history['x'], 
                            vehicle.history['y'],
                            color=self.trail_color,
                            linestyle='--',
                            label='Trajectory')
        
        self.ax_main.set_title('Vehicle Motion')
        self.ax_main.grid(True)
        
    def _update_plots(self, vehicle):
        """Update motion plots with latest data."""
        # Clear previous plots
        self.ax_v.clear()
        self.ax_omega.clear()
        self.ax_steering.clear()
        self.ax_wheel_v.clear()
        self._setup_plot_styles()
        
        # Plot velocity
        self.ax_v.plot(vehicle.history['time'],
                      vehicle.history['v'])
        
        # Plot angular velocity
        self.ax_omega.plot(vehicle.history['time'],
                          vehicle.history['omega'])
        
        # Plot steering angles
        self.ax_steering.plot(vehicle.history['time'],
                            vehicle.history['delta_left'],
                            label='Left')
        self.ax_steering.plot(vehicle.history['time'],
                            vehicle.history['delta_right'],
                            label='Right')
        self.ax_steering.legend()
        
        # Plot wheel velocities
        self.ax_wheel_v.plot(vehicle.history['time'],
                            vehicle.history['omega_LF'],
                            label='LF')
        self.ax_wheel_v.plot(vehicle.history['time'],
                            vehicle.history['omega_RF'],
                            label='RF')
        self.ax_wheel_v.plot(vehicle.history['time'],
                            vehicle.history['omega_LR'],
                            label='LR')
        self.ax_wheel_v.plot(vehicle.history['time'],
                            vehicle.history['omega_RR'],
                            label='RR')
        self.ax_wheel_v.legend()
        
    def update(self, vehicle, map_data=None, path=None, frontiers=None):
        """Update visualization with latest vehicle state and map data."""
        self._draw_vehicle(vehicle, map_data)
        
        # Draw path if provided
        if path:
            px, py = zip(*path)
            self.ax_main.plot(px, py, 'b--', label='Planned Path')
        
        # Draw frontiers if provided
        if frontiers:
            fx = [f[0] for f in frontiers]
            fy = [f[1] for f in frontiers]
            self.ax_main.scatter(fx, fy, c='magenta', marker='*',
                               label='Frontiers', s=100)
        
        self.ax_main.legend()
        
        # Update motion plots
        self._update_plots(vehicle)
        
        # Adjust layout and draw
        self.fig.tight_layout()
        plt.pause(0.01)
        
    def save_plots(self, prefix='vehicle_motion'):
        """Save all plots as images."""
        self.fig.savefig(f'{prefix}_combined.png')
        
        # Create separate figures for each plot
        plt.figure(figsize=(8, 6))
        plt.plot(self.ax_v.lines[0].get_xdata(),
                self.ax_v.lines[0].get_ydata())
        plt.title('Linear Velocity')
        plt.xlabel('Time [s]')
        plt.ylabel('v [m/s]')
        plt.grid(True)
        plt.savefig(f'{prefix}_velocity.png')
        
        plt.figure(figsize=(8, 6))
        plt.plot(self.ax_omega.lines[0].get_xdata(),
                self.ax_omega.lines[0].get_ydata())
        plt.title('Angular Velocity')
        plt.xlabel('Time [s]')
        plt.ylabel('ω [rad/s]')
        plt.grid(True)
        plt.savefig(f'{prefix}_angular_velocity.png')
        
        plt.figure(figsize=(8, 6))
        for line in self.ax_wheel_v.lines:
            plt.plot(line.get_xdata(), line.get_ydata(),
                    label=line.get_label())
        plt.title('Wheel Angular Velocities')
        plt.xlabel('Time [s]')
        plt.ylabel('ω [rad/s]')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'{prefix}_wheel_velocities.png')
        
        plt.close('all')