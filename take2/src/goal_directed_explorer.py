"""
Goal-Directed Autonomous Exploration
Fast, smooth Ackermann navigation with per-wheel dynamics visualization.
"""

import numpy as np
import time
import matplotlib.pyplot as plt
from typing import Optional, List, Tuple
from matplotlib.patches import Rectangle
import matplotlib.transforms as mtransforms
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Circle


# --- Local imports ---
from src.robot.ackermann import AckermannRobot
from src.sensors.lidar import LidarSensor
from src.map.occupancy_grid import OccupancyGrid
from src.planning.astar import AStarPlanner
from src.planning.frontier_explorer import FrontierExplorer
from src.control.velocity_controller import PathFollowingController, TrapezoidalVelocityController
from src.objects.generator import generate_environment
class Simple3DRenderer:
    def __init__(self, ax3d):
        self.ax = ax3d
        self.car_box = None
        self.goal_marker = None
        self.obstacles = []

    def draw_environment(self, obstacles, goal):
        for obs in obstacles:
            x, y = getattr(obs, "x", 0), getattr(obs, "y", 0)
            h = getattr(obs, "height", 1.0)
            color = getattr(obs, "color", (0.4, 0.3, 0.2))
            if hasattr(obs, "radius"):  # cylinder-like
                theta = np.linspace(0, 2*np.pi, 20)
                X = obs.radius*np.cos(theta) + x
                Y = obs.radius*np.sin(theta) + y
                Z = np.zeros_like(theta)
                self.ax.plot_surface(
                    np.vstack([X, X]),
                    np.vstack([Y, Y]),
                    np.array([[0]*20, [h]*20]),
                    color=color, alpha=0.6, linewidth=0
                )
            else:  # box-like
                L, W = getattr(obs, "width", 1.0), getattr(obs, "length", 1.0)
                xx = [x-L/2, x+L/2]
                yy = [y-W/2, y+W/2]
                X, Y = np.meshgrid(xx, yy)
                self.ax.plot_surface(X, Y, np.zeros_like(X), color=color, alpha=0.6)
                self.ax.plot_surface(X, Y, np.full_like(X, h), color=color, alpha=0.6)
        gx, gy = goal
        self.goal_marker = self.ax.scatter(gx, gy, 0.1, color="red", s=80, marker="*")

    def update_car(self, x, y, theta):
        L, W, H = 0.6, 0.3, 0.2
        verts = np.array([
            [-L/2, -W/2, 0],
            [ L/2, -W/2, 0],
            [ L/2,  W/2, 0],
            [-L/2,  W/2, 0],
            [-L/2, -W/2, H],
            [ L/2, -W/2, H],
            [ L/2,  W/2, H],
            [-L/2,  W/2, H],
        ])
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1],
        ])
        verts = verts @ R.T + np.array([x, y, 0])
        faces = [
            [verts[i] for i in [0,1,2,3]],
            [verts[i] for i in [4,5,6,7]],
            [verts[i] for i in [0,1,5,4]],
            [verts[i] for i in [1,2,6,5]],
            [verts[i] for i in [2,3,7,6]],
            [verts[i] for i in [3,0,4,7]],
        ]
        if self.car_box:
            self.car_box.remove()
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        self.car_box = Poly3DCollection(faces, facecolor='skyblue', edgecolor='k', alpha=0.9)
        self.ax.add_collection3d(self.car_box)

        # Camera follows behind vehicle
        cam_offset = np.array([-3*np.cos(theta), -3*np.sin(theta), 2])
        cam_target = np.array([x, y, 0])
        cam_pos = cam_target + cam_offset
        self.ax.set_xlim(x - 5, x + 5)
        self.ax.set_ylim(y - 5, y + 5)
        self.ax.set_zlim(0, 3)
        self.ax.view_init(elev=25, azim=-np.degrees(theta) + 90)

class VehiclePatch:
    """Draws a simple rectangular Ackermann car with turning wheels."""

    def __init__(self, ax, length=0.6, width=0.3, wheel_w=0.08, wheel_l=0.15):
        self.ax = ax
        self.length = length
        self.width = width
        self.wheel_w = wheel_w
        self.wheel_l = wheel_l

        # Car body
        self.body = Rectangle((-length/2, -width/2), length, width,
                              facecolor='skyblue', edgecolor='navy', lw=1.5, zorder=5)
        ax.add_patch(self.body)

        # Four wheels (front-left, front-right, rear-left, rear-right)
        self.wheels = []
        offsets = [
            (-length/2 + 0.05,  width/2 - wheel_w/2),   # rear-left
            (-length/2 + 0.05, -width/2 - wheel_w/2),   # rear-right
            ( length/2 - wheel_l - 0.05,  width/2 - wheel_w/2),  # front-left
            ( length/2 - wheel_l - 0.05, -width/2 - wheel_w/2)   # front-right
        ]
        for (ox, oy) in offsets:
            wheel = Rectangle((ox, oy), wheel_l, wheel_w,
                              facecolor='black', edgecolor='gray', zorder=6)
            ax.add_patch(wheel)
            self.wheels.append(wheel)

    def update(self, x, y, theta, steer_angle):
        """Update car pose and wheel steering in world coordinates."""
        # Base transform (vehicle rotation + translation)
        t_body = mtransforms.Affine2D().rotate(theta).translate(x, y)
        self.body.set_transform(t_body + self.ax.transData)

        # Rear wheels (no steering)
        for i in [0, 1]:
            wheel = self.wheels[i]
            t = mtransforms.Affine2D().rotate(theta).translate(x, y)
            wheel.set_transform(t + self.ax.transData)

        # Front wheels (add steering rotation)
        for i in [2, 3]:
            wheel = self.wheels[i]
            t = mtransforms.Affine2D().rotate(theta + steer_angle).translate(x, y)
            wheel.set_transform(t + self.ax.transData)


class GoalDirectedExplorer:
    def __init__(self, start_pos=None, goal_pos=None, map_size=(20, 20), resolution=0.1):
        # --- Random environment ---
        obstacles = generate_environment(num_obs=6, bounds=map_size,
                                         min_clearance=0.5, edge_buffer=0.3, rng_seed=42)
        if start_pos is None:
            start_pos = generate_environment.last_start
        if goal_pos is None:
            goal_pos = generate_environment.last_goal

        self.start_pos, self.goal_pos = start_pos, goal_pos
        self.map_size, self.resolution = map_size, resolution
        self.obstacles = obstacles

        print("=" * 70)
        print("GOAL-DIRECTED AUTONOMOUS EXPLORATION SYSTEM (v2.2)")
        print("=" * 70)
        print(f"Start: {self.start_pos}  Goal: {self.goal_pos}")

        # --- Core modules ---
        self.robot = AckermannRobot(x=start_pos[0], y=start_pos[1], theta=0,
                                    wheelbase=0.3, dt=0.02)
        self.lidar = LidarSensor(fov=270, num_beams=108, max_range=6.0)
        self.occupancy_grid = OccupancyGrid(map_size=map_size, resolution=resolution)
        self.path_planner = AStarPlanner(self.occupancy_grid)
        self.frontier_explorer = FrontierExplorer(self.occupancy_grid)

        velocity_controller = TrapezoidalVelocityController(
            max_velocity=2.5,
            max_acceleration=2.0,
            max_angular_velocity=2.5
        )
        self.path_controller = PathFollowingController(
            lookahead_distance=1.2,
            velocity_controller=velocity_controller
        )

        # --- Visualization ---
        self._setup_visualization()

        # --- State ---
        self.current_path = None
        self.current_goal = None
        self.goal_reached = False
        self.frontier_display = []
        self.frontiers_visited = []
        self.visited_positions = []
        self.total_distance = 0.0
        self.best_distance_to_goal = float("inf")
        self.iterations_without_progress = 0
        self.inflation_radius_cells = int(round(0.4 / resolution))
        self.start_time = time.time()

        # --- Logging arrays ---
        self.time_log = []
        self.vel_log = []
        self.ang_log = []
        self.fl_log, self.fr_log, self.rl_log, self.rr_log = [], [], [], []

        print(f"✓ Initialization complete (Inflation radius: {self.inflation_radius_cells} cells)")

    # ================================================================
    #  Visualization setup
    # ================================================================
    def _setup_visualization(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3D projection)

        plt.ion()
        self.fig = plt.figure(figsize=(14, 9))
        gs = self.fig.add_gridspec(
            3, 2,
            width_ratios=[1.4, 1.0],
            height_ratios=[1, 1, 1]
        )

        # Left (full height) — 2D exploration
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self.ax_map.set_xlim(0, self.map_size[0])
        self.ax_map.set_ylim(0, self.map_size[1])
        self.ax_map.set_aspect("equal")
        self.ax_map.set_title("2D Exploration Map — Frontiers, Robot, Goal")

        # Robot rectangle + wheels
        self.vehicle_patch = VehiclePatch(self.ax_map)

        # Top-right — linear/angular velocity
        self.ax_vel = self.fig.add_subplot(gs[0, 1])
        self.ax_vel.set_title("Linear / Angular Velocity")
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity (m/s, rad/s)")

        # Mid-right — per-wheel dynamics
        self.ax_wheel = self.fig.add_subplot(gs[1, 1])
        self.ax_wheel.set_title("Per-Wheel Velocity")
        self.ax_wheel.set_xlabel("Time (s)")
        self.ax_wheel.set_ylabel("Velocity (m/s)")

        # Bottom-right — 3D environment view (follow-cam)
        self.ax_3d = self.fig.add_subplot(gs[2, 1], projection="3d")
        self.ax_3d.set_title("3D Follow-Camera View")
        self.ax_3d.set_box_aspect([1, 1, 0.5])

        # Create 3D renderer and draw static scene
        self.renderer3d = Simple3DRenderer(self.ax_3d)
        self.renderer3d.draw_environment(self.obstacles, self.goal_pos)

        # Back-compat alias
        self.ax_2d = self.ax_map
    def _update_visualization(self):
        """Update exploration map, dynamics plots, and 3D follow-cam in real time."""
        import numpy as np
        import matplotlib.pyplot as plt

        # ---------------- Map panel (no full clear) ----------------
        grid = self.occupancy_grid.grid

        # colorize occupancy once; then just set_data()
        def _colorize(g):
            colored = np.zeros((*g.shape, 3))
            unknown = (g >= 0.35) & (g <= 0.65)
            free = g < 0.35
            occ = g > 0.65
            colored[unknown] = [0.7, 0.7, 0.7]
            colored[free]    = [0.8, 0.9, 1.0]
            colored[occ]     = [0.2, 0.1, 0.1]
            return colored

        if not hasattr(self, "map_img"):
            self.map_img = self.ax_map.imshow(
                _colorize(grid),
                origin="lower",
                extent=[0, self.map_size[0], 0, self.map_size[1]]
            )
        else:
            self.map_img.set_data(_colorize(grid))

        # trajectory & anchors
        traj = np.array(self.visited_positions)
        if len(traj) > 1:
            self.ax_map.plot(traj[:, 0], traj[:, 1], "b-", lw=1.5, label="Trajectory")
        self.ax_map.scatter(*self.start_pos, c="green", s=80, label="Start")
        self.ax_map.scatter(*self.goal_pos,  c="red",   s=90, marker="X", label="Goal")

        # live car patch (rectangle + steering wheels)
        steer_angle = self.ang_log[-1] if self.ang_log else 0.0
        current_v   = self.vel_log[-1] if self.vel_log else 0.0
        # If your VehiclePatch.update supports v (for spinning wheels), pass v=current_v
        try:
            self.vehicle_patch.update(self.robot.x, self.robot.y, self.robot.theta, steer_angle, v=current_v)
        except TypeError:
            # fallback to signature without v
            self.vehicle_patch.update(self.robot.x, self.robot.y, self.robot.theta, steer_angle)

        # frontiers & selected frontier
        if self.frontier_display:
            fx, fy = zip(*self.frontier_display)
            self.ax_map.scatter(fx, fy, s=25, color="lime", alpha=0.8, label="Frontiers")
        if self.current_goal:
            self.ax_map.scatter(
                self.current_goal[0], self.current_goal[1],
                marker="*", s=160, color="gold", edgecolor="k", label="Selected Frontier"
            )

        self.ax_map.set_xlim(0, self.map_size[0])
        self.ax_map.set_ylim(0, self.map_size[1])
        self.ax_map.set_title("Exploration Map — Robot, Frontiers, Goal")

        # deduplicate legend entries
        handles, labels = self.ax_map.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        self.ax_map.legend(by_label.values(), by_label.keys(), loc="upper right")

        # ---------------- Velocity panel ----------------
        self.ax_vel.clear()
        if self.time_log:
            self.ax_vel.plot(self.time_log, self.vel_log, "b-", lw=1.2, label="Linear Velocity v (m/s)")
            self.ax_vel.plot(self.time_log, self.ang_log, "r--", lw=1.0, label="Steering Angle δ (rad)")
            self.ax_vel.grid(True)
            self.ax_vel.set_xlabel("Time (s)")
            self.ax_vel.set_ylabel("Velocity (m/s) / Angle (rad)")
            self.ax_vel.legend()
            self.ax_vel.set_title(
                f"Linear / Angular Velocity — v={self.vel_log[-1]:.2f} m/s, δ={np.degrees(self.ang_log[-1]):.1f}°"
            )

        # ---------------- Wheel dynamics panel ----------------
        self.ax_wheel.clear()
        if self.time_log:
            self.ax_wheel.plot(self.time_log, self.fl_log, "r-", label="Front Left (FL)")
            self.ax_wheel.plot(self.time_log, self.fr_log, "m-", label="Front Right (FR)")
            self.ax_wheel.plot(self.time_log, self.rl_log, "g-", label="Rear Left (RL)")
            self.ax_wheel.plot(self.time_log, self.rr_log, "b-", label="Rear Right (RR)")
            self.ax_wheel.grid(True)
            self.ax_wheel.set_xlabel("Time (s)")
            self.ax_wheel.set_ylabel("Wheel Linear Velocity (m/s)")
            self.ax_wheel.legend()
            avg_speed = np.mean([self.fl_log[-1], self.fr_log[-1], self.rl_log[-1], self.rr_log[-1]])
            self.ax_wheel.set_title(f"Per-Wheel Dynamics — Avg={avg_speed:.2f} m/s")

        # ---------------- 3D follow camera ----------------
        self.renderer3d.update_car(self.robot.x, self.robot.y, self.robot.theta)

        # ---------------- draw frame ----------------
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.0001)

    # ================================================================
    #  Exploration loop
    # ================================================================
    def run_exploration(self, max_iterations=10000):
        self._sense_and_map()
        self._update_visualization()

        print("\n🚀 Starting Exploration Loop...")
        iteration = 0
        while iteration < max_iterations and not self.goal_reached:
            iteration += 1

            self._sense_and_map()

            if self.current_goal is None or self._is_goal_reached():
                self._plan_next_goal()

            if self.current_path:
                self._follow_path()

            self._monitor_progress(iteration)

            if iteration % 5 == 0:
                self._update_visualization()

            if self._is_at_final_goal():
                print("\n🎯 FINAL GOAL REACHED!")
                self.goal_reached = True
                break

        self._print_final_stats(iteration)
        plt.ioff()
        plt.show()

    # ================================================================
    #  Core subroutines
    # ================================================================
    def _sense_and_map(self):
        ranges = self.lidar.scan(self.robot, self.obstacles)
        self.occupancy_grid.update_from_lidar(
            self.robot.x, self.robot.y, self.robot.theta, ranges, self.lidar.angles
        )
        self.visited_positions.append((self.robot.x, self.robot.y))

    def _plan_next_goal(self):
        all_frontiers = self.frontier_explorer.find_frontiers()
        self.frontier_display = all_frontiers
        if not all_frontiers:
            print("⚠️ No frontiers — attempting direct goal path.")
            path = self.path_planner.plan(
                self.robot.x, self.robot.y, self.goal_pos[0], self.goal_pos[1],
                inflation_radius=self.inflation_radius_cells
            )
            if path:
                self.current_goal, self.current_path = self.goal_pos, path
                print(f"🎯 Direct path to goal found ({len(path)} pts)")
            return

        reachable = []
        for fx, fy in all_frontiers:
            gx, gy = self.occupancy_grid.world_to_grid(fx, fy)
            if not self.occupancy_grid.is_free(gx, gy):
                continue
            path = self.path_planner.plan(
                self.robot.x, self.robot.y, fx, fy, inflation_radius=self.inflation_radius_cells
            )
            if path:
                reachable.append((fx, fy, path))

        if not reachable:
            return

        min_frontier_dist = min(np.hypot(fx - self.goal_pos[0], fy - self.goal_pos[1]) for fx, fy, _ in reachable)
        goal_path = self.path_planner.plan(
            self.robot.x, self.robot.y, self.goal_pos[0], self.goal_pos[1],
            inflation_radius=self.inflation_radius_cells
        )
        if goal_path and min_frontier_dist < 2.0:
            self.current_goal, self.current_path = self.goal_pos, goal_path
            print(f"🎯 Goal near explored area (dist={min_frontier_dist:.2f}) → going directly to goal.")
            return

        best_score, best_frontier, best_path = float("inf"), None, None
        for fx, fy, path in reachable:
            dist_goal = np.hypot(self.goal_pos[0] - fx, self.goal_pos[1] - fy)
            info = self.frontier_explorer._calculate_information_gain(fx, fy)
            score = dist_goal - 0.02 * info
            if score < best_score:
                best_score, best_frontier, best_path = score, (fx, fy), path
        self.current_goal, self.current_path = best_frontier, best_path
        print(f"🧭 Selected frontier {best_frontier} (path {len(best_path)} pts)")
        self._update_visualization()

    def _follow_path(self):
        if not self.current_path or len(self.current_path) < 2:
            return

        # --- Control ---
        v, delta = self.path_controller.compute_control(
            self.robot.x, self.robot.y, self.robot.theta,
            self.current_path, self.robot.dt
        )

        # --- Ackermann per-wheel dynamics ---
        def wheel_speeds(v, delta, L=0.3, track=0.25):
            """
            Compute per-wheel linear velocities for Ackermann vehicle.
            Returns (FL, FR, RL, RR) in m/s.
            """
            if abs(delta) < 1e-6:  # going straight
                return v, v, v, v

            # Instantaneous center of curvature (ICC)
            R_center = L / np.tan(delta)
            omega = v / L * np.tan(delta)  # rotational velocity around ICC

            # Radii to each wheel
            R_rear_inner = abs(R_center) - track / 2
            R_rear_outer = abs(R_center) + track / 2
            R_front_inner = np.hypot(L, R_rear_inner)
            R_front_outer = np.hypot(L, R_rear_outer)

            # Linear velocities
            v_rear_inner = omega * R_rear_inner
            v_rear_outer = omega * R_rear_outer
            v_front_inner = omega * R_front_inner
            v_front_outer = omega * R_front_outer

            # Assign depending on turn direction
            if delta > 0:   # turning left
                return v_front_inner, v_front_outer, v_rear_inner, v_rear_outer
            else:           # turning right
                return v_front_outer, v_front_inner, v_rear_outer, v_rear_inner


        FL, FR, RL, RR = wheel_speeds(
            v, delta,
            getattr(self.robot, "L", getattr(self.robot, "wheelbase", 0.3)),
            0.25
        )

        t = time.time() - self.start_time
        self.time_log.append(t)
        self.vel_log.append(v)
        self.ang_log.append(delta)
        self.fl_log.append(FL)
        self.fr_log.append(FR)
        self.rl_log.append(RL)
        self.rr_log.append(RR)

        # --- Move ---
        prev_x, prev_y = self.robot.x, self.robot.y
        self.robot.step(v, delta)
        self.total_distance += np.hypot(self.robot.x - prev_x, self.robot.y - prev_y)

    # ================================================================
    #  Progress / stats
    # ================================================================
    def _monitor_progress(self, iteration):
        dist = np.hypot(self.robot.x - self.goal_pos[0], self.robot.y - self.goal_pos[1])
        if dist < self.best_distance_to_goal - 0.5:
            self.best_distance_to_goal = dist
            self.iterations_without_progress = 0
        else:
            self.iterations_without_progress += 1
        if self.iterations_without_progress > 300:
            print("⚠️ Stuck → resetting goal.")
            self.current_goal = None
            self.iterations_without_progress = 0

    def _is_goal_reached(self):
        if self.current_goal is None:
            return True
        return self.path_controller.is_goal_reached(
            self.robot.x, self.robot.y, self.current_goal[0], self.current_goal[1], tolerance=0.8
        )

    def _is_at_final_goal(self):
        tol = max(0.8, 0.1 * max(self.map_size))
        dist = np.hypot(self.robot.x - self.goal_pos[0], self.robot.y - self.goal_pos[1])
        if dist < tol:
            print(f"🏁 Within goal tolerance ({dist:.2f} m < {tol:.2f})")
            return True
        return False

    def _print_final_stats(self, iterations):
        elapsed = time.time() - self.start_time
        total_cells = self.occupancy_grid.width * self.occupancy_grid.height
        known_cells = np.sum((self.occupancy_grid.grid < 0.35) | (self.occupancy_grid.grid > 0.65))
        ratio = known_cells / total_cells
        print("\n" + "=" * 70)
        print("EXPLORATION SUMMARY")
        print("=" * 70)
        print(f"Goal Reached: {'YES ✓' if self.goal_reached else 'NO ✗'}")
        print(f"Iterations: {iterations}")
        print(f"Distance: {self.total_distance:.2f} m")
        print(f"Explored Map: {ratio * 100:.1f}%")
        print(f"Elapsed: {elapsed:.1f}s | Avg Speed: {self.total_distance / elapsed:.2f} m/s")
        print("=" * 70)


# ================================================================
# Auto dependency check and environment bootstrap
# ================================================================
import importlib
import subprocess
import sys
import os
import venv

REQUIRED_PACKAGES = ["numpy", "matplotlib", "scipy"]

def ensure_dependencies():
    missing = []
    for pkg in REQUIRED_PACKAGES:
        try:
            importlib.import_module(pkg)
        except ImportError:
            missing.append(pkg)

    if not missing:
        print("✅ All dependencies satisfied.")
        return

    print(f"⚙️  Installing missing dependencies: {missing}")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", *missing])
        print("✅ Installed successfully.")
        return
    except Exception as e:
        print(f"❌ Direct install failed: {e}")
        print("Creating virtual environment...")
        venv_dir = "env"
        venv.EnvBuilder(with_pip=True).create(venv_dir)
        pip_exe = os.path.join(venv_dir, "bin", "pip")
        python_exe = os.path.join(venv_dir, "bin", "python")
        subprocess.check_call([pip_exe, "install", *missing])
        print("✅ Dependencies installed in virtual environment.")
        print(f"To activate: source {venv_dir}/bin/activate")
        sys.exit(0)



# ================================================================
#  Entry point
# ================================================================
def main():
    ensure_dependencies()
    explorer = GoalDirectedExplorer(start_pos=None, goal_pos=None,
                                    map_size=(10, 10), resolution=0.1)
    explorer.run_exploration(max_iterations=15000)


if __name__ == "__main__":
    main()
