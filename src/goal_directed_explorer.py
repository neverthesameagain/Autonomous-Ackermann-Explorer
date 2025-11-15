import json
import os
import time
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import numpy as np
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection



from src.robot.ackermann import AckermannRobot
from src.sensors.lidar import LidarSensor
from src.map.occupancy_grid import OccupancyGrid
from src.planning.astar import AStarPlanner
from src.planning.frontier_explorer import FrontierExplorer
from src.control.velocity_controller import (
    TrapezoidalVelocityController,
    PathFollowingController
)
from src.objects.primitives import Box, Cylinder
from src.env.Simple3drenderer import Simple3DRenderer

class VehiclePatch:
    def __init__(self, ax, length=0.6, width=0.3, wheel_w=0.08, wheel_l=0.15):
        self.ax = ax
        self.length = length
        self.width = width
        self.wheel_w = wheel_w
        self.wheel_l = wheel_l

        # body
        self.body = Rectangle((-length/2, -width/2), length, width,
                              facecolor='skyblue', edgecolor='navy', lw=1.5, zorder=5)
        ax.add_patch(self.body)

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
        # Base transform (vehicle rotation + translation)
        t_body = mtransforms.Affine2D().rotate(theta).translate(x, y)
        self.body.set_transform(t_body + self.ax.transData)

        # Rear wheels (no steering)
        for i in [0, 1]:
            wheel = self.wheels[i]
            t = mtransforms.Affine2D().rotate(theta).translate(x, y)
            wheel.set_transform(t + self.ax.transData)
        # Front wheels ( steering rotation)
        for i in [2, 3]:
            wheel = self.wheels[i]
            t = mtransforms.Affine2D().rotate(theta + steer_angle).translate(x, y)
            wheel.set_transform(t + self.ax.transData)


class GoalDirectedExplorer:
    def __init__(self, start_pos=None, goal_pos=None, map_size=(20, 20), resolution=0.1,
                 use_saved_environment: bool = False):
        obstacles, default_start, default_goal = self._load_or_generate_environment(
            use_saved=use_saved_environment,
            map_size=map_size,
            resolution=resolution,
            start_override=start_pos,
            goal_override=goal_pos,
        )
        if start_pos is None:
            start_pos = default_start
        if goal_pos is None:
            goal_pos = default_goal

        self.start_pos, self.goal_pos = start_pos, goal_pos
        self.map_size, self.resolution = map_size, resolution
        self.obstacles = obstacles

        print("=" * 70)

        print(f"Start: {self.start_pos}  Goal: {self.goal_pos}")
        self._persist_environment()

        self.robot = AckermannRobot(x=start_pos[0], y=start_pos[1], theta=0,
                                    wheelbase=0.3, dt=0.02)
        self.lidar = LidarSensor(fov=270, num_beams=108, max_range=6.0)
        self.occupancy_grid = OccupancyGrid(map_size=map_size, resolution=resolution)
        self.path_planner = AStarPlanner(self.occupancy_grid)
        self.frontier_explorer = FrontierExplorer(self.occupancy_grid)

        velocity_controller = TrapezoidalVelocityController(
            max_velocity=2.5,
            max_acceleration=3,
            max_angular_velocity=2.5
        )
        self.path_controller = PathFollowingController(
            lookahead_distance=1.5,
            velocity_controller=velocity_controller,
            wheelbase=self.robot.wheelbase,
            min_lookahead=0.35,
            max_lookahead=1.8
        )

        # --- Visualization ---
        self._setup_visualization()

        # --- State ---
        self.current_path = None
        self.current_goal = None
        self.goal_reached = False
        self.frontier_display = []
        self.visited_positions = []
        self.total_distance = 0.0
        self.best_distance_to_goal = float("inf")
        self.iterations_without_progress = 0
        self.inflation_radius_cells = int(round(0.25 / resolution))
        self.start_time = time.time()
        self.inflated_mask = np.zeros_like(self.occupancy_grid.grid, dtype=bool)
        self.iteration_count = 0
        self.stuck_counter = 0
        self.max_stuck_iterations = 120
        self.obstacle_overlay = None
        self.inflation_overlay = None
        self.blocked_frontiers = []
        self.traj_line = None
        self.frontier_scatter = None
        self.goal_selection_scatter = None
        self.start_marker = None
        self.goal_marker_2d = None
        self.legend_initialized = False
        self.goal_tolerance = max(0.15, 1.5 * resolution)
        self.final_goal_tolerance = max(0.25, 2.5 * resolution)

        # --- Logging arrays ---
        self.time_log = []
        self.vel_log = []
        self.yaw_log = []
        self.steer_log = []
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
        self.renderer3d = Simple3DRenderer(self.ax_3d, self.map_size)
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

        colored_grid = _colorize(grid)
        if not hasattr(self, "map_img"):
            self.map_img = self.ax_map.imshow(
                colored_grid,
                origin="lower",
                extent=[0, self.map_size[0], 0, self.map_size[1]]
            )
        else:
            self.map_img.set_data(colored_grid)

        # overlay actual occupied cells and inflated safety margin in distinct colors
        occ_rgba = np.zeros((*grid.shape, 4))
        occ_rgba[grid > 0.65] = [0.5, 0.0, 0.0, 0.75]
        inflated_only = np.logical_and(self.inflated_mask, grid <= 0.65)
        infl_rgba = np.zeros((*grid.shape, 4))
        infl_rgba[inflated_only] = [1.0, 0.6, 0.0, 0.25]

        if self.obstacle_overlay is None:
            self.obstacle_overlay = self.ax_map.imshow(
                occ_rgba, origin="lower",
                extent=[0, self.map_size[0], 0, self.map_size[1]],
                zorder=3
            )
        else:
            self.obstacle_overlay.set_data(occ_rgba)

        if self.inflation_overlay is None:
            self.inflation_overlay = self.ax_map.imshow(
                infl_rgba, origin="lower",
                extent=[0, self.map_size[0], 0, self.map_size[1]],
                zorder=2
            )
        else:
            self.inflation_overlay.set_data(infl_rgba)

        # trajectory & anchors
        traj = np.array(self.visited_positions)
        if self.traj_line is None:
            (self.traj_line,) = self.ax_map.plot([], [], "b-", lw=1.5, label="Trajectory")
        if len(traj) > 1:
            self.traj_line.set_data(traj[:, 0], traj[:, 1])
        else:
            self.traj_line.set_data([], [])

        if self.start_marker is None:
            self.start_marker = self.ax_map.scatter(
                self.start_pos[0], self.start_pos[1], c="green", s=80, label="Start", zorder=6
            )
        if self.goal_marker_2d is None:
            self.goal_marker_2d = self.ax_map.scatter(
                self.goal_pos[0], self.goal_pos[1], c="red", s=90, marker="X", label="Goal", zorder=6
            )

        # live car patch (rectangle + steering wheels)
        steer_angle = self.steer_log[-1] if self.steer_log else 0.0
        current_v   = self.vel_log[-1] if self.vel_log else 0.0
        try:
            self.vehicle_patch.update(self.robot.x, self.robot.y, self.robot.theta, steer_angle, v=current_v)
        except TypeError:
            # fallback to signature without v
            self.vehicle_patch.update(self.robot.x, self.robot.y, self.robot.theta, steer_angle)

        # frontiers & selected frontier
        if self.frontier_scatter is None:
            self.frontier_scatter = self.ax_map.scatter(
                [], [], s=25, color="lime", alpha=0.8, label="Frontiers", zorder=7
            ) 
        if self.frontier_display:
            offsets = np.array(self.frontier_display)
        else:
            offsets = np.empty((0, 2))
        self.frontier_scatter.set_offsets(offsets)

        if self.goal_selection_scatter is None:
            self.goal_selection_scatter = self.ax_map.scatter(
                [], [], marker="*", s=160, color="gold", edgecolor="k",
                label="Selected Frontier", zorder=8
            )
        if self.current_goal:
            self.goal_selection_scatter.set_offsets(
                np.array([[self.current_goal[0], self.current_goal[1]]])
            )
        else:
            self.goal_selection_scatter.set_offsets(np.empty((0, 2)))

        self.ax_map.set_xlim(0, self.map_size[0])
        self.ax_map.set_ylim(0, self.map_size[1])
        self.ax_map.set_title("Exploration Map — Robot, Frontiers, Goal")

        # deduplicate legend entries
        if not self.legend_initialized:
            handles, labels = self.ax_map.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            self.ax_map.legend(by_label.values(), by_label.keys(), loc="upper right")
            self.legend_initialized = True

        # ---------------- Velocity panel ----------------
        self.ax_vel.clear()
        if self.time_log:
            self.ax_vel.plot(self.time_log, self.vel_log, "b-", lw=1.2, label="Linear Velocity v (m/s)")
            self.ax_vel.plot(self.time_log, self.yaw_log, "r--", lw=1.0, label="Yaw Rate ω (rad/s)")
            self.ax_vel.grid(True)
            self.ax_vel.set_xlabel("Time (s)")
            self.ax_vel.set_ylabel("Velocity (m/s) / Angular Rate (rad/s)")
            self.ax_vel.legend()
            self.ax_vel.set_title(
                f"Linear / Angular Velocity — v={self.vel_log[-1]:.2f} m/s, ω={self.yaw_log[-1]:.2f} rad/s"
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
        if len(traj) > 0:
            self.renderer3d.update_path(traj)
        else:
            self.renderer3d.update_path([])
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

        print("\n Starting Exploration Loop...")
        iteration = 0
        while iteration < max_iterations and not self.goal_reached:
            iteration += 1
            self.iteration_count = iteration

            self._sense_and_map()

            needs_replan = False

            if (self.current_goal is not None
                    and not self._matches_final_goal(self.current_goal)):
                gx, gy = self.occupancy_grid.world_to_grid(
                    self.current_goal[0], self.current_goal[1]
                )
                if self._within_inflation_buffer(gx, gy):
                    print("⚠️ Current frontier now lies inside inflated safety buffer — replanning.")
                    self._mark_frontier_blocked(self.current_goal)
                    self.current_goal = None
                    self.current_path = None
                    self.path_controller.reset()
                    needs_replan = True

            if self._path_is_blocked():
                print("⚠️ Current path obstructed by newly detected obstacle — replanning.")
                if self.current_goal is not None and not self._matches_final_goal(self.current_goal):
                    self._mark_frontier_blocked(self.current_goal)
                self.current_goal = None
                self.current_path = None
                needs_replan = True

            if self.current_goal is None or self._is_goal_reached() or needs_replan:
                self._plan_next_goal()

            if self.current_path:
                self._follow_path()
            else:
                self.path_controller.reset()

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
    #  Core 
    # ================================================================
    def _sense_and_map(self):
        ranges = self.lidar.scan(self.robot, self.obstacles)
        self.occupancy_grid.update_from_lidar(
            self.robot.x, self.robot.y, self.robot.theta, ranges, self.lidar.angles
        )
        self.visited_positions.append((self.robot.x, self.robot.y))

    def _plan_next_goal(self):
        """Select either the final goal or a new frontier to chase."""
        self.frontier_display = self.frontier_explorer.find_frontiers()

        # If the final goal has already been uncovered, try to go straight there.
        if self._goal_in_known_space():
            path = self._plan_path(self.goal_pos)
            if path:
                self._activate_plan(self.goal_pos, path,
                                    "🎯 Goal is within explored space — committing to direct path.")
                self._update_visualization()
                return

        if not self.frontier_display:
            print("⚠️ No frontiers — attempting direct goal path.")
            path = self._plan_path(self.goal_pos)
            if path:
                self._activate_plan(self.goal_pos, path,
                                    f"🎯 Direct path to goal found ({len(path)} pts)")
                self._update_visualization()
            else:
                print("✗ Goal still unreachable — waiting for more observations.")
                self._clear_inflation_mask()
            return

        reachable = []
        for fx, fy in self.frontier_display:
            gx, gy = self.occupancy_grid.world_to_grid(fx, fy)
            if not self.occupancy_grid.is_free(gx, gy):
                continue
            if self._is_blocked_frontier((fx, fy)):
                continue
            if not self._matches_final_goal((fx, fy)) and self._within_inflation_buffer(gx, gy):
                continue
            path = self._plan_path((fx, fy))
            if path:
                reachable.append(((fx, fy), path))

        if not reachable:
            print("⚠️ Frontiers detected but no collision-free paths yet. Retrying goal.")
            path = self._plan_path(self.goal_pos)
            if path:
                self._activate_plan(self.goal_pos, path,
                                    f"🎯 Using fallback path to goal ({len(path)} pts)")
            else:
                self._clear_inflation_mask()
            self._update_visualization()
            return

        best_score = float("inf")
        best_frontier = None
        best_path = None
        for (fx, fy), path in reachable:
            dist_goal = np.hypot(self.goal_pos[0] - fx, self.goal_pos[1] - fy)
            robot_dist = np.hypot(self.robot.x - fx, self.robot.y - fy)
            score = 0.7 * dist_goal + 0.3 * robot_dist
            if score < best_score:
                best_score = score
                best_frontier = (fx, fy)
                best_path = path

        if best_frontier and best_path:
            self._activate_plan(best_frontier, best_path,
                                f"🧭 Selected frontier {best_frontier} ({len(best_path)} pts)")
            self._update_visualization()

    def _plan_path(self, target: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """Run A* toward the provided target tuple."""
        path = self.path_planner.plan(
            self.robot.x, self.robot.y, target[0], target[1],
            inflation_radius=self.inflation_radius_cells
        )
        if target != self.goal_pos and path and self._goal_in_known_space():
            direct_goal = self.path_planner.plan(
                self.robot.x, self.robot.y, self.goal_pos[0], self.goal_pos[1],
                inflation_radius=self.inflation_radius_cells
            )
            if direct_goal:
                print("🔄 Overriding frontier with direct path to final goal.")
                return direct_goal
        return path

    def _persist_environment(self):
        data = {
            "obstacles": [self._serialize_obstacle(obs) for obs in self.obstacles],
            "start": list(self.start_pos),
            "goal": list(self.goal_pos),
            "map_size": list(self.map_size),
            "resolution": self.resolution,
        }
        os.makedirs("runs", exist_ok=True)
        with open(os.path.join("runs", "last_environment.json"), "w") as f:
            json.dump(data, f, indent=2)

    def _load_or_generate_environment(self, use_saved, map_size, resolution,
                                      start_override, goal_override):
        if use_saved:
            saved = self._load_saved_environment()
            if saved:
                print("📁 Loaded last saved environment.")
                start = tuple(saved["start"])
                goal = tuple(saved["goal"])
                if start_override is not None:
                    start = start_override
                if goal_override is not None:
                    goal = goal_override
                self.map_size = tuple(saved.get("map_size", map_size))
                self.resolution = saved.get("resolution", resolution)
                return [self._deserialize_obstacle(o) for o in saved["obstacles"]], start, goal
            else:
                print("⚠️ No saved environment found; generating new one.")

        obstacles, start, goal = self._manual_maze_environment(map_size)
        if start_override is not None:
            start = start_override
        if goal_override is not None:
            goal = goal_override
        return obstacles, start, goal

    def _load_saved_environment(self):
        path = os.path.join("runs", "last_environment.json")
        if not os.path.exists(path):
            return None
        with open(path, "r") as f:
            return json.load(f)

    def _manual_maze_environment(self, map_size: Tuple[float, float]):
        W, H = map_size
        obstacles = [
            Box(x=3.5, y=H - 2.0, width=4.0, length=0.9, height=1.5, color=(0.6, 0.4, 0.3)),
            Box(x=W - 4.25, y=H - 2.0, width=4.0, length=0.3, height=1.5, color=(0.45, 0.35, 0.3)),
            Box(x=7.5, y=H - 5.0, width=0.9, length=4.5, height=1.6, color=(0.4, 0.4, 0.4)),
            Cylinder(x=1.5, y=H / 2.0, radius=0.55, height=1.2, color=(0.7, 0.6, 0.2)),
            Cylinder(x=W - 1.5, y=(H / 2.0) + 1.0, radius=0.65, height=1.2, color=(0.7, 0.5, 0.3)),
            Cylinder(x=(W / 2.0) - 3.0, y=(H / 2.0) - 2.5, radius=0.8, height=1.3, color=(0.55, 0.55, 0.2)),
            Cylinder(x=(W / 2.0) + 3.0, y=(H / 2.0) - 2.5, radius=0.8, height=1.3, color=(0.55, 0.55, 0.2)),
            Box(x=3.5, y=2.8, width=4.2, length=0.9, height=1.4, rotation=0.10, color=(0.5, 0.35, 0.25)),
            Box(x=W - 3.5, y=3.0, width=3.8, length=0.9, height=1.4, rotation=-0.10, color=(0.55, 0.3, 0.2)),
            Box(x=10.0, y=8.0, width=1.0, length=1.0, height=1.4, rotation=0.30, color=(0.5, 0.35, 0.25)),
            Cylinder(x=(W / 2.0) + 2.0, y=4.0, radius=0.75, height=1.0, color=(0.65, 0.4, 0.35)),
            Cylinder(x=(6.5) , y=5.0, radius=0.85, height=1.0, color=(0.65, 0.4, 0.35)),

        ]

        start = (0, 0)
        goal  = (6, 14)

        return obstacles, start, goal

    def _serialize_obstacle(self, obs):
        data = {"type": obs.__class__.__name__}
        for attr in ["x", "y", "width", "length", "height", "rotation", "radius", "color"]:
            if hasattr(obs, attr):
                val = getattr(obs, attr)
                if isinstance(val, np.ndarray):
                    val = val.tolist()
                data[attr] = val
        return data

    def _deserialize_obstacle(self, data):
        from src.objects.primitives import Box, Cylinder, Cone, Hemisphere
        cls_map = {
            "Box": Box,
            "Cylinder": Cylinder,
            "Cone": Cone,
            "Hemisphere": Hemisphere,
        }
        cls = cls_map.get(data.get("type", "Box"), Box)
        kwargs = {k: v for k, v in data.items() if k not in ("type",)}
        return cls(**kwargs)

    def _activate_plan(self, goal: Tuple[float, float], path: List[Tuple[float, float]], message: str = ""):
        """Accept a newly planned path and reset controller state."""
        self.current_goal = goal
        self.current_path = path
        self.path_controller.reset()
        self._store_inflation_mask()
        self.stuck_counter = 0
        if message:
            print(message)

    def _store_inflation_mask(self):
        mask = getattr(self.path_planner, "_inflated_mask", None)
        if mask is None:
            self._clear_inflation_mask()
            return
        if self.inflated_mask.shape != mask.shape:
            self.inflated_mask = np.array(mask, dtype=bool, copy=True)
        else:
            self.inflated_mask = mask.copy()

    def _clear_inflation_mask(self):
        if self.inflated_mask.shape != self.occupancy_grid.grid.shape:
            self.inflated_mask = np.zeros_like(self.occupancy_grid.grid, dtype=bool)
        else:
            self.inflated_mask.fill(False)

    def _goal_in_known_space(self) -> bool:
        """Return True if the final goal cell has been observed (not unknown)."""
        gx, gy = self.occupancy_grid.world_to_grid(self.goal_pos[0], self.goal_pos[1])
        if not self.occupancy_grid.is_valid(gx, gy):
            return False
        return not self.occupancy_grid.is_unknown(gx, gy)
    
    def _matches_final_goal(self, point: Tuple[float, float]) -> bool:
        """Check if a candidate point coincides with the final goal (within resolution tolerance)."""
        return np.hypot(point[0] - self.goal_pos[0], point[1] - self.goal_pos[1]) < max(
            0.5 * self.resolution, 0.05
        )

    def _follow_path(self):
        if not self.current_path or len(self.current_path) < 2:
            return

        if self._path_is_blocked():
            self.current_goal = None
            self.current_path = None
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
            if abs(delta) < 1e-6 or abs(v) < 1e-6:
                return v, v, v, v

            # Instantaneous center of curvature (ICC)
            R_center = L / np.tan(delta)
            if abs(R_center) < 1e-3:
                R_center = np.sign(R_center) * 1e-3
            omega = v / R_center
            omega_mag = abs(omega)
            sign_v = np.sign(v)

            # Radii to each wheel (ensure non-negative distances)
            R_rear_inner = max(1e-3, abs(R_center) - track / 2)
            R_rear_outer = abs(R_center) + track / 2
            R_front_inner = np.hypot(L, R_rear_inner)
            R_front_outer = np.hypot(L, R_rear_outer)

            # Linear velocities (match direction of commanded velocity)
            v_rear_inner = sign_v * omega_mag * R_rear_inner
            v_rear_outer = sign_v * omega_mag * R_rear_outer
            v_front_inner = sign_v * omega_mag * R_front_inner
            v_front_outer = sign_v * omega_mag * R_front_outer

            # Assign depending on turn direction
            if delta > 0:   # turning left
                return v_front_inner, v_front_outer, v_rear_inner, v_rear_outer
            else:           # turning right
                return v_front_outer, v_front_inner, v_rear_outer, v_rear_inner

        wheelbase = getattr(self.robot, "L", getattr(self.robot, "wheelbase", 0.3))
        track_width = 0.25
        FL, FR, RL, RR = wheel_speeds(v, delta, wheelbase, track_width)

        t = time.time() - self.start_time
        self.time_log.append(t)
        self.vel_log.append(v)
        yaw_rate = (v / wheelbase) * np.tan(delta) if abs(wheelbase) > 1e-6 else 0.0
        self.yaw_log.append(yaw_rate)
        self.steer_log.append(delta)
        self.fl_log.append(FL)
        self.fr_log.append(FR)
        self.rl_log.append(RL)
        self.rr_log.append(RR)

        # --- Move ---
        prev_x, prev_y, prev_theta = self.robot.x, self.robot.y, self.robot.theta
        self.robot.step(v, delta)
        if self._is_in_collision(self.robot.x, self.robot.y):
            print("⚠️ Collision detected/anticipated — backing up and replanning.")
            self.robot.x, self.robot.y, self.robot.theta = prev_x, prev_y, prev_theta
            self._perform_reverse_escape(steps=20, speed=0.4)
            self.current_goal = None
            self.current_path = None
            self.path_controller.reset()
            return

        self.total_distance += np.hypot(self.robot.x - prev_x, self.robot.y - prev_y)
        self._trim_path_after_progress()

        if len(self.current_path) < 2:
            self.current_path = None

        step_progress = np.hypot(self.robot.x - prev_x, self.robot.y - prev_y)
        if step_progress < 1e-3:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        if self.stuck_counter > self.max_stuck_iterations:
            print("⚠️ Vehicle appears stuck — clearing current goal to replan.")
            self._perform_reverse_escape()
            self.current_goal = None
            self.current_path = None
            self.stuck_counter = 0

    def _trim_path_after_progress(self):
        """Drop waypoints that are already behind the robot."""
        if not self.current_path:
            return
        while len(self.current_path) > 2:
            next_pt = self.current_path[1]
            dist = np.hypot(self.robot.x - next_pt[0], self.robot.y - next_pt[1])
            if dist < 0.18:
                self.current_path.pop(0)
            else:
                break

    def _path_is_blocked(self) -> bool:
        """Check if newly observed obstacles invalidate the current path."""
        if not self.current_path or len(self.current_path) < 2:
            return False
        sample_step = max(1, len(self.current_path) // 30)
        for idx in range(0, len(self.current_path), sample_step):
            px, py = self.current_path[idx]
            gx, gy = self.occupancy_grid.world_to_grid(px, py)
            if not self.occupancy_grid.is_valid(gx, gy):
                return True
            if self._cell_is_blocked(gx, gy, include_margin=False):
                return True
            if self._neighborhood_has_new_obstacle(gx, gy):
                return True
        return False

    def _cell_is_blocked(self, gx: int, gy: int, include_margin: bool = True) -> bool:
        if not self.occupancy_grid.is_valid(gx, gy):
            return True
        if self.occupancy_grid.is_occupied(gx, gy):
            return True
        if include_margin:
            if 0 <= gy < self.inflated_mask.shape[0] and 0 <= gx < self.inflated_mask.shape[1]:
                if self.inflated_mask[gy, gx]:
                    return True
        return False

    def _neighborhood_has_new_obstacle(self, gx: int, gy: int) -> bool:
        """Check if newly mapped occupancy violates the safety radius."""
        rad = max(1, self.inflation_radius_cells)
        x0 = max(0, gx - rad)
        x1 = min(self.occupancy_grid.width, gx + rad + 1)
        y0 = max(0, gy - rad)
        y1 = min(self.occupancy_grid.height, gy + rad + 1)
        window = self.occupancy_grid.grid[y0:y1, x0:x1]
        return np.any(window > 0.65)

    def _is_in_collision(self, x: float, y: float) -> bool:
        gx, gy = self.occupancy_grid.world_to_grid(x, y)
        in_buffer = self._within_inflation_buffer(gx, gy)
        include_margin = not in_buffer
        return self._cell_is_blocked(gx, gy, include_margin=include_margin)

    def _within_inflation_buffer(self, gx: int, gy: int) -> bool:
        if not self.occupancy_grid.is_valid(gx, gy):
            return False
        rad = max(1, self.inflation_radius_cells)
        for dy in range(-rad, rad + 1):
            ny = gy + dy
            if ny < 0 or ny >= self.occupancy_grid.height:
                continue
            for dx in range(-rad, rad + 1):
                if dx * dx + dy * dy > rad * rad:
                    continue
                nx = gx + dx
                if nx < 0 or nx >= self.occupancy_grid.width:
                    continue
                if self.occupancy_grid.is_occupied(nx, ny):
                    return True
        return False

    def _perform_reverse_escape(self, steps: int = 12, speed: float = 0.25):
        """Back up a few steps to free the robot before replanning."""
        print("↩️ Backing up to regain clearance...")
        reverse_speed = -abs(speed)
        for _ in range(steps):
            prev_x, prev_y = self.robot.x, self.robot.y
            self.robot.step(reverse_speed, 0.0)
            self.visited_positions.append((self.robot.x, self.robot.y))
            self.total_distance += np.hypot(self.robot.x - prev_x, self.robot.y - prev_y)
            if self._is_in_collision(self.robot.x, self.robot.y):
                break
        self.path_controller.reset()

    def _mark_frontier_blocked(self, frontier: Tuple[float, float]):
        """Remember a problematic frontier so it is not selected again."""
        if self._matches_final_goal(frontier):
            return
        tol = max(self.resolution, 0.15)
        if not self._is_blocked_frontier(frontier, tol):
            self.blocked_frontiers.append(frontier)

    def _is_blocked_frontier(self, frontier: Tuple[float, float], tol: Optional[float] = None) -> bool:
        """Check if a frontier is in the blocked cache."""
        tol = tol or max(self.resolution, 0.15)
        for bx, by in self.blocked_frontiers:
            if np.hypot(bx - frontier[0], by - frontier[1]) <= tol:
                return True
        return False

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
            if self.current_goal and not self._matches_final_goal(self.current_goal):
                self._mark_frontier_blocked(self.current_goal)
            self.current_goal = None
            self.current_path = None
            self.iterations_without_progress = 0

    def _is_goal_reached(self):
        if self.current_goal is None:
            return True
        tol = self.final_goal_tolerance if self._matches_final_goal(self.current_goal) else self.goal_tolerance
        return self.path_controller.is_goal_reached(
            self.robot.x, self.robot.y, self.current_goal[0], self.current_goal[1], tolerance=tol
        )

    def _is_at_final_goal(self):
        tol = 0.2
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



def main():
    explorer = GoalDirectedExplorer(start_pos=None, goal_pos=None,
                                    map_size=(15, 15), resolution=0.1)
    explorer.run_exploration(max_iterations=15000)


if __name__ == "__main__":
    main()
