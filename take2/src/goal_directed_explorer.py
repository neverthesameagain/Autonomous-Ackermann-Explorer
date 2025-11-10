"""
Goal-Directed Autonomous Exploration
Navigate from start to goal while exploring and mapping the environment.
"""
import numpy as np
import time
import matplotlib.pyplot as plt
from typing import Optional, List, Tuple

from src.robot.ackermann import AckermannRobot
from src.sensors.lidar import LidarSensor
from src.map.occupancy_grid import OccupancyGrid
from src.planning.astar import AStarPlanner
from src.planning.frontier_explorer import FrontierExplorer
from src.control.velocity_controller import PathFollowingController, TrapezoidalVelocityController
from src.env.env_3d import Renderer3D
from src.objects.generator import generate_environment


class GoalDirectedExplorer:
    def __init__(self, start_pos=None, goal_pos=None, map_size=(20, 20), resolution=0.1):
        # --- Generate random environment first ---
        obstacles = generate_environment(num_obs=15, bounds=map_size,
                                         start_pos=start_pos, goal_pos=goal_pos)

        # Retrieve random start/goal if they were None
        if start_pos is None:
            start_pos = generate_environment.last_start
        if goal_pos is None:
            goal_pos = generate_environment.last_goal

        self.start_pos, self.goal_pos = start_pos, goal_pos
        self.map_size, self.resolution = map_size, resolution
        self.obstacles = obstacles

        print("=" * 70)
        print("GOAL-DIRECTED AUTONOMOUS EXPLORATION SYSTEM (v2.1)")
        print("=" * 70)
        print(f"Start: {self.start_pos}  Goal: {self.goal_pos}")


        # Initialize modules
        self.robot = AckermannRobot(x=start_pos[0], y=start_pos[1], theta=0, wheelbase=0.3, dt=0.1)
        self.lidar = LidarSensor(fov=270, num_beams=108, max_range=6.0)
        self.occupancy_grid = OccupancyGrid(map_size=map_size, resolution=resolution)
        self.path_planner = AStarPlanner(self.occupancy_grid)
        self.frontier_explorer = FrontierExplorer(self.occupancy_grid)
        velocity_controller = TrapezoidalVelocityController(max_velocity=0.5, max_acceleration=0.3, max_angular_velocity=0.8)
        self.path_controller = PathFollowingController(lookahead_distance=0.5, velocity_controller=velocity_controller)
        self.obstacles = generate_environment(num_obs=10, bounds=map_size, start_pos=start_pos, goal_pos=goal_pos)

        self._setup_visualization()

        # State
        self.current_path = None
        self.current_goal = None
        self.goal_reached = False
        self.total_distance = 0.0
        self.visited_positions = []
        self.frontiers_visited = []
        self.frontier_display = []       # currently active frontiers
        self.visited_frontiers = set()   # frontiers already reached
        self.blacklist = set()
        self.blacklist_ttl = {}
        self.iterations_without_progress = 0
        self.best_distance_to_goal = float('inf')
        self.position_history = []
        self.start_time = time.time()
        self.inflation_radius_cells = int(round(0.4 / resolution))
        print(f"✓ Initialization complete (Inflation radius: {self.inflation_radius_cells} cells)")

    # ----------------------------
    #  VISUALIZATION
    # ----------------------------
    def _setup_visualization(self):
        plt.ion()
        self.fig, self.ax_2d = plt.subplots(figsize=(7, 7))
        self.ax_2d.set_xlim(0, self.map_size[0])
        self.ax_2d.set_ylim(0, self.map_size[1])
        self.ax_2d.set_aspect('equal')
        self.ax_2d.set_title("2D Exploration Map")

    def _update_visualization(self):
        self.ax_2d.clear()
        grid = self.occupancy_grid.grid
        colored = np.zeros((*grid.shape, 3))
        unknown = (grid >= 0.35) & (grid <= 0.65)
        free = grid < 0.35
        occ = grid > 0.65
        colored[unknown] = [0.7, 0.7, 0.7]
        colored[free] = [0.8, 0.9, 1.0]
        colored[occ] = [0.2, 0.1, 0.1]
        self.ax_2d.imshow(colored, origin='lower',
                          extent=[0, self.map_size[0], 0, self.map_size[1]])
        traj = np.array(self.visited_positions)
        if len(traj) > 1:
            self.ax_2d.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1.5)
        self.ax_2d.scatter(*self.start_pos, c='green', s=100, label="Start")
        self.ax_2d.scatter(*self.goal_pos, c='red', s=100, marker='X', label="Goal")
        self.ax_2d.scatter(self.robot.x, self.robot.y, c='blue', s=80)
        self.ax_2d.legend()
        # Draw frontiers (green dots) and selected frontier (red star)
# Draw detected frontiers
        if hasattr(self, 'frontier_display') and self.frontier_display:
            fx, fy = zip(*self.frontier_display)
            self.ax_2d.scatter(fx, fy, s=40, color='lime', alpha=0.8, label="Frontiers")

        # Draw selected frontier
        if self.current_goal:
            self.ax_2d.scatter(self.current_goal[0], self.current_goal[1],
                            marker='*', s=180, color='red', edgecolor='k', zorder=6, label="Selected Frontier")


        plt.pause(0.001)

    # ----------------------------
    #  CORE LOOP
    # ----------------------------
    def run_exploration(self, max_iterations=5000):
        print("\n🚀 Starting Exploration Loop...")
        iteration = 0
        while iteration < max_iterations and not self.goal_reached:
            iteration += 1

            # 1. SENSE + MAP
            self._sense_and_map()

            # 2. PLAN if no path
            if self.current_goal is None or self._is_goal_reached():
                self._plan_next_goal()

            # 3. CONTROL
            if self.current_path:
                self._follow_path()

            # 4. MONITOR
            self._monitor_progress(iteration)

            # 5. VISUALIZE
            if iteration % 10 == 0:
                self._update_visualization()

            if self._is_at_final_goal():
                print("\n🎯 FINAL GOAL REACHED!")
                self.goal_reached = True
                break

        self._print_final_stats(iteration)
        plt.ioff()
        plt.show()

    # ----------------------------
    #  SUBROUTINES
    # ----------------------------
    def _sense_and_map(self):
        ranges = self.lidar.scan(self.robot, self.obstacles)
        self.occupancy_grid.update_from_lidar(
            self.robot.x, self.robot.y, self.robot.theta,
            ranges, self.lidar.angles)
        self.visited_positions.append((self.robot.x, self.robot.y))

    def _plan_next_goal(self):
        """Find all visible frontiers, show them, and select one reachable frontier to explore next."""

        # 1️⃣ Try direct path to final goal first
        direct_path = self.path_planner.plan(
            self.robot.x, self.robot.y,
            self.goal_pos[0], self.goal_pos[1],
            inflation_radius=self.inflation_radius_cells
        )
        # if direct_path:
        #     print(f"🎯 Direct path to goal found ({len(direct_path)} waypoints)")
        #     self.current_goal, self.current_path = self.goal_pos, direct_path
        #     return

        # 2️⃣ Detect all current frontiers
        all_frontiers = self.frontier_explorer.find_frontiers()
        self.frontier_display = all_frontiers
        if not all_frontiers:
            print("⚠️ No frontiers detected — exploration may be complete.")
            return

        print(f"\n🔍 Detected {len(all_frontiers)} raw frontiers in the explored map:")
        for (fx, fy) in all_frontiers:
            print(f"   → Frontier at ({fx:.2f}, {fy:.2f})")

        # Update visualization to show all frontiers
        self._update_visualization()
        plt.pause(0.5)

        # 3️⃣ Filter reachable frontiers
        reachable = []
        for fx, fy in all_frontiers:
            gx, gy = self.occupancy_grid.world_to_grid(fx, fy)
            cell_status = "free" if self.occupancy_grid.is_free(gx, gy) else "blocked"

            # Skip if not free or previously visited
            if not self.occupancy_grid.is_free(gx, gy):
                print(f"   ✗ Frontier ({fx:.2f}, {fy:.2f}) rejected — not in free space ({cell_status}).")
                continue
            if any(np.hypot(fx - vx, fy - vy) < 0.5 for vx, vy in self.frontiers_visited):
                print(f"   ✗ Frontier ({fx:.2f}, {fy:.2f}) skipped — already visited.")
                continue

            # Check reachability using A*
            path = self.path_planner.plan(
                self.robot.x, self.robot.y, fx, fy,
                inflation_radius=self.inflation_radius_cells
            )
            if path:
                print(f"   ✓ Frontier ({fx:.2f}, {fy:.2f}) reachable (path length {len(path)}).")
                reachable.append((fx, fy, path))
            else:
                print(f"   ✗ Frontier ({fx:.2f}, {fy:.2f}) unreachable (no path).")

        if not reachable:
            print("❌ No reachable frontiers in explored region — robot will rescan.")
            return

        # 4️⃣ Select the best reachable frontier
        best_score, best_frontier, best_path, reason = float("inf"), None, None, ""
        for (fx, fy, path) in reachable:
            dist_to_goal = np.hypot(self.goal_pos[0] - fx, self.goal_pos[1] - fy)
            info_gain = self.frontier_explorer._calculate_information_gain(fx, fy)
            score = dist_to_goal - 0.02 * info_gain

            if score < best_score:
                best_score, best_frontier, best_path = score, (fx, fy), path
                reason = f"closest to goal ({dist_to_goal:.2f}) with info gain {info_gain:.2f}"

        # 5️⃣ Commit and show decision
        self.current_goal = best_frontier
        self.current_path = best_path
        print(f"\n🧭 Selected frontier: {best_frontier} — Reason: {reason}")
        print(f"→ Path length: {len(best_path)} waypoints")

        # Visualize selection
        self._update_visualization()
        plt.pause(0.5)



    def _follow_path(self):
        if not self.current_path or len(self.current_path) < 2:
            return
        # If reached the current frontier, mark it visited and clear display
# If reached current frontier, clear markers and replan
        if self.current_goal and np.hypot(self.robot.x - self.current_goal[0], self.robot.y - self.current_goal[1]) < 0.5:
            print(f"\n✅ Reached selected frontier at {self.current_goal}, clearing frontiers...")
            self.frontiers_visited.append(self.current_goal)
            self.frontier_display.clear()
            self.current_goal = None
            self.current_path = None
            return


        v, delta = self.path_controller.compute_control(
            self.robot.x, self.robot.y, self.robot.theta,
            self.current_path, self.robot.dt)
        ranges = self.lidar.scan(self.robot, self.obstacles)
        if min(ranges) < 0.5:
            v = min(v, 0.1)
        prev_x, prev_y = self.robot.x, self.robot.y
        self.robot.step(v, delta)
        self.total_distance += np.hypot(self.robot.x - prev_x, self.robot.y - prev_y)

    def _monitor_progress(self, iteration):
        dist_to_goal = np.hypot(self.robot.x - self.goal_pos[0], self.robot.y - self.goal_pos[1])
        if dist_to_goal < self.best_distance_to_goal - 0.5:
            self.best_distance_to_goal = dist_to_goal
            self.iterations_without_progress = 0
        else:
            self.iterations_without_progress += 1
        if self.iterations_without_progress > 200:
            print("⚠️ No progress detected, resetting goal...")
            self.current_goal = None
            self.iterations_without_progress = 0

    # ----------------------------
    #  HELPERS
    # ----------------------------
    def _select_best_frontier(self, frontiers):
        best_score, best_frontier = float('inf'), None
        for (fx, fy) in frontiers:
            if any(np.hypot(fx - vx, fy - vy) < 0.5 for vx, vy in self.frontiers_visited):
                continue
            dist_goal = np.hypot(self.goal_pos[0] - fx, self.goal_pos[1] - fy)
            info = self.frontier_explorer._calculate_information_gain(fx, fy)
            score = dist_goal - 0.01 * info
            if score < best_score:
                best_score, best_frontier = score, (fx, fy)
        return best_frontier

    def _is_goal_reached(self):
        if self.current_goal is None:
            return True
        return self.path_controller.is_goal_reached(
            self.robot.x, self.robot.y,
            self.current_goal[0], self.current_goal[1],
            tolerance=0.8)

    def _is_at_final_goal(self):
        return np.hypot(self.robot.x - self.goal_pos[0], self.robot.y - self.goal_pos[1]) < 0.4

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
        print(f"Elapsed: {elapsed:.1f}s  |  Avg Speed: {self.total_distance / elapsed:.2f} m/s")
        print("=" * 70)


# ======================================================
# ============= ENTRY POINT ============================
# ======================================================



def main():
    """Main entry point."""
    # Define start and goal positions (closer with obstacles in between)
    START = None
    GOAL = None  # Closer goal to see more exploration behavior
    
    explorer = GoalDirectedExplorer(
        start_pos=START,
        goal_pos=GOAL,
        map_size=(20, 20),
        resolution=0.1
    )
    
    explorer.run_exploration(max_iterations=20000)


if __name__ == "__main__":
    main()
