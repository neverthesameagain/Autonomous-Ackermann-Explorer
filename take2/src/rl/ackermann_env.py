# src/rl/ackermann_env.py
import numpy as np
import gymnasium as gym
from gymnasium import spaces

# You already have these:
# from src.goal_directed_explorer import GoalDirectedExplorer
# But for RL, we only need the world/sensor/step hooks from your components:
from src.robot.ackermann import AckermannRobot
from src.sensors.lidar import LidarSensor
from src.map.occupancy_grid import OccupancyGrid
from src.planning.astar import AStarPlanner
from src.planning.frontier_explorer import FrontierExplorer
from src.objects.generator import generate_environment

class AckermannExploreEnv(gym.Env):
    """
    Continuous-control RL env for adaptive avoidance.
    Action = [v_cmd, delta_cmd] (Box)
    Observation = lidar ranges + goal vector (robot frame) + (v, delta)
    Reward = progress-to-goal + frontier info gain - collision/oscillation penalties
    """
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(
        self,
        map_size=(20, 20),
        resolution=0.1,
        lidar_fov=270,
        lidar_beams=108,
        lidar_range=6.0,
        max_steps=1500,
        seed: int | None = 42,
        discrete: bool = False
    ):
        super().__init__()
        self.rng = np.random.default_rng(seed)
        self.map_size = map_size
        self.resolution = resolution
        self.max_steps = max_steps
        self.discrete = discrete

        # --- World ---
        self.obstacles = generate_environment(
            num_obs=10, bounds=map_size, min_clearance=0.6, edge_buffer=0.3, rng_seed=seed
        )
        self.start = generate_environment.last_start
        self.goal = generate_environment.last_goal

        # --- Robot & sensors ---
        self.robot = AckermannRobot(x=self.start[0], y=self.start[1], theta=0.0, wheelbase=0.3, dt=0.03)
        self.lidar = LidarSensor(fov=lidar_fov, num_beams=lidar_beams, max_range=lidar_range)

        # --- Mapping & planning helpers (for shaping only, not mandatory to use each step)
        self.grid = OccupancyGrid(map_size=map_size, resolution=resolution)
        self.frontier = FrontierExplorer(self.grid)
        self.planner = AStarPlanner(self.grid)

        # --- Spaces ---
        # Observation: lidar (N), goal vector in robot frame (dx, dy, dist, bearing), speed v, steer delta
        self.n_beams = lidar_beams
        low_obs  = np.concatenate([np.zeros(self.n_beams),  [-map_size[0], -map_size[1], 0.0, -np.pi, -3.0, -np.pi/3]])
        high_obs = np.concatenate([np.ones(self.n_beams),   [ map_size[0],  map_size[1], np.hypot(*map_size),  np.pi,  3.0,  np.pi/3]])
        self.observation_space = spaces.Box(low=low_obs.astype(np.float32), high=high_obs.astype(np.float32), dtype=np.float32)

        if self.discrete:
            # 5 macro-actions: sharp left, left, straight, right, sharp right
            self.action_space = spaces.Discrete(5)
        else:
            # Continuous [v_cmd, delta_cmd]
            self.action_space = spaces.Box(
                low=np.array([0.0, -np.pi/3], dtype=np.float32),
                high=np.array([2.0,  np.pi/3], dtype=np.float32),
                dtype=np.float32
            )

        self._steps = 0
        self._last_goal_dist = None
        self._last_pose = None

        # Prime mapping
        self._sense_and_map()

    # ---------- Core Gym API ----------
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        # New world each reset
        self.obstacles = generate_environment(
            num_obs=10, bounds=self.map_size, min_clearance=0.6, edge_buffer=0.3, rng_seed=int(self.rng.integers(0, 1e6))
        )
        self.start = generate_environment.last_start
        self.goal  = generate_environment.last_goal
        self.robot = AckermannRobot(x=self.start[0], y=self.start[1], theta=0.0, wheelbase=0.3, dt=0.03)
        self.grid  = OccupancyGrid(map_size=self.map_size, resolution=self.resolution)
        self.frontier = FrontierExplorer(self.grid)
        self.planner  = AStarPlanner(self.grid)

        self._steps = 0
        self._last_pose = np.array([self.robot.x, self.robot.y, self.robot.theta])
        self._sense_and_map()
        self._last_goal_dist = self._goal_distance()

        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action):
        self._steps += 1

        # --- Decode action ---
        if self.discrete:
            # map discrete to (v, delta)
            mapping = {
                0: (0.8,  +0.6),   # sharp left
                1: (1.2,  +0.3),   # left
                2: (1.5,   0.0),   # straight
                3: (1.2,  -0.3),   # right
                4: (0.8,  -0.6),   # sharp right
            }
            v_cmd, delta_cmd = mapping[int(action)]
        else:
            v_cmd  = float(np.clip(action[0], 0.0, 2.0))
            delta_cmd = float(np.clip(action[1], -np.pi/3, np.pi/3))

        # --- Safety: stop if cell ahead is occupied (inflation as guard) ---
        if self._imminent_collision(v_cmd, delta_cmd):
            v_cmd = 0.0  # hard brake, agent learns to avoid high-cost states

        # --- Simulate step ---
        prev = np.array([self.robot.x, self.robot.y])
        self.robot.step(v_cmd, delta_cmd)

        # --- Sense & map ---
        self._sense_and_map()

        # --- Compute reward ---
        reward, terminated, truncated = self._reward_and_done(prev)

        obs = self._get_obs()
        info = {"goal": self.goal, "pos": (self.robot.x, self.robot.y)}
        return obs, reward, terminated, truncated, info

    def render(self):
        # Rendering is handled by your Matplotlib windows; no-op here.
        pass

    # ---------- Helpers ----------
    def _sense_and_map(self):
        ranges = self.lidar.scan(self.robot, self.obstacles)
        self.grid.update_from_lidar(self.robot.x, self.robot.y, self.robot.theta, ranges, self.lidar.angles)

    def _goal_distance(self):
        return float(np.hypot(self.robot.x - self.goal[0], self.robot.y - self.goal[1]))

    def _bearing_to_goal(self):
        dx, dy = self.goal[0] - self.robot.x, self.goal[1] - self.robot.y
        ang = np.arctan2(dy, dx) - self.robot.theta
        return float(np.arctan2(np.sin(ang), np.cos(ang)))  # wrap

    def _get_obs(self):
        # Lidar normalized to [0,1] (1 = max range)
        ranges = self.lidar.scan(self.robot, self.obstacles)
        r_norm = np.clip(np.array(ranges) / self.lidar.max_range, 0.0, 1.0)

        # Goal vector in robot frame
        dx, dy = self.goal[0] - self.robot.x, self.goal[1] - self.robot.y
        # rotate into robot frame
        c, s = np.cos(-self.robot.theta), np.sin(-self.robot.theta)
        gx_r = c * dx - s * dy
        gy_r = s * dx + c * dy
        dist = np.hypot(dx, dy)
        bearing = self._bearing_to_goal()

        # Append current commanded speeds if you cache them; otherwise zeros:
        v = getattr(self.robot, "v", 0.0)
        delta = getattr(self.robot, "delta", 0.0)

        obs = np.concatenate([r_norm, [gx_r, gy_r, dist, bearing, v, delta]]).astype(np.float32)
        return obs

    def _imminent_collision(self, v_cmd, delta_cmd):
        # Project a short arc ahead and check inflated occupancy
        horizon = max(0.15, 0.6 * v_cmd)  # lookahead horizon
        steps = 5
        L = self.robot.wheelbase if hasattr(self.robot, "wheelbase") else 0.3
        x, y, th = self.robot.x, self.robot.y, self.robot.theta
        for _ in range(steps):
            x += (horizon/steps) * np.cos(th)
            y += (horizon/steps) * np.sin(th)
            th += (horizon/steps) * np.tan(delta_cmd) / L
            gx, gy = self.grid.world_to_grid(x, y)
            if not self.grid.is_valid(gx, gy):  # out of map treated as collision
                return True
            if self.grid.is_occupied(gx, gy):   # inflated obstacle
                return True
        return False

    def _reward_and_done(self, prev_pos):
        terminated = False
        truncated  = False

        # 1) Goal progress
        d_now = self._goal_distance()
        d_prev = self._last_goal_dist
        prog = d_prev - d_now  # positive when getting closer
        self._last_goal_dist = d_now

        # 2) Frontier info gain (optional, cheap proxy: number of newly known cells)
        # Use change in known cells since last step
        known = np.sum((self.grid.grid < 0.35) | (self.grid.grid > 0.65))
        if not hasattr(self, "_last_known"):
            self._last_known = known
        info_gain = (known - self._last_known) / 100.0
        self._last_known = known

        # 3) Smoothness (penalize oscillation)
        pose = np.array([self.robot.x, self.robot.y, self.robot.theta])
        jerk = np.linalg.norm(pose[:2] - prev_pos)  # small step -> small jerk proxy
        smooth_pen = 0.0 if jerk > 0 else -0.01

        # 4) Collision penalty (hard)
        gx, gy = self.grid.world_to_grid(self.robot.x, self.robot.y)
        collision = (not self.grid.is_valid(gx, gy)) or self.grid.is_occupied(gx, gy)
        col_pen = -5.0 if collision else 0.0
        if collision:
            terminated = True

        # 5) Success
        goal_reached = d_now < 0.6
        if goal_reached:
            terminated = True

        # 6) Time limit
        if self._steps >= self.max_steps:
            truncated = True

        reward = 1.0 * prog + 0.2 * info_gain + smooth_pen + col_pen + (10.0 if goal_reached else 0.0)
        return float(reward), terminated, truncated
