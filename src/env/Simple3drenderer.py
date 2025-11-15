import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Simple3DRenderer:
    def __init__(self, ax3d, map_size):
        self.ax = ax3d
        self.map_size = map_size
        self.car_box = None
        self.goal_marker = None
        self.obstacles = []
        self.goal = None
        self.path_line = None
        self.heading_line = None
        self.smooth_focus = np.array([map_size[0] / 2.0, map_size[1] / 2.0, 0.0])
        self.cam_alpha = 0.15
        self._configure_axes()

    def _configure_axes(self):
        self.ax.set_facecolor((0.92, 0.95, 1.0))
        self.ax.set_box_aspect([self.map_size[0], self.map_size[1], 0.5 * max(self.map_size)])
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.set_xlim(0, self.map_size[0])
        self.ax.set_ylim(0, self.map_size[1])
        self.ax.set_zlim(0, max(2.5, 0.2 * max(self.map_size)))
        self.ax.set_title("3D Follow-Camera View")

    def draw_environment(self, obstacles, goal):
        self.ax.cla()
        self._configure_axes()
        self.obstacles = list(obstacles)
        self.goal = goal
        self._draw_ground()
        self._draw_bounds()
        for obs in self.obstacles:
            if hasattr(obs, "radius"):
                self._draw_cylinder(obs)
            else:
                self._draw_box(obs)
        self._draw_goal_marker()

    def _draw_ground(self):
        gx = np.linspace(0, self.map_size[0], 2)
        gy = np.linspace(0, self.map_size[1], 2)
        GX, GY = np.meshgrid(gx, gy)
        Z = np.zeros_like(GX)
        self.ax.plot_surface(GX, GY, Z, color=(0.88, 0.93, 0.98), alpha=0.7, linewidth=0, zorder=0)

    def _draw_bounds(self):
        W, H = self.map_size
        base = np.array([
            [0, 0, 0],
            [W, 0, 0],
            [W, H, 0],
            [0, H, 0],
            [0, 0, 0]
        ])
        self.ax.plot(base[:, 0], base[:, 1], base[:, 2], color="dimgray", lw=1.2)
        self.ax.plot(base[:, 0], base[:, 1], base[:, 2] + 0.02, color="white", lw=0.8, alpha=0.6)

    def _draw_box(self, obs):
        half_w = getattr(obs, "width", 1.0) / 2.0
        half_l = getattr(obs, "length", 1.0) / 2.0
        corners = np.array([
            [-half_w, -half_l],
            [ half_w, -half_l],
            [ half_w,  half_l],
            [-half_w,  half_l],
        ])
        rot = getattr(obs, "rotation", 0.0)
        R = np.array([[np.cos(rot), -np.sin(rot)],
                      [np.sin(rot),  np.cos(rot)]])
        rotated = corners @ R.T + np.array([getattr(obs, "x", 0), getattr(obs, "y", 0)])
        base = np.column_stack([rotated, np.zeros(4)])
        height = getattr(obs, "height", 1.0)
        top = base.copy()
        top[:, 2] += height
        faces = [
            [base[0], base[1], base[2], base[3]],
            [top[0], top[1], top[2], top[3]],
            [base[0], base[1], top[1], top[0]],
            [base[1], base[2], top[2], top[1]],
            [base[2], base[3], top[3], top[2]],
            [base[3], base[0], top[0], top[3]],
        ]
        color = getattr(obs, "color", (0.4, 0.3, 0.2))
        box = Poly3DCollection(faces, facecolor=color, edgecolor="k", linewidth=0.5, alpha=0.75)
        self.ax.add_collection3d(box)

    def _draw_cylinder(self, obs):
        x, y = getattr(obs, "x", 0), getattr(obs, "y", 0)
        radius = getattr(obs, "radius", 0.5)
        height = getattr(obs, "height", 1.0)
        color = getattr(obs, "color", (0.4, 0.3, 0.2))
        theta = np.linspace(0, 2 * np.pi, 32)
        z = np.linspace(0, height, 2)
        Theta, Z = np.meshgrid(theta, z)
        X = radius * np.cos(Theta) + x
        Y = radius * np.sin(Theta) + y
        self.ax.plot_surface(X, Y, Z, color=color, alpha=0.6, linewidth=0)
        self.ax.plot(radius * np.cos(theta) + x, radius * np.sin(theta) + y,
                     np.zeros_like(theta), color=color, alpha=0.8)
        self.ax.plot(radius * np.cos(theta) + x, radius * np.sin(theta) + y,
                     np.full_like(theta, height), color=color, alpha=0.8)

    def _draw_goal_marker(self):
        if self.goal is None:
            return
        gx_goal, gy_goal = self.goal
        self.goal_marker = self.ax.scatter(
            gx_goal, gy_goal, 0.05, color="red", s=90, marker="*", depthshade=False, zorder=5
        )

    def update_path(self, trajectory):
        if len(trajectory) < 2:
            if self.path_line:
                self.path_line.set_data([], [])
                self.path_line.set_3d_properties([])
            return
        traj = np.asarray(trajectory)
        z = np.zeros(len(traj))
        if self.path_line is None:
            (self.path_line,) = self.ax.plot(traj[:, 0], traj[:, 1], z, color="navy", lw=1.8, alpha=0.8)
        else:
            self.path_line.set_data(traj[:, 0], traj[:, 1])
            self.path_line.set_3d_properties(z)

    def update_car(self, x, y, theta):
        L, W, H = 0.6, 0.3, 0.2
        verts = np.array([
            [-L / 2, -W / 2, 0],
            [ L / 2, -W / 2, 0],
            [ L / 2,  W / 2, 0],
            [-L / 2,  W / 2, 0],
            [-L / 2, -W / 2, H],
            [ L / 2, -W / 2, H],
            [ L / 2,  W / 2, H],
            [-L / 2,  W / 2, H],
        ])
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0, 0, 1],
        ])
        verts = verts @ R.T + np.array([x, y, 0])
        faces = [
            [verts[i] for i in [0, 1, 2, 3]],
            [verts[i] for i in [4, 5, 6, 7]],
            [verts[i] for i in [0, 1, 5, 4]],
            [verts[i] for i in [1, 2, 6, 5]],
            [verts[i] for i in [2, 3, 7, 6]],
            [verts[i] for i in [3, 0, 4, 7]],
        ]
        if self.car_box:
            self.car_box.remove()
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        self.car_box = Poly3DCollection(faces, facecolor='skyblue', edgecolor='k', alpha=0.9)
        self.ax.add_collection3d(self.car_box)

        front = np.array([x + np.cos(theta) * L, y + np.sin(theta) * L, H * 0.5])
        if self.heading_line is None:
            (self.heading_line,) = self.ax.plot(
                [x, front[0]], [y, front[1]], [H * 0.5, front[2]], color="orange", lw=2.0
            )
        else:
            self.heading_line.set_data([x, front[0]], [y, front[1]])
            self.heading_line.set_3d_properties([H * 0.5, front[2]])

        self._update_camera(np.array([x, y, 0]), theta)

    def _update_camera(self, focus_point, theta):
        target = np.array([focus_point[0], focus_point[1], 0])
        self.smooth_focus = (1 - self.cam_alpha) * self.smooth_focus + self.cam_alpha * target
        span = 4.5
        x_c = np.clip(self.smooth_focus[0], span, self.map_size[0] - span)
        y_c = np.clip(self.smooth_focus[1], span, self.map_size[1] - span)
        self.ax.set_xlim(x_c - span, x_c + span)
        self.ax.set_ylim(y_c - span, y_c + span)
        self.ax.set_zlim(0, max(2.5, 0.2 * max(self.map_size)))
        self.ax.view_init(elev=30, azim=-np.degrees(theta) + 90)
