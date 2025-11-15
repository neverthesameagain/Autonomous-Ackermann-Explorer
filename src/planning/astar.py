"""
A* Path Planner (robust version)
--------------------------------
✓ Grid ↔ world coordinate consistency
✓ Inflation fallback for tight maps
✓ Line-of-sight smoothing with inflated obstacle checks
✓ Path densification for stable Ackermann/Pure-Pursuit control
✓ Informative debug logging
"""

import heapq
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass(order=True)
class Node:
    f: float
    g: float = field(compare=False)
    pos: Tuple[int, int] = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)


class AStarPlanner:
    """A* path planner operating on an OccupancyGrid."""

    def __init__(self, occupancy_grid):
        self.grid = occupancy_grid
        self.motions = [
            (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]
        self._inflated_mask = None

    # ==========================================================
    #                    PUBLIC API
    # ==========================================================
    def plan(self, start_x: float, start_y: float,
             goal_x: float, goal_y: float,
             inflation_radius: int = 2) -> Optional[List[Tuple[float, float]]]:
        """Plan a collision-free path in world coordinates."""

        start_gx, start_gy = self.grid.world_to_grid(start_x, start_y)
        goal_gx, goal_gy = self.grid.world_to_grid(goal_x, goal_y)

        if not self.grid.is_valid(start_gx, start_gy) or not self.grid.is_valid(goal_gx, goal_gy):
            print("⚠️ Start or goal out of bounds.")
            return None

        # Inflate obstacles and ensure start/goal remain feasible
        inflated = self._inflate_obstacles(inflation_radius)
        used_radius = inflation_radius

        inflated, used_radius = self._ensure_cell_is_free(
            inflated, used_radius, start_gx, start_gy, label="Start"
        )
        if inflated is None:
            return None

        inflated, used_radius = self._ensure_cell_is_free(
            inflated, used_radius, goal_gx, goal_gy, label="Goal"
        )
        if inflated is None:
            return None

        self._inflated_mask = inflated.copy()

        # --- A* Search ---
        open_set, closed_set = [], set()
        start_node = Node(f=self._heuristic(start_gx, start_gy, goal_gx, goal_gy),
                          g=0, pos=(start_gx, start_gy))
        heapq.heappush(open_set, start_node)
        g_score = {(start_gx, start_gy): 0}

        while open_set:
            current = heapq.heappop(open_set)
            if current.pos in closed_set:
                continue

            if current.pos == (goal_gx, goal_gy):
                path = self._reconstruct_path(current)
                if len(path) <= 3:
                    print(f"⚠️ A* produced very short path ({len(path)} pts).")
                return path

            closed_set.add(current.pos)

            for dx, dy, cost in self.motions:
                nx, ny = current.pos[0] + dx, current.pos[1] + dy
                if not self.grid.is_valid(nx, ny):
                    continue
                if inflated[ny, nx]:
                    continue
                tentative_g = current.g + cost
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + self._heuristic(nx, ny, goal_gx, goal_gy)
                    heapq.heappush(open_set, Node(f=f, g=tentative_g, pos=(nx, ny), parent=current))

        print("✗ No path found by A*.")
        return None

    # ==========================================================
    #                    INTERNAL HELPERS
    # ==========================================================

    def _heuristic(self, x1, y1, x2, y2):
        return np.hypot(x1 - x2, y1 - y2)

    def _inflate_obstacles(self, radius: int):
        """Return a binary mask with inflated obstacles."""
        from scipy.ndimage import binary_dilation
        occ = self.grid.grid > 0.65
        if radius <= 0:
            return occ
        struct = np.ones((2 * radius + 1, 2 * radius + 1))
        return binary_dilation(occ, structure=struct)

    def _reconstruct_path(self, node) -> List[Tuple[float, float]]:
        """Rebuild world-coordinate path from goal to start."""
        path = []
        while node:
            wx, wy = self.grid.grid_to_world(node.pos[0], node.pos[1])
            path.append((wx, wy))
            node = node.parent
        path.reverse()
        path = self._smooth_path(path)
        return self._densify_path(path, step=max(0.75 * self.grid.resolution, 0.1))

    def _ensure_cell_is_free(self, inflated, current_radius, gx, gy, label="Cell"):
        """Ensure a particular cell remains free after inflation, reducing radius if needed."""
        if not inflated[gy, gx]:
            return inflated, current_radius

        print(f"⚠️ {label} cell blocked after inflation → reducing inflation radius...")
        for r in range(current_radius - 1, -1, -1):
            new_mask = self._inflate_obstacles(r)
            if not new_mask[gy, gx]:
                print(f"  ✓ {label} recovered with inflation={r}")
                return new_mask, r

        print(f"✗ {label} cell still blocked → aborting.")
        return None, None

    def _smooth_path(self, path: List[Tuple[float, float]], max_iterations: int = 100):
        """Shortcut smoothing using line-of-sight tests on inflated map."""
        if len(path) <= 2:
            return path
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            for j in range(len(path) - 1, i, -1):
                if self._is_line_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
        return smoothed

    def _is_line_free(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """Check if the straight segment p1→p2 is free of inflated obstacles."""
        if self._inflated_mask is None:
            return True
        x1, y1 = self.grid.world_to_grid(*p1)
        x2, y2 = self.grid.world_to_grid(*p2)
        dx, dy = abs(x2 - x1), abs(y2 - y1)
        sx, sy = (1 if x1 < x2 else -1), (1 if y1 < y2 else -1)
        err = dx - dy
        H, W = self._inflated_mask.shape
        while True:
            if not (0 <= x1 < W and 0 <= y1 < H):
                return False
            if self._inflated_mask[y1, x1]:
                return False
            if (x1, y1) == (x2, y2):
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return True

    def _densify_path(self, path: List[Tuple[float, float]], step: float = 0.1) -> List[Tuple[float, float]]:
        """Interpolate intermediate points for smooth Ackermann following."""
        if len(path) < 2:
            return path
        out = [path[0]]
        for a, b in zip(path, path[1:]):
            ax, ay = a
            bx, by = b
            seg_len = np.hypot(bx - ax, by - ay)
            n_steps = max(1, int(seg_len / step))
            for k in range(1, n_steps + 1):
                t = k / n_steps
                out.append((ax + t * (bx - ax), ay + t * (by - ay)))
        return out
