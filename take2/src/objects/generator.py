"""
Procedural 3D Environment Generator (small-map safe)
----------------------------------------------------
• Random mix of Box / Cylinder / Cone / Hemisphere
• Start/Goal always sampled in free space with clearance
• Edge buffer + obstacle spacing respected
• Adaptive obstacle sizes (scale with bounds)
• Graceful relaxation if space is tight (e.g., 9x9 worlds)
"""

import numpy as np
import random
from typing import List, Tuple, Optional
from .primitives import Box, Cylinder, Cone, Hemisphere


def generate_environment(
    num_obs: int = 7,
    bounds: Tuple[float, float] = (10.0, 10.0),
    start_pos: Optional[Tuple[float, float]] = None,
    goal_pos: Optional[Tuple[float, float]] = None,
    min_clearance: float = 0.5,
    edge_buffer: float = 0.4,
    rng_seed: Optional[int] = None,
) -> List:
    """
    Generate a random environment with 3D obstacles that works even on tiny maps.

    Args:
        num_obs: nominal obstacle count (may be reduced automatically if too dense)
        bounds: (W, H) world size in meters
        start_pos: fixed start (random if None)
        goal_pos: fixed goal  (random if None)
        min_clearance: req. spacing between start/goal and any obstacle (m)
        edge_buffer: min distance from walls for any sampled point (m)
        rng_seed: optional seed for reproducibility

    Returns:
        obstacles: list of primitive obstacle objects.
    Side-effects:
        generate_environment.last_start / last_goal store the chosen start/goal.
    """
    W, H = float(bounds[0]), float(bounds[1])
    rng = np.random.default_rng(rng_seed)
    obstacles: List = []
    placed = []  # (x, y, r_equiv)

    # ---- Adaptive sizing for small worlds ----
    s = min(W, H)
    # Cap obstacle count so we don't over-pack small maps
    max_obs_by_area = max(1, int((W * H) / (s * s * 0.12)))  # rough density cap
    num_obs = int(min(num_obs, max_obs_by_area))

    # Base size ranges (scale with map size)
    box_w_range = (0.12 * s, 0.28 * s)
    box_l_range = (0.12 * s, 0.28 * s)
    cyl_r_range = (0.06 * s, 0.16 * s)
    cone_r_range = (0.06 * s, 0.14 * s)
    hemi_r_range = (0.06 * s, 0.14 * s)
    h_range = (0.20 * s, 0.35 * s)  # heights just for 3D viz

    # ---- Helpers ----
    def valid_against_obstacles(x: float, y: float, min_dist: float) -> bool:
        for ox, oy, orad in placed:
            if np.hypot(x - ox, y - oy) <= (orad + min_dist):
                return False
        return True

    def within_bounds(x: float, y: float, buf: float) -> bool:
        return (buf <= x <= W - buf) and (buf <= y <= H - buf)

    def sample_free_point(min_dist: float, buf: float, attempts: int = 700) -> Optional[Tuple[float, float]]:
        for _ in range(attempts):
            x = rng.uniform(buf, W - buf)
            y = rng.uniform(buf, H - buf)
            if valid_against_obstacles(x, y, min_dist):
                return (x, y)
        return None

    # Progressive relaxation if sampling fails
    def sample_free_point_relaxed(min_dist: float, buf: float) -> Tuple[float, float]:
        md = float(min_dist)
        eb = float(buf)
        for _ in range(5):  # up to 5 relaxation rounds
            p = sample_free_point(md, eb)
            if p is not None:
                return p
            md = max(0.1, md * 0.75)  # relax spacing
            eb = max(0.1, eb * 0.75)  # relax wall buffer
        # final brute-force attempt
        p = sample_free_point(md, eb, attempts=1500)
        if p is None:
            # As an absolute fallback, drop spacing entirely but keep inside bounds
            x = rng.uniform(0.1, W - 0.1)
            y = rng.uniform(0.1, H - 0.1)
            return (x, y)
        return p

    # ---- Create random obstacles ----
    for _ in range(num_obs):
        x, y = sample_free_point_relaxed(min_dist=0.5 * s * 0.08 + 0.2, buf=edge_buffer)

        shape = random.choice(["box", "cylinder", "cone", "hemisphere"])
        color = tuple(rng.uniform(0.2, 0.8, size=3))
        height = float(rng.uniform(*h_range))

        if shape == "box":
            width = float(rng.uniform(*box_w_range))
            length = float(rng.uniform(*box_l_range))
            rotation = float(rng.uniform(0, np.pi))
            obstacles.append(
                Box(x=x, y=y, width=width, length=length, height=height, rotation=rotation, color=color)
            )
            r_equiv = max(width, length) / 2.0

        elif shape == "cylinder":
            radius = float(rng.uniform(*cyl_r_range))
            obstacles.append(Cylinder(x=x, y=y, radius=radius, height=height, color=color))
            r_equiv = radius

        elif shape == "cone":
            radius = float(rng.uniform(*cone_r_range))
            obstacles.append(Cone(x=x, y=y, radius=radius, height=height, color=color))
            r_equiv = radius

        else:  # hemisphere
            radius = float(rng.uniform(*hemi_r_range))
            obstacles.append(Hemisphere(x=x, y=y, radius=radius, height=height, color=color))
            r_equiv = radius

        placed.append((x, y, r_equiv))

    # ---- Sample Start / Goal in free space (not too close to each other) ----
    min_sg = max(min_clearance, 0.12 * s)  # ensure some separation on tiny maps
    if start_pos is None:
        start_pos = sample_free_point_relaxed(min_dist=min_clearance, buf=edge_buffer)
    if goal_pos is None:
        # try to keep a reasonable separation from start
        for _ in range(700):
            gx, gy = sample_free_point_relaxed(min_dist=min_clearance, buf=edge_buffer)
            if np.hypot(gx - start_pos[0], gy - start_pos[1]) >= min_sg * 2.0:
                goal_pos = (gx, gy)
                break
        if goal_pos is None:
            # final fallback
            gx, gy = sample_free_point_relaxed(min_dist=min_clearance * 0.7, buf=edge_buffer * 0.7)
            goal_pos = (gx, gy)

    # expose start/goal like before
    generate_environment.last_start = start_pos
    generate_environment.last_goal = goal_pos
    return obstacles
