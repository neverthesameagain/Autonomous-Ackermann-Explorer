"""
Procedural 3D Environment Generator
------------------------------------
Creates random obstacle distributions with varied shapes, colors, and heights.
Automatically ensures start and goal are placed in free regions.
"""

import numpy as np
import random
from typing import List, Tuple, Optional, Union
from .primitives import Box, Cylinder, Cone, Hemisphere


def generate_environment(
    num_obs: int = 20,
    bounds: Tuple[float, float] = (20.0, 20.0),
    start_pos: Optional[Tuple[float, float]] = None,
    goal_pos: Optional[Tuple[float, float]] = None,
    min_clearance: float = 2.5
) -> List:
    """
    Generate a fully random environment with 3D obstacles.

    Args:
        num_obs: number of obstacles to generate
        bounds: (width, height) bounds of the environment
        start_pos: optional fixed start position (random if None)
        goal_pos: optional fixed goal position (random if None)
        min_clearance: minimum spacing between start/goal and obstacles

    Returns:
        obstacles: list of primitive obstacle objects (Box, Cylinder, Cone, Hemisphere)
    """
    W, H = bounds
    rng = np.random.default_rng()
    obstacles = []
    placed = []  # store (x, y, r_equiv) for spacing checks

    def sample_free_point(min_dist=2.0) -> Tuple[float, float]:
        """Sample a random point that doesn’t collide with any existing obstacle."""
        for _ in range(500):
            x = rng.uniform(1.0, W - 1.0)
            y = rng.uniform(1.0, H - 1.0)
            if all(np.hypot(x - ox, y - oy) > (orad + min_dist) for ox, oy, orad in placed):
                return x, y
        raise RuntimeError("Could not sample free point in environment")

    # === Create random obstacles ===
    for _ in range(num_obs):
        x, y = sample_free_point(min_dist=1.5)
        shape = random.choice(["box", "cylinder", "cone", "hemisphere"])
        color = tuple(rng.uniform(0.2, 0.8, size=3))
        height = rng.uniform(1.0, 2.8)

        if shape == "box":
            width = rng.uniform(1.0, 3.0)
            length = rng.uniform(1.0, 3.0)
            rotation = rng.uniform(0, np.pi)
            obstacles.append(Box(x=x, y=y, width=width, length=length, height=height,
                                 rotation=rotation, color=color))
            r_equiv = max(width, length) / 2.0
        elif shape == "cylinder":
            radius = rng.uniform(0.5, 1.5)
            obstacles.append(Cylinder(x=x, y=y, radius=radius, height=height, color=color))
            r_equiv = radius
        elif shape == "cone":
            radius = rng.uniform(0.5, 1.2)
            obstacles.append(Cone(x=x, y=y, radius=radius, height=height, color=color))
            r_equiv = radius
        else:
            radius = rng.uniform(0.6, 1.2)
            obstacles.append(Hemisphere(x=x, y=y, radius=radius, height=height, color=color))
            r_equiv = radius

        placed.append((x, y, r_equiv))

    # === Randomize start and goal if not provided ===
    if start_pos is None:
        start_pos = sample_free_point(min_dist=min_clearance)
    if goal_pos is None:
        for _ in range(200):
            gx, gy = sample_free_point(min_dist=min_clearance)
            if np.hypot(gx - start_pos[0], gy - start_pos[1]) > (W + H) / 6.0:
                goal_pos = (gx, gy)
                break
        if goal_pos is None:
            goal_pos = sample_free_point(min_dist=min_clearance)

    # Attach start/goal to environment info (for convenience)
    generate_environment.last_start = start_pos
    generate_environment.last_goal = goal_pos

    return obstacles
