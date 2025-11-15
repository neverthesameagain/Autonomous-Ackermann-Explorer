"""
Primitive 3D shapes for the simulation environment.
Supports: Box, Cylinder, Cone, Hemisphere.
Each primitive provides geometric vertices for rendering
and a `contains(x, y, z)` method for spatial queries.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Optional


# ==========================================================
# Base Class
# ==========================================================
@dataclass
class Primitive:
    """Base class for all primitive shapes."""
    x: float                  # x-coordinate of the center
    y: float                  # y-coordinate of the center
    height: float             # height of the shape along z-axis
    color: Optional[Tuple[float, float, float]] = None  # RGB color (0-1)

    def get_vertices(self) -> List[Tuple[float, float, float]]:
        """Return vertices for rendering."""
        raise NotImplementedError("Subclasses must implement get_vertices()")

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if a point (x, y, z) is inside the shape."""
        raise NotImplementedError("Subclasses must implement contains()")


# ==========================================================
# Box (Rectangular Prism)
# ==========================================================
@dataclass
class Box(Primitive):
    width: float = 1.0     # size along x-axis
    length: float = 1.0    # size along y-axis
    rotation: float = 0.0  # rotation in radians around z-axis

    def get_vertices(self) -> List[Tuple[float, float, float]]:
        """Get the 8 vertices of the box for rendering."""
        hw, hl = self.width / 2, self.length / 2

        # Local vertices (bottom + top faces)
        local_vertices = [
            (-hw, -hl, 0), (hw, -hl, 0), (hw, hl, 0), (-hw, hl, 0),
            (-hw, -hl, self.height), (hw, -hl, self.height),
            (hw, hl, self.height), (-hw, hl, self.height)
        ]

        # Apply rotation + translation
        cos_t, sin_t = np.cos(self.rotation), np.sin(self.rotation)
        world_vertices = []
        for lx, ly, lz in local_vertices:
            wx = self.x + (lx * cos_t - ly * sin_t)
            wy = self.y + (lx * sin_t + ly * cos_t)
            world_vertices.append((wx, wy, lz))
        return world_vertices

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point (x,y,z) lies inside the box."""
        if not (0 <= z <= self.height):
            return False

        # Transform to local coordinates
        dx, dy = x - self.x, y - self.y
        cos_t, sin_t = np.cos(-self.rotation), np.sin(-self.rotation)
        local_x = dx * cos_t - dy * sin_t
        local_y = dx * sin_t + dy * cos_t

        return (abs(local_x) <= self.width / 2) and (abs(local_y) <= self.length / 2)


# ==========================================================
# Cylinder
# ==========================================================
@dataclass
class Cylinder(Primitive):
    radius: float = 0.5

    def get_vertices(self, segments: int = 24) -> List[Tuple[float, float, float]]:
        """Approximate the cylinder surface using circular segments."""
        verts = []
        angles = np.linspace(0, 2 * np.pi, segments, endpoint=False)
        for theta in angles:
            cx = self.x + self.radius * np.cos(theta)
            cy = self.y + self.radius * np.sin(theta)
            verts.append((cx, cy, 0))
            verts.append((cx, cy, self.height))
        return verts

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point is inside the cylinder."""
        if not (0 <= z <= self.height):
            return False
        dx, dy = x - self.x, y - self.y
        return (dx**2 + dy**2) <= (self.radius**2)


# ==========================================================
# Cone
# ==========================================================
@dataclass
class Cone(Primitive):
    radius: float = 0.5

    def get_vertices(self, segments: int = 24) -> List[Tuple[float, float, float]]:
        """Generate vertices for cone surface."""
        verts = []
        apex = (self.x, self.y, self.height)
        verts.append(apex)

        # Base circle
        angles = np.linspace(0, 2 * np.pi, segments + 1)
        for theta in angles:
            bx = self.x + self.radius * np.cos(theta)
            by = self.y + self.radius * np.sin(theta)
            verts.append((bx, by, 0))
        return verts

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point is inside the cone volume."""
        if not (0 <= z <= self.height):
            return False
        dx, dy = x - self.x, y - self.y
        r_at_z = self.radius * (1 - z / self.height)
        return (dx**2 + dy**2) <= r_at_z**2


# ==========================================================
# Hemisphere
# ==========================================================
@dataclass
class Hemisphere(Primitive):
    radius: float = 0.5

    def get_vertices(self, rings: int = 10, segments: int = 24) -> List[Tuple[float, float, float]]:
        """Generate hemisphere surface points."""
        verts = []
        for j in range(rings + 1):
            phi = (np.pi / 2) * (j / rings)   # 0 → π/2
            r = self.radius * np.sin(phi)
            z = self.radius * np.cos(phi)
            for i in range(segments):
                theta = 2 * np.pi * i / segments
                x = self.x + r * np.cos(theta)
                y = self.y + r * np.sin(theta)
                verts.append((x, y, z))
        return verts

    def contains(self, x: float, y: float, z: float) -> bool:
        """Check if point lies within the hemisphere."""
        if not (0 <= z <= self.radius):
            return False
        dx, dy, dz = x - self.x, y - self.y, z
        return (dx**2 + dy**2 + dz**2) <= (self.radius**2)
