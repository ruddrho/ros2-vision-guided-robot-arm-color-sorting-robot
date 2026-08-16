#!/usr/bin/env python3
"""Generate the lightweight industrial-arm visual meshes used by the URDF.

The meshes are visual shells only.  Kinematic frames, collision primitives,
joint limits, ros2_control interfaces, and the calibrated workcell remain in
the Xacro model so simulation behavior stays reproducible.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MESH_DIRECTORY = ROOT / "meshes"


def superellipse_ring(radius_a: float, radius_b: float, count: int) -> np.ndarray:
    """Return a rounded-rectangle ring in its local two-dimensional plane."""
    points = []
    for index in range(count):
        angle = 2.0 * math.pi * index / count
        cosine = math.cos(angle)
        sine = math.sin(angle)
        first = radius_a * math.copysign(math.sqrt(abs(cosine)), cosine)
        second = radius_b * math.copysign(math.sqrt(abs(sine)), sine)
        points.append((first, second))
    return np.asarray(points, dtype=float)


def loft_mesh(
    axis: str,
    sections: Sequence[tuple[float, float, float]],
    ring_points: int = 32,
) -> tuple[np.ndarray, np.ndarray]:
    """Create a capped, tapered superellipse loft along the requested axis."""
    vertices: list[tuple[float, float, float]] = []
    for position, radius_a, radius_b in sections:
        for first, second in superellipse_ring(radius_a, radius_b, ring_points):
            if axis == "x":
                vertices.append((position, first, second))
            elif axis == "z":
                vertices.append((first, second, position))
            else:
                raise ValueError(f"Unsupported loft axis: {axis}")

    faces: list[tuple[int, int, int]] = []
    for section_index in range(len(sections) - 1):
        current = section_index * ring_points
        following = (section_index + 1) * ring_points
        for point_index in range(ring_points):
            next_index = (point_index + 1) % ring_points
            faces.append((current + point_index, following + point_index, following + next_index))
            faces.append((current + point_index, following + next_index, current + next_index))

    start_center = len(vertices)
    end_center = start_center + 1
    first_position = sections[0][0]
    last_position = sections[-1][0]
    if axis == "x":
        vertices.extend(((first_position, 0.0, 0.0), (last_position, 0.0, 0.0)))
    else:
        vertices.extend(((0.0, 0.0, first_position), (0.0, 0.0, last_position)))

    final_ring = (len(sections) - 1) * ring_points
    for point_index in range(ring_points):
        next_index = (point_index + 1) % ring_points
        faces.append((start_center, next_index, point_index))
        faces.append((end_center, final_ring + point_index, final_ring + next_index))

    return np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int)


def normal_for_triangle(vertices: np.ndarray, face: Iterable[int]) -> np.ndarray:
    first, second, third = (vertices[index] for index in face)
    normal = np.cross(second - first, third - first)
    length = np.linalg.norm(normal)
    return normal / length if length > 1.0e-12 else np.zeros(3)


def write_ascii_stl(name: str, vertices: np.ndarray, faces: np.ndarray) -> None:
    path = MESH_DIRECTORY / name
    with path.open("w", encoding="ascii", newline="\n") as output:
        output.write(f"solid {path.stem}\n")
        for face in faces:
            normal = normal_for_triangle(vertices, face)
            output.write(
                "  facet normal "
                f"{normal[0]:.9g} {normal[1]:.9g} {normal[2]:.9g}\n"
            )
            output.write("    outer loop\n")
            for vertex_index in face:
                vertex = vertices[vertex_index]
                output.write(
                    "      vertex "
                    f"{vertex[0]:.9g} {vertex[1]:.9g} {vertex[2]:.9g}\n"
                )
            output.write("    endloop\n  endfacet\n")
        output.write(f"endsolid {path.stem}\n")


def main() -> None:
    MESH_DIRECTORY.mkdir(parents=True, exist_ok=True)
    specifications = {
        "upper_arm_shell.stl": (
            "x",
            (
                (0.000, 0.062, 0.058),
                (0.035, 0.068, 0.061),
                (0.155, 0.059, 0.052),
                (0.275, 0.049, 0.044),
                (0.320, 0.052, 0.047),
            ),
        ),
        "forearm_shell.stl": (
            "x",
            (
                (0.000, 0.058, 0.054),
                (0.030, 0.063, 0.057),
                (0.135, 0.052, 0.047),
                (0.235, 0.043, 0.039),
                (0.280, 0.047, 0.042),
            ),
        ),
        "wrist_column_shell.stl": (
            "z",
            (
                (0.000, 0.049, 0.044),
                (0.025, 0.052, 0.046),
                (0.110, 0.040, 0.035),
                (0.195, 0.042, 0.037),
                (0.220, 0.047, 0.042),
            ),
        ),
        "wrist_extension_shell.stl": (
            "z",
            (
                (0.000, 0.038, 0.034),
                (0.020, 0.040, 0.036),
                (0.080, 0.032, 0.028),
                (0.100, 0.035, 0.031),
            ),
        ),
        "gripper_palm_shell.stl": (
            "z",
            (
                (0.000, 0.047, 0.055),
                (0.012, 0.050, 0.058),
                (0.050, 0.046, 0.054),
                (0.067, 0.040, 0.048),
            ),
        ),
        "gripper_finger_shell.stl": (
            "z",
            (
                (0.000, 0.015, 0.012),
                (0.018, 0.016, 0.013),
                (0.105, 0.014, 0.011),
                (0.140, 0.011, 0.009),
            ),
        ),
    }

    for filename, (axis, sections) in specifications.items():
        vertices, faces = loft_mesh(axis, sections)
        write_ascii_stl(filename, vertices, faces)
        bounds = np.vstack((vertices.min(axis=0), vertices.max(axis=0)))
        print(f"generated {filename}: bounds={bounds.tolist()}, triangles={len(faces)}")


if __name__ == "__main__":
    main()
