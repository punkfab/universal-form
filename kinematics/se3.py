"""Minimal SE(3) / SO(3) helpers — just enough for the kinematic chain and its IK.

Transforms are 4x4 homogeneous numpy arrays; rotations are 3x3. We keep this tiny and
dependency-light (numpy only) on purpose: the kinematics is the thing under test, so its
substrate should be boring and obviously correct.
"""

from __future__ import annotations

import numpy as np


def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def se3(R: np.ndarray, p) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, float)
    return T


def transl(x: float, y: float, z: float) -> np.ndarray:
    return se3(np.eye(3), (x, y, z))


def pose_close(A: np.ndarray, B: np.ndarray, tol: float = 1e-9) -> bool:
    """True iff two SE(3) poses agree in both rotation and translation within `tol`."""
    return bool(np.max(np.abs(A - B)) <= tol)


def frame_from_x(x_axis, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Build a right-handed rotation whose local x-axis points along `x_axis`.

    Used to synthesize a target *orientation* from just an approach direction (the tool
    reaches out along its local x): pick any consistent y/z that complete the frame.
    """
    x = np.asarray(x_axis, float)
    x = x / np.linalg.norm(x)
    up = np.asarray(up, float)
    if abs(float(np.dot(x, up))) > 0.99:  # x nearly parallel to `up` — pick another ref
        up = np.array([1.0, 0.0, 0.0])
    z = up - np.dot(up, x) * x
    z = z / np.linalg.norm(z)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi
