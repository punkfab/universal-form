"""The 6-DOF arm: an anthropomorphic 3R positioner + a spherical 3R wrist.

This is the README's "add the spherical wrist biology skipped." The three wrist axes
intersect at a single point (the *wrist center*), which is Pieper's criterion — so inverse
kinematics **decouples**: joints 1-3 solve the wrist-center *position*, joints 4-6 solve the
tool *orientation*, and the whole thing is closed-form (no numerical search).

Geometry (offsets = 0, the clean elbow-arm case), all rotations in the moving frame:

    q1  shoulder yaw     about local z
    q2  shoulder pitch   about local y     --- these two are the 3-DOF "shoulder ball"
    --- upper arm, length L1, along local x
    q3  elbow  pitch     about local y     --- the one-way hinge (see README: the elbow tell)
    --- forearm, length L2, along local x, ending at the WRIST CENTER
    q4  wrist roll       about local x      \
    q5  wrist pitch      about local y       }- spherical wrist, axes meet at wrist center
    q6  wrist roll       about local x      /
    --- tool, length Lt, along local x, ending at the TCP

q2 and q3 pitch about parallel axes, so the arm bends in a plane -- exactly a human arm.
Because FK and IK below are derived from *these same* equations, they are consistent by
construction; `scripts/ik_check.py` proves it to machine precision over random configs.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .se3 import rot_x, rot_y, rot_z, se3, wrap


@dataclass(frozen=True)
class Arm:
    L1: float = 0.30   # upper arm (shoulder -> elbow)
    L2: float = 0.28   # forearm (elbow -> wrist center)
    Lt: float = 0.10   # tool (wrist center -> TCP), along the final x-axis

    # --- reachability, stated honestly as a spherical shell around the shoulder ---
    @property
    def reach_max(self) -> float:
        return self.L1 + self.L2

    @property
    def reach_min(self) -> float:
        return abs(self.L1 - self.L2)

    # ------------------------------------------------------------------ forward
    def R03(self, q1: float, q2: float, q3: float) -> np.ndarray:
        """Orientation of the forearm frame: Rz(q1) . Ry(q2+q3)."""
        return rot_z(q1) @ rot_y(q2 + q3)

    def wrist_center(self, q1: float, q2: float, q3: float) -> np.ndarray:
        """Position of the wrist center from joints 1-3 only (Pieper decoupling)."""
        X = self.L1 * np.cos(q2) + self.L2 * np.cos(q2 + q3)   # radial reach
        Z = -(self.L1 * np.sin(q2) + self.L2 * np.sin(q2 + q3))  # +pitch reaches downward
        return np.array([np.cos(q1) * X, np.sin(q1) * X, Z])

    def fk(self, q) -> np.ndarray:
        """Tool pose (4x4) in the shoulder frame for joint vector q = [q1..q6]."""
        q1, q2, q3, q4, q5, q6 = q
        R = self.R03(q1, q2, q3) @ (rot_x(q4) @ rot_y(q5) @ rot_x(q6))
        wc = self.wrist_center(q1, q2, q3)
        p = wc + self.Lt * R[:, 0]   # tool extends along the final local x-axis
        return se3(R, p)

    # ------------------------------------------------------------------ inverse
    def ik(self, pose: np.ndarray, tol: float = 1e-7):
        """All closed-form solutions q=[q1..q6] whose FK reproduces `pose`.

        Returns a list (possibly empty if the target is outside the reachable shell). Up to
        8 branches: 2 shoulder (front/back) x 2 elbow (up/down) x 2 wrist (flip). Every
        returned solution is FK-validated against the target to `tol`, so the caller can
        trust the set -- unreachable or numerically degenerate candidates are dropped.
        """
        R = np.asarray(pose, float)[:3, :3]
        p = np.asarray(pose, float)[:3, 3]
        wc = p - self.Lt * R[:, 0]          # back off the tool -> wrist center

        cands: list[np.ndarray] = []
        for q1 in (np.arctan2(wc[1], wc[0]), np.arctan2(wc[1], wc[0]) + np.pi):
            # radial coord in this shoulder plane (signed: negative for the "reach behind" branch)
            r = wc[0] * np.cos(q1) + wc[1] * np.sin(q1)
            zz = -wc[2]
            D2 = r * r + zz * zz
            c3 = (D2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            if c3 < -1.0 - 1e-9 or c3 > 1.0 + 1e-9:
                continue                     # outside the reachable shell -- honest miss
            c3 = float(np.clip(c3, -1.0, 1.0))
            for q3 in (np.arccos(c3), -np.arccos(c3)):        # elbow up / down
                q2 = np.arctan2(zz, r) - np.arctan2(self.L2 * np.sin(q3),
                                                    self.L1 + self.L2 * np.cos(q3))
                # orientation left over for the wrist: R36 = R03^T . R
                M = self.R03(q1, q2, q3).T @ R
                for q4, q5, q6 in _wrist_xyx(M):              # wrist / wrist-flip
                    cands.append(np.array([q1, q2, q3, q4, q5, q6]))

        # keep only FK-exact solutions, deduped in (wrapped) joint space
        out: list[np.ndarray] = []
        for q in cands:
            if np.max(np.abs(self.fk(q) - pose)) > tol:
                continue
            qn = wrap(q)
            if not any(np.max(np.abs(wrap(qn - s))) < 1e-6 for s in out):
                out.append(qn)
        return out


def _wrist_xyx(M: np.ndarray):
    """Solve Rx(q4).Ry(q5).Rx(q6) = M for the two Euler branches (X-Y-X, repeated axis).

    Derived directly from the symbolic product; the two solutions are
    (a, b, c) and (a+pi, -b, c+pi). At the gimbal (b ~ 0 or pi) the split degenerates, so we
    emit a single q4=0 fallback and let ik()'s FK-validation accept or drop it.
    """
    b = np.arccos(float(np.clip(M[0, 0], -1.0, 1.0)))        # q5 in [0, pi]
    sb = np.sin(b)
    if sb < 1e-9:                                            # gimbal lock
        # b ~ 0: M = Rx(a+c); b ~ pi: M = Rx(a-c).Ry(pi). Pin q4=0, solve the residual roll.
        q6 = np.arctan2(M[2, 1], M[1, 1])
        return [(0.0, b, q6)]
    a = np.arctan2(M[1, 0], -M[2, 0])
    c = np.arctan2(M[0, 1], M[0, 2])
    return [(a, b, c), (a + np.pi, -b, c + np.pi)]
