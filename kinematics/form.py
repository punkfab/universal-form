"""The whole minimal form: holonomic base (x, y, yaw) + torso lift (z) + the 6-DOF arm.

This is where the README's two structural claims become code you can run:

  * "Locomotion is the ultimate extra DOF" -- the base's 3 planar DOF and the lift's 1 vertical
    DOF sit *proximal* to the arm and are part of the same kinematic chain.
  * "Base delivers, arm settles" -- `place_for()` uses the coarse, unlimited-range base+lift to
    park the *shoulder* one arm's-length from the target, then hands the precise final settling
    to the arm's closed-form IK. Reach that the arm alone cannot achieve becomes reachable once
    the base repositions.

`place_for()` is a first-cut *placement*, not a general motion planner: no obstacles, no base
footprint, no collision. It exists to demonstrate the redundancy resolution, and `reach_check.py`
holds it to that.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .arm import Arm
from .se3 import frame_from_x, rot_z, se3


@dataclass(frozen=True)
class UniversalForm:
    arm: Arm = field(default_factory=Arm)
    base_height: float = 0.40   # deck height: shoulder sits here at zero lift
    lift_max: float = 0.55      # torso lift travel -- the "squat" DOF (vertical reach)
    shoulder_offset: tuple = (0.0, 0.18, 0.0)  # shoulder mount in the base frame (right of center)

    # ------------------------------------------------------ forward kinematics
    def shoulder_pose(self, x: float, y: float, yaw: float, lift: float) -> np.ndarray:
        """World pose of the shoulder frame given the base pose and lift."""
        Rz = rot_z(yaw)
        off = Rz @ np.asarray(self.shoulder_offset, float)
        p = np.array([x, y, 0.0]) + off + np.array([0.0, 0.0, self.base_height + lift])
        return se3(Rz, p)

    def fk(self, base, lift: float, q_arm) -> np.ndarray:
        """World TCP pose. `base` = (x, y, yaw); `q_arm` = the 6 arm joints."""
        x, y, yaw = base
        return self.shoulder_pose(x, y, yaw, lift) @ self.arm.fk(q_arm)

    # ------------------------------------------------ base delivers, arm settles
    def vertical_ceiling(self, approach=(0.0, 0.0, -1.0)) -> float:
        """Highest target z this form can reach with the given approach -- the *anisotropy*.

        Horizontal reach is unbounded (the base drives anywhere); vertical is not. The shoulder
        tops out at base_height + lift_max, and from there the arm's shell adds reach_max, minus
        the tool offset carried along the approach. This is the README's "vertical reach has no
        base solution and must live in the body."
        """
        a = np.asarray(approach, float)
        a = a / np.linalg.norm(a)
        shoulder_top = self.base_height + self.lift_max
        # wc_z tops out at shoulder_top + reach_max; TCP = wc + Lt*approach, so TCP_z = wc_z + Lt*a_z.
        return shoulder_top + self.arm.reach_max + self.arm.Lt * a[2]

    def place_for(self, target, approach=(0.0, 0.0, -1.0), dh_frac: float = 0.45):
        """Park the base+lift so `target` is comfortably inside the arm's shell, then IK.

        `target`     : world xyz the TCP should reach.
        `approach`   : desired tool direction (tool reaches along its local x). Default: reach
                       straight down onto the target, the common table-pick case.
        Returns (base=(x,y,yaw), lift, q_arm) or None -- None means the target is genuinely
        outside the form's envelope (above `vertical_ceiling`), the honest anisotropy limit.
        """
        target = np.asarray(target, float)
        a = np.asarray(approach, float)
        a = a / np.linalg.norm(a)

        # The arm's reachable shell is centered on the WRIST CENTER, not the TCP: the tool carries
        # the TCP a length Lt further along the approach. Place the wrist center in the shell.
        wc = target - self.arm.Lt * a

        # 1) Vertical: use the lift to bring the shoulder to the wrist center's height if possible.
        h_shoulder = float(np.clip(wc[2], self.base_height, self.base_height + self.lift_max))
        lift = h_shoulder - self.base_height
        dz = wc[2] - h_shoulder                           # vertical residual the arm must cover

        # 2) Range: pick a shoulder->wrist-center distance in the shell -- prefer a comfortable
        #    horizontal standoff, but grow it toward reach_max when dz needs the vertical room.
        dh_pref = dh_frac * self.arm.reach_max
        standoff = float(np.hypot(dz, dh_pref))
        standoff = float(np.clip(standoff, self.arm.reach_min + 1e-3, self.arm.reach_max * 0.99))
        if standoff <= abs(dz):                            # beyond vertical reach -> honest miss
            return None
        dh = float(np.sqrt(standoff**2 - dz**2))          # horizontal standoff that fits the shell

        # 3) Horizontal: place the shoulder `dh` from the wrist center, along approach's horizontal.
        a_h = np.array([a[0], a[1], 0.0])
        if np.linalg.norm(a_h) < 1e-6:                    # straight-down approach: pick +x to back off
            a_h = np.array([1.0, 0.0, 0.0])
        a_h = a_h / np.linalg.norm(a_h)
        shoulder_xy = wc[:2] - dh * a_h[:2]

        # 4) Yaw the base so the shoulder faces the wrist center; undo the shoulder mount offset.
        yaw = float(np.arctan2(wc[1] - shoulder_xy[1], wc[0] - shoulder_xy[0]))
        off = (rot_z(yaw) @ np.asarray(self.shoulder_offset, float))[:2]
        base = (float(shoulder_xy[0] - off[0]), float(shoulder_xy[1] - off[1]), yaw)

        # 5) Arm settles: closed-form IK in the shoulder frame for the full target pose.
        R_target = frame_from_x(a)                        # tool x-axis along the approach
        Sh = self.shoulder_pose(base[0], base[1], base[2], lift)
        pose_in_shoulder = np.linalg.inv(Sh) @ se3(R_target, target)
        sols = self.arm.ik(pose_in_shoulder)
        if not sols:
            return None
        return base, lift, sols[0]
