#!/usr/bin/env python3
"""Gate: "base delivers, arm settles" -- the base+lift buy reach the arm alone cannot.

Two things proven here, both runnable:

  A. Workspace honesty. The arm's reach is a spherical shell around the shoulder. We assert IK
     succeeds exactly on targets inside the shell and fails cleanly outside it -- never a silent
     wrong answer. (The same honesty ik_check.py checks, stated as a workspace here.)

  B. Redundancy resolution. We scatter targets across a room-sized volume, well outside any one
     arm-pose's reach, and show:
        * the arm bolted to a *fixed* shoulder solves almost none of them, but
        * the full form -- free to drive its holonomic base and raise its lift -- places the
          shoulder one arm's-length away and then solves them with closed-form arm IK,
     the placed solution's FK lands on the target to 1e-9, and every target the form *refuses*
     is legitimately above its vertical ceiling -- horizontal reach is free (drive there),
     vertical reach is not (README: "vertical reach must live in the body"). The form never
     refuses a target inside its envelope.

That is the whole point of making locomotion part of the kinematic chain.
"""

from __future__ import annotations

import sys

import numpy as np

sys.path.insert(0, ".")
from kinematics.arm import Arm                    # noqa: E402
from kinematics.form import UniversalForm         # noqa: E402
from kinematics.se3 import frame_from_x, se3      # noqa: E402


def main() -> int:
    rng = np.random.default_rng(7142026)
    arm = Arm()
    form = UniversalForm(arm=arm)

    # --- A. workspace honesty: in-shell solvable, out-of-shell cleanly empty ----------
    honesty_bad = 0
    for _ in range(6000):
        d = rng.uniform(0.0, arm.reach_max * 1.4)           # distance shoulder->wrist center
        u = rng.normal(size=3)
        wc = d * u / np.linalg.norm(u)
        R = frame_from_x(rng.normal(size=3))
        p = wc + arm.Lt * R[:, 0]
        sols = arm.ik(se3(R, p))
        inside = arm.reach_min + 1e-3 <= d <= arm.reach_max - 1e-3
        if inside and not sols:
            honesty_bad += 1
        if not (arm.reach_min - 1e-3 <= d <= arm.reach_max + 1e-3) and sols:
            honesty_bad += 1

    # --- B. redundancy: fixed arm vs. mobile form over a room --------------------------
    M = 3000
    approach = np.array([0.0, 0.0, -1.0])          # reach straight down onto the target
    ceiling = form.vertical_ceiling(approach)      # the honest vertical limit
    margin = 0.05                                  # classify just inside/outside to avoid boundary noise

    fixed_hits = 0
    in_env = 0            # targets inside the vertical envelope
    in_env_solved = 0
    wrongful_refusals = 0  # refused a target that is inside the envelope -> a real failure
    bad_refusals = 0       # "solved" a target above the ceiling -> overreach
    worst_land = 0.0
    land_bad = 0
    # A shoulder bolted at the base's default stance (base at origin, no lift, zero yaw).
    Sh_fixed = form.shoulder_pose(0.0, 0.0, 0.0, 0.0)

    for _ in range(M):
        # targets anywhere in a 6 m x 6 m room, floor to 1.8 m -- deliberately past the ceiling
        target = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(0.0, 1.8)])
        inside = target[2] <= ceiling - margin
        above = target[2] >= ceiling + margin

        # fixed arm: can it reach this world point from the bolted shoulder?
        pose_world = se3(frame_from_x(approach), target)
        if arm.ik(np.linalg.inv(Sh_fixed) @ pose_world):
            fixed_hits += 1

        # mobile form: drive + lift to deliver, then settle
        placed = form.place_for(target, approach=approach)
        if inside:
            in_env += 1
            if placed is None:
                wrongful_refusals += 1
        if above and placed is not None:
            bad_refusals += 1

        if placed is not None:
            base, lift, q = placed
            land = float(np.max(np.abs(form.fk(base, lift, q)[:3, 3] - target)))
            worst_land = max(worst_land, land)
            if land > 1e-9:
                land_bad += 1
            elif inside:
                in_env_solved += 1

    fixed_pct = 100 * fixed_hits / M
    env_pct = 100 * in_env_solved / in_env if in_env else 0.0

    print("A. workspace honesty:")
    print(f"   {honesty_bad} shell violations over 6000 targets (want 0)")
    print(f"B. base delivers, arm settles  ({M} targets in a 6x6 m room, floor..1.8 m):")
    print(f"   vertical ceiling         {ceiling:.2f} m   (horizontal reach is unbounded)")
    print(f"   fixed shoulder solved    {fixed_hits:4d}/{M}      = {fixed_pct:5.1f}%")
    print(f"   mobile form, in-envelope {in_env_solved:4d}/{in_env:<4d} = {env_pct:5.1f}%")
    print(f"   worst landing error      {worst_land:.2e}   (placed FK vs target)")
    print(f"   mis-landed placements    {land_bad}")
    print(f"   wrongful refusals        {wrongful_refusals}   (in-envelope target refused)")
    print(f"   overreach                {bad_refusals}   (target above ceiling 'solved')")

    ok = (honesty_bad == 0 and land_bad == 0
          and wrongful_refusals == 0        # never refuse a target inside the envelope
          and bad_refusals == 0             # never claim one above the ceiling
          and env_pct > 99.0                # solve ~everything within vertical reach
          and env_pct > fixed_pct + 40)     # ... vastly more than the bolted-down arm
    print("\nREACH-CHECK:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
