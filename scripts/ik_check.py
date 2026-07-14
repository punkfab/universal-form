#!/usr/bin/env python3
"""Gate: the arm's closed-form IK exactly inverts its FK.

The claim in the README is "add the spherical wrist biology skipped -> closed-form IK." This
gate is that claim made runnable. For thousands of random joint configs q:

  1. pose = FK(q)
  2. sols = IK(pose)          -- WITHOUT looking at q
  3. every returned sol must FK back to `pose` to 1e-9   (soundness)
  4. at least one returned sol must reproduce `pose`      (completeness)
  5. the original q must be recovered by some branch      (no missed solution)

It also reports the solution-branch histogram (a generic interior pose has 8) and confirms the
reachability shell is honest: targets outside it return no solutions, never a wrong one.

Deterministic: fixed RNG seed, so the gate is reproducible in CI.
"""

from __future__ import annotations

import sys
from collections import Counter

import numpy as np

sys.path.insert(0, ".")
from kinematics.arm import Arm            # noqa: E402
from kinematics.se3 import wrap           # noqa: E402

N = 20000
POSE_TOL = 1e-9        # FK(IK(pose)) must match pose to this (machine-ish precision)
Q_TOL = 1e-7          # a branch must reproduce the input joints to this


def main() -> int:
    arm = Arm()
    rng = np.random.default_rng(20260714)

    # Sample joints away from the exact singular set (measure zero anyway) for clean q-recovery.
    lo = np.array([-np.pi, -2.6, -2.7, -np.pi, 0.15, -np.pi])
    hi = np.array([np.pi, 2.6, 2.7, np.pi, 2.99, np.pi])
    Q = rng.uniform(lo, hi, size=(N, 6))

    worst_pose = 0.0
    worst_q = 0.0
    unsound = 0        # a returned solution that does NOT reproduce the pose
    incomplete = 0     # no solution returned for a reachable, FK-generated pose
    q_missed = 0       # input config not recovered by any branch
    counts = Counter()

    for q in Q:
        pose = arm.fk(q)
        sols = arm.ik(pose, tol=POSE_TOL)
        counts[len(sols)] += 1

        if not sols:
            incomplete += 1
            continue

        for s in sols:                                  # soundness: all returned are exact
            e = float(np.max(np.abs(arm.fk(s) - pose)))
            worst_pose = max(worst_pose, e)
            if e > POSE_TOL:
                unsound += 1

        # no-missed-solution: the input q is one of the branches
        dq = min(float(np.max(np.abs(wrap(s - q)))) for s in sols)
        worst_q = max(worst_q, dq)
        if dq > Q_TOL:
            q_missed += 1

    # Reachability honesty: random targets outside the shell must yield zero solutions.
    bad_reach = 0
    checked_out = 0
    for _ in range(4000):
        p = rng.uniform(-1.2, 1.2, size=3)
        R = arm.fk(rng.uniform(lo, hi))[:3, :3]         # some valid orientation
        wc = p - arm.Lt * R[:, 0]
        d = float(np.linalg.norm(wc))
        pose = np.eye(4)
        pose[:3, :3] = R
        pose[:3, 3] = p
        sols = arm.ik(pose, tol=POSE_TOL)
        if d > arm.reach_max + 1e-3 or d < arm.reach_min - 1e-3:
            checked_out += 1
            if sols:
                bad_reach += 1

    print(f"round-trip over {N} random configs:")
    print(f"  worst |FK(IK(pose)) - pose|  = {worst_pose:.2e}   (tol {POSE_TOL:.0e})")
    print(f"  worst input-joint recovery   = {worst_q:.2e}   (tol {Q_TOL:.0e})")
    print(f"  unsound solutions            = {unsound}")
    print(f"  incomplete (no solution)     = {incomplete}")
    print(f"  input config missed          = {q_missed}")
    print(f"  solution-count histogram     = {dict(sorted(counts.items()))}")
    print(f"reachability honesty: {checked_out} out-of-shell targets, "
          f"{bad_reach} wrongly 'solved'")

    ok = (worst_pose <= POSE_TOL and unsound == 0 and incomplete == 0
          and q_missed == 0 and bad_reach == 0)
    print("\nIK-CHECK:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
