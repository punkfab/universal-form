# universal-form — kickoff brief

- **Problem:** `punkfab` has designs for *how robots get around* (`robot-locomotion`) and for
  *making parts* (`software-mfg`), but no home for the question that sits above both: **what shape
  should the whole manipulating body be?** How many degrees of freedom, where do they live, and why
  — argued from first principles rather than copied from a humanoid because humanoids are familiar.
- **Done looks like:** A defensible **minimal kinematic budget** for general indoor manipulation
  (base + lift + arm + spherical wrist + underactuated hand, ≈ 11–13 DOF) that isn't just asserted —
  each load-bearing claim (full-pose reach, the value of the 7th DOF, closed-form IK from a
  spherical wrist, brace-instead-of-second-arm) becomes a **gate you can run** in sim. Sibling
  projects consume the budget by reference instead of re-deriving it.
- **Not now:** No hardware build. No second dexterous arm until a task distribution demands it (the
  default is one dexterous arm + bracing affordances). No humanoid-for-its-own-sake — legs, faces,
  and aesthetics are out; this is about the *kinematics of manipulation*, not the silhouette.
- **First slice:** This repo ships **reasoning-first** — the design brief (this file) + the thesis
  (`README.md`) that distills the argument: separate function from substrate artifact (the elbow
  tell), concentrate DOF proximally, locomotion-as-extra-DOF, base-delivers/arm-settles, contact
  regulation over pose reaching, the spherical-wrist divergence, and bracing over bimanuality. The
  first *code* slice is the kinematic model + closed-form IK gate.
- **Open question:** The **redundancy knee** — once the base is holonomic, does the arm want exactly
  6 DOF or 7, and is everything beyond that better resolved in the base's null space than added to
  the arm? And is bracing *actually* cheaper than a second arm once you pay for the contact-modeling
  policy it needs? Both are empirical; they're why the reachability + whole-body sims exist.

## Relationship to siblings

- **`robot-locomotion`** owns the base (holonomic omni tripod). `universal-form` treats that base as
  the proximal-most links of the manipulator's chain — reach-by-repositioning — and imports its
  kinematics by reference rather than forking them.
- **`software-mfg`** manufactures parts. When `universal-form` authors its own geometry (lift column,
  wrist, bracing surfaces), those parts follow the same hub-and-spoke compose-by-reference pattern.
- Same house rules as the siblings: **code-first, parametric CAD, every claim a gate you can run.**
  Here the claims start as arguments and become gates as the sim lands.

## Status (2026-07-14)

- ✅ **Design thesis** written and public: the minimal DOF budget, the function-vs-substrate test,
  and the bracing-over-bimanuality stance are argued end-to-end in `README.md`.
- ✅ **Kinematic model + closed-form IK** (spherical-wrist decoupling) — `kinematics/` + `make
  ik-check`, proving `FK∘IK = identity` to ~5e-14 over 20k random configs (all 8 branches, none
  missed, reachability honest).
- ✅ **Workspace + base/lift redundancy** — `make reach-check`: the mobile form solves 100% of
  in-envelope room targets vs ~1% for a bolted arm, and reports its honest 1.43 m vertical ceiling.
- ⬜ **The redundancy knee** — promote the arm to 7-DOF and measure what the extra joint buys.
- ⬜ **Bracing affordance study** + **MuJoCo whole-body** — test brace-instead-of-second-arm.
