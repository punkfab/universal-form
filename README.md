# universal-form

**What shape should a robot be** — a code-first search for the *simplest kinematic system that
solves general manipulation in the real world*, argued from first principles and (soon) tested in
sim.

This is the `punkfab` home for **morphology**. Where [`robot-locomotion`](../robot-locomotion) owns
*how robots get around*, this repo owns *what the whole manipulating body should look like* — the
degrees of freedom, where they live, and why. It is the **source of truth** for these design
decisions; sibling projects consume the resulting kinematic budgets by reference rather than
re-deriving them. See [BRIEF.md](./BRIEF.md) for the why.

> **Status: design brief.** Today this repo is the *argument*. The claims below are stated as
> arguments, not yet as gates you can run — the reachability / closed-form-IK / bracing sims that
> turn them into `make check` gates are the first code slice (see [Roadmap](#roadmap)). Reasoning
> comes first here on purpose.

## The thesis

Take the human body as a worked example of a general manipulator, then ask of every feature:
**is this a function, or a substrate artifact?** Evolution optimized for grow-from-one-cell,
self-repair, and lifetime energy. We optimize for **manufacturability, IK-solvability, and
serviceability**. So we don't copy the body — we copy only the principles that *survive changing
the objective*, and we delete the limitations that were only ever accidents of being made of meat.

The whole skill is telling those two apart. The elbow is the tell:

> **Why don't humans have elbows that bend backward?** Two reasons, and only one of them
> generalizes. (1) The shoulder is a 3-DOF ball joint, so it already reorients the arm into any
> half-space — a backward-bending elbow would be a redundant DOF stacked on one you already have.
> (2) A *one-way* hinge earns a hard bony endstop (the olecranon seating in its fossa), which lets
> you bear load with **zero actuator torque** — lock out a pushup, hang from a bar. Reason (1) is
> "don't duplicate reach the base already provides." Reason (2) is "a unidirectional hinge gets a
> free structural lock." **Both are functions. Keep them both.**

Contrast the limits that are *substrate artifacts* — inherit the function, delete the limit:

| human limit | why it exists | robot answer |
|---|---|---|
| forearm roll capped at ~180° | two bones (radius/ulna) physically cross | **continuous** roll on a slip ring |
| neck won't turn 360° | arteries/nerves would strangle | **continuous** head yaw on a slip ring |
| shoulder dislocates | soft tissue traded stability for range | ball joint **both** maximally mobile *and* strong |
| 27-DOF hand you can't independently drive | grown, not designed | a few **coupled synergies** baked into tendons |
| two identical (symmetric) arms | mirror-symmetric development | **asymmetric** arms matching the doer/holder split |

## Principles that transfer

- **6 DOF is the floor, 7 is the sweet spot.** Placing a hand at an arbitrary position *and*
  orientation is SE(3) — exactly 6. The human arm has 7 (3 shoulder + elbow + forearm roll + 2
  wrist). The extra one is *redundancy*: move the elbow on a circle while the hand stays put — reach
  around obstacles, dodge self-collision, escape singularities.
- **Concentrate DOF proximally; keep distal joints simple, strong, and light.** Big articulation at
  the base, dumb hinges at the tip. The base's *range* substitutes for the tip's *bidirectionality*,
  and keeping heavy actuators proximal minimizes distal inertia. Fingers are the extreme: the motors
  live in the forearm and pull tendons.
- **Locomotion is the ultimate extra DOF.** Don't build an arm that reaches everywhere; build a
  *base that repositions*. This is why `universal-form` and `robot-locomotion` are siblings — the
  manipulator's kinematic chain **includes the base**.
- **Reach is not isotropic.** Horizontal reach is cheap to borrow from a mobile base; **vertical
  reach has no base solution** and must live in the body (a lift column + arm). Biology pays for this
  by squatting — expensive and unstable. A lift column is the honest robot answer.
- **The base delivers, the arm settles.** Base and arm aren't interchangeable reach — they differ on
  *settling time and precision*. The base gets the hand to within one arm-workspace and holds still;
  the arm does everything precise inside that bubble. So the arm only needs to span the largest
  *single* manipulation that happens without repositioning — much smaller than "human reach."
- **Manipulation is contact regulation, not pose reaching.** Almost nothing hard is a reaching
  problem; it's constrained, contact-rich motion where you must regulate *force* along the directions
  the world removes from you. This adds an axis orthogonal to any DOF count: **impedance** —
  controllable stiffness, soft along constrained directions, stiff along free ones.
- **Add the spherical wrist biology skipped.** If the last three axes intersect at a point
  (Pieper's criterion), IK *decouples*: the arm solves position, the wrist solves orientation,
  closed-form. Humans can't (offset wrist axes) and don't need to — they solve IK with a brain.
  A robot computes IK, so it should pay the packaging cost to make IK analytic. The biggest
  worth-making divergence from the body.
- **Complexity is conserved — choose where to pay it.** A spherical wrist is mechanically harder
  but makes IK trivial; a holonomic base is mechanically fussier but makes planning trivial;
  underactuation drops motors but moves intelligence into passive mechanics. Bias toward
  *mechanically-simple / software-expensive*: mechanism is atoms built many times, software is
  written once.

## The minimal universal form

The smallest DOF budget that plausibly solves general **indoor** manipulation:

| stage | DOF | why it's there |
|---|---|---|
| holonomic base (omni) | 3 (x, y, yaw) | reach-by-repositioning — owned by `robot-locomotion` |
| torso lift (prismatic) | 1 | floor-to-shelf vertical span, the "squat" DOF at no arm cost |
| arm | 6 (or 7) | 6 = full pose; the 7th buys obstacle/singularity redundancy |
| spherical wrist | *(last 3 of the arm)* | intersecting axes → closed-form IK |
| hand | 1–3 | underactuated, opposition-based, self-adapting |
| head pan-tilt | 2 | sensing decoupled from the arms |

**≈ 11–13 controllable DOF** for a general single-arm manipulator — remarkably few. The hand is
where "general" actually lives: the one primitive that matters is **opposition** (a thumb pressing
an object against fingers or palm). Two-point antipodal pinch (parallel jaw) solves an astonishing
fraction of pick-and-place; opposition + a palm adds enveloping power grasps; three tendon-driven
self-adapting fingers on ~2–3 actuators sit at the knee of the curve — you were never independently
driving 27 finger DOF anyway. Bake the synergies into the tendons, not the controller.

## The second hand: bracing, not bimanuality

"Bimanual vs. not" is the wrong axis. A fixture is a *specialized* second hand; a second hand is a
*universal* fixture — so the real variable is **who designs the environment**. In a designed space
you pre-install the constraint (one arm + jig wins). In an *undesigned* space you can't fixture the
long tail, and the only general on-demand holder for an unmodeled object is a hand.

But the "second constraint" is a spectrum, and a full second dexterous arm is its over-provisioned
end: `full arm → cheap clamp-positioner → body-as-anvil → environment bracing → fixture`. A
one-armed person opening a jar (armpit) or folding a towel is the existence proof that most
"bimanual" tasks are **one active hand + one passive brace**, and the brace need not be a hand. The
irreducible core of *true* bimanuality is small and nameable: **two continuously-coupled fine
motions on a deformable or multi-body object** — cloth, cable, food, in-air assembly — where the
holding role is itself dynamic. Everything rigid collapses to one dexterous hand + a brace.

So the design stance:

- **One** genuinely dexterous arm (the full stack above).
- A body **designed as bracing affordances** — torso, forearms, lift column, and base mass as
  deliberate anvils the policy presses objects into — not just a mounting frame.
- A **contact-recruiting policy** that treats environment and body surfaces as first-class
  constraint sources.
- A second effector only as a *cheap asymmetric holder* by default; a *second dexterous arm* strictly
  if deformable-object or in-air-assembly work is in scope. (Note: a real second arm costs far more
  than 2× hardware — closed-chain calibration, internal-force regulation, self-collision, and dual-arm
  coordination are where the complexity actually hides.)

The deeper reframe: humans improvise with *many* constraint sources — pin paper under an elbow, hold
a pen in the teeth, trap a box between the knees. The quantity that matters is **the number and
quality of controllable + opportunistic constraint sources; dexterity is needed only at the one or
two that must move finely.** You can't afford N dexterous arms; you can afford N bracing surfaces
that cost nothing extra.

## Roadmap

Turning the argument into gates you can run:

- ⬜ **Kinematic model** — the base + lift + arm + spherical-wrist chain as parametric code, with a
  **closed-form IK** solver and a gate proving it hits target poses (`make ik-check`).
- ⬜ **Reachability / workspace sim** — sample manipulation targets, measure workspace coverage and
  the value of the 7th DOF; empirically find the **redundancy knee** (6 vs 7 vs base-null-space).
- ⬜ **Bracing affordance study** — what a torso/forearm/base surface must be (geometry, compliance,
  friction, sensing) to be a trustworthy anvil, and whether a contact-recruiting policy is harder
  than dual-arm coordination or just differently hard.
- ⬜ **MuJoCo whole-body** — the form as a contact-rich body; test the base-delivers/arm-settles and
  brace-instead-of-second-arm claims against a real task distribution.

## Open questions

- **The redundancy knee.** Once the base is holonomic, does the arm want exactly 6, or 7 — and is
  everything beyond that better resolved in the base's null space than by piling DOF onto the arm?
- **Is bracing actually cheap?** It pays in software (modeling uncertain world contacts). Does that
  bill come in under the cost of a second arm, or does contact uncertainty eat the savings?
- **How many constraint sources does the task distribution really want** — and can a body of cheap
  bracing affordances + a good policy cover the tail that a second arm would?

## Layout *(planned — arrives with the first code slice)*

```
kinematics/   # the DOF chain as parametric code + closed-form IK (spherical-wrist decoupling)
sim/          # reachability / workspace coverage + MuJoCo whole-body contact-rich tests
parts/        # build123d CAD for the form's own geometry (lift column, bracing surfaces, wrist)
scripts/      # one gate per claim (ik-check, reach-check, brace-check)
```

## License

MIT © 2026 dnewcome
