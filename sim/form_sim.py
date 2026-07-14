#!/usr/bin/env python3
"""form_sim.py -- interactive MuJoCo bench for the universal-form manipulator.

The whole point of the repo, made pokeable: you move a TARGET in space and the whole form solves
its own closed-form IK to reach it -- the holonomic base drives, the lift raises, the arm settles.
Drag the target above the vertical ceiling and it turns RED: the base can chase any horizontal
target for free, but vertical reach is bounded by the body (README: "vertical must live in the
body"). That anisotropy is the thing you can now feel with your hand on the keyboard.

Source of truth: the MJCF is DERIVED from the `kinematics` package -- every link length, the lift
travel, the shoulder mount -- never re-typed here. The MuJoCo tree is built joint-for-joint to
match `kinematics.arm` FK, so MuJoCo's TCP equals `form.fk(...)` and the imported IK just works.

Drive: this is a *reachability* sim (contacts OFF -- the bracing/contact sim is the next roadmap
item), so the joints are driven KINEMATICALLY to the closed-form IK solution, rate-limited so the
base drives and the arm settles at believable speeds, then `mj_forward`. Exact at rest, always
stable -- no gain tuning, no gravity fight. (Dynamics + contacts arrive with the whole-body sim.)

Three modes, one file (per the house pattern):
    python sim/form_sim.py              # interactive viewer (glfw): WASD/RF move the target
    python sim/form_sim.py --demo       # headless scripted sweep -> build/form_sim.gif
    python sim/form_sim.py --selftest   # build + step + assert, no window (CI-safe)
"""

from __future__ import annotations

import os
import sys

_MODE = "demo" if "--demo" in sys.argv else "selftest" if "--selftest" in sys.argv else "interactive"
# GL backend is read at import time and depends on the mode: a window needs glfw; headless wants osmesa.
os.environ.setdefault("MUJOCO_GL", "glfw" if _MODE == "interactive" else "osmesa")

import numpy as np                                    # noqa: E402
import mujoco                                         # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from kinematics.arm import Arm                        # noqa: E402
from kinematics.form import UniversalForm             # noqa: E402
from kinematics.se3 import wrap                        # noqa: E402

# Canonical joint order for the qpos vector we command (matches make_xml's joint definition order).
JOINTS = ["base_x", "base_y", "base_yaw", "lift", "q1", "q2", "q3", "q4", "q5", "q6"]
ARM_JOINTS = ["q1", "q2", "q3", "q4", "q5", "q6"]
# Per-DOF slew rate (m/s for slides, rad/s for hinges): base drives fast, arm settles smartly.
RATE = {"base_x": 2.5, "base_y": 2.5, "base_yaw": 4.0, "lift": 1.0,
        "q1": 5.0, "q2": 5.0, "q3": 5.0, "q4": 6.0, "q5": 6.0, "q6": 6.0}
DT = 2e-3


def make_xml(form: UniversalForm) -> str:
    """MJCF built entirely from the kinematics constants -- geometry never drifts from the chain."""
    a: Arm = form.arm
    sx, sy, _ = form.shoulder_offset
    ceil = form.vertical_ceiling()
    return f"""
<mujoco model="universal-form">
  <option timestep="{DT}" integrator="implicitfast" gravity="0 0 0"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.7 0.7 0.7" specular="0.1 0.1 0.1"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.16 0.17 0.20" rgb2="0.22 0.24 0.28"
             width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="12 12" reflectance="0.05"/>
    <material name="link" rgba="0.60 0.63 0.70 1"/>
    <material name="link2" rgba="0.72 0.75 0.82 1"/>
    <material name="deck" rgba="0.30 0.34 0.42 1"/>
  </asset>

  <worldbody>
    <light pos="2 -2 4" dir="-0.4 0.4 -1" diffuse="0.9 0.9 0.9" specular="0.2 0.2 0.2"/>
    <light pos="-2 2 4" dir="0.4 -0.4 -1" diffuse="0.5 0.5 0.5"/>
    <geom name="floor" type="plane" size="6 6 0.1" material="grid" contype="0" conaffinity="0"/>

    <!-- the vertical ceiling, drawn as a faint disc: targets above it are unreachable -->
    <geom name="ceiling" type="cylinder" pos="0 0 {ceil}" size="6 0.002" contype="0" conaffinity="0"
          rgba="0.90 0.35 0.35 0.06"/>

    <!-- key-driven / draggable target (mocap: kinematic, set by the user, never integrated) -->
    <body name="target" mocap="true" pos="0.9 0.0 0.6">
      <geom name="target" type="sphere" size="0.045" contype="0" conaffinity="0"
            rgba="0.30 0.85 0.45 0.9"/>
    </body>

    <!-- holonomic base: 3 planar DOF (x, y, yaw). All robot geoms are visual-only (contacts off). -->
    <body name="base" pos="0 0 0">
      <joint name="base_x" type="slide" axis="1 0 0"/>
      <joint name="base_y" type="slide" axis="0 1 0"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1"/>
      <geom type="cylinder" pos="0 0 0.06" size="0.24 0.06" material="deck"
            contype="0" conaffinity="0"/>
      <geom type="cylinder" pos="0 0 {form.base_height/2:.4f}" size="0.06 {form.base_height/2:.4f}"
            material="deck" contype="0" conaffinity="0"/>

      <!-- torso lift: 1 prismatic DOF (z). The 'squat' DOF that owns vertical reach. -->
      <body name="torso" pos="{sx:.4f} {sy:.4f} {form.base_height:.4f}">
        <joint name="lift" type="slide" axis="0 0 1" range="0 {form.lift_max}" limited="true"/>
        <geom type="box" pos="0 0 -0.02" size="0.05 0.05 0.05" material="deck"
              contype="0" conaffinity="0"/>

        <!-- arm: shoulder-yaw + shoulder-pitch (the 'ball'), elbow, then the spherical wrist.
             Built joint-for-joint to match kinematics.arm FK. -->
        <body name="upperarm" pos="0 0 0">
          <joint name="q1" type="hinge" axis="0 0 1"/>
          <joint name="q2" type="hinge" axis="0 1 0"/>
          <geom type="capsule" fromto="0 0 0 {a.L1} 0 0" size="0.032" material="link"
                contype="0" conaffinity="0"/>
          <body name="forearm" pos="{a.L1} 0 0">
            <joint name="q3" type="hinge" axis="0 1 0"/>
            <geom type="capsule" fromto="0 0 0 {a.L2} 0 0" size="0.026" material="link2"
                  contype="0" conaffinity="0"/>
            <body name="wrist" pos="{a.L2} 0 0">
              <joint name="q4" type="hinge" axis="1 0 0"/>
              <joint name="q5" type="hinge" axis="0 1 0"/>
              <joint name="q6" type="hinge" axis="1 0 0"/>
              <geom type="sphere" size="0.03" material="link" contype="0" conaffinity="0"/>
              <geom type="capsule" fromto="0 0 0 {a.Lt} 0 0" size="0.018" material="link2"
                    contype="0" conaffinity="0"/>
              <site name="tcp" pos="{a.Lt} 0 0" size="0.02" rgba="0.35 0.85 0.5 1"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class Rig:
    """Model + data + the index bookkeeping to drive the chain from IK solutions (kinematically)."""

    def __init__(self, form: UniversalForm | None = None):
        self.form = form or UniversalForm()
        self.model = mujoco.MjModel.from_xml_string(make_xml(self.form))
        self.data = mujoco.MjData(self.model)
        self.ceiling = self.form.vertical_ceiling()
        _id = lambda kind, n: mujoco.mj_name2id(self.model, kind, n)
        jid = {j: _id(mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS}
        self.qadr = {j: int(self.model.jnt_qposadr[jid[j]]) for j in JOINTS}
        self.is_hinge = {j: self.model.jnt_type[jid[j]] == mujoco.mjtJoint.mjJNT_HINGE for j in JOINTS}
        self.arm_qadr = [self.qadr[j] for j in ARM_JOINTS]
        self.tcp = _id(mujoco.mjtObj.mjOBJ_SITE, "tcp")
        self.target_gid = _id(mujoco.mjtObj.mjOBJ_GEOM, "target")
        self.setpoint = {j: 0.0 for j in JOINTS}
        self.reachable = True
        mujoco.mj_forward(self.model, self.data)

    def track_target(self):
        """Solve whole-body IK for the current target, then slew the joints toward it and refresh.

        Kinematic drive: rate-limited motion to the exact closed-form IK setpoint. If the target is
        unreachable (above the ceiling), hold the last setpoint and flag it (caller recolors it red).
        """
        target = self.data.mocap_pos[0].copy()
        q_ref = self.data.qpos[self.arm_qadr]
        placed = self.form.place_for(target, approach=(0, 0, -1), q_ref=q_ref)
        self.reachable = placed is not None
        if placed is not None:
            (bx, by, yaw), lift, q = placed
            self.setpoint = dict(base_x=bx, base_y=by, base_yaw=yaw, lift=lift,
                                 q1=q[0], q2=q[1], q3=q[2], q4=q[3], q5=q[4], q6=q[5])

        for j in JOINTS:                                  # rate-limited slew toward the setpoint
            adr = self.qadr[j]
            diff = self.setpoint[j] - self.data.qpos[adr]
            if self.is_hinge[j]:
                diff = float(wrap(diff))
            lim = RATE[j] * DT
            self.data.qpos[adr] += float(np.clip(diff, -lim, lim))

        self.model.geom_rgba[self.target_gid] = ((0.90, 0.30, 0.30, 0.9) if not self.reachable
                                                 else (0.30, 0.85, 0.45, 0.9))
        mujoco.mj_forward(self.model, self.data)

    def tcp_pos(self) -> np.ndarray:
        return self.data.site_xpos[self.tcp].copy()

    def set_target(self, xyz):
        self.data.mocap_pos[0] = np.asarray(xyz, float)


# --------------------------------------------------------------------------- modes
def run_interactive():
    import time
    import mujoco.viewer

    rig = Rig()
    rig.set_target((0.9, 0.0, 0.6))
    step = 0.04

    def on_key(keycode):
        try:
            ch = chr(keycode).upper()
        except ValueError:
            return
        d = {"A": (-step, 0, 0), "D": (step, 0, 0), "W": (0, step, 0), "S": (0, -step, 0),
             "R": (0, 0, step), "F": (0, 0, -step)}.get(ch)
        if d is not None:
            rig.data.mocap_pos[0] += np.array(d)

    print(__doc__)
    print("controls:  A/D = -x/+x   W/S = +y/-y   R/F = up/down   (or ctrl-drag the sphere)")
    print(f"vertical ceiling = {rig.ceiling:.2f} m  -> the target turns RED above it\n")
    with mujoco.viewer.launch_passive(rig.model, rig.data, key_callback=on_key) as viewer:
        while viewer.is_running():
            t0 = time.time()
            rig.track_target()
            viewer.sync()
            dt = rig.model.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


def _demo_target(t: float, ceiling: float) -> np.ndarray:
    """Deterministic, continuous scripted target: reach out, rise through the ceiling, come back
    into reach, then arc around -- so the base drives, the lift works, and the ceiling shows."""
    top = ceiling + 0.22
    if t < 3.0:                                  # 1) reach out horizontally -> base drives across
        return np.array([0.6 + (2.4 - 0.6) * (t / 3.0), 0.0, 0.55])
    if t < 5.5:                                  # 2) rise straight up, past the ceiling -> red
        return np.array([2.4, 0.0, 0.55 + (top - 0.55) * ((t - 3.0) / 2.5)])
    if t < 7.0:                                  # 3) come back down into reach -> green again
        return np.array([2.4, 0.0, top + (0.9 - top) * ((t - 5.5) / 1.5)])
    s = (t - 7.0) / 3.0                          # 4) sweep an arc -> base repositions around
    ang = s * np.pi
    return np.array([2.4 * np.cos(ang), 2.4 * np.sin(ang), 0.9])


def run_demo():
    import imageio.v2 as imageio

    rig = Rig()
    dur, fps = 10.0, 30
    every = int(round(1.0 / fps / DT))
    renderer = mujoco.Renderer(rig.model, 540, 960)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (1.0, 0.0, 0.7)
    cam.distance, cam.azimuth, cam.elevation = 5.8, 135, -18
    frames, lag, stills = [], [], {}

    for k in range(int(dur / DT)):
        t = k * DT
        rig.set_target(_demo_target(t, rig.ceiling))
        rig.track_target()
        if rig.reachable:
            lag.append(float(np.linalg.norm(rig.tcp_pos() - rig.data.mocap_pos[0])))
        if k % every == 0:
            renderer.update_scene(rig.data, cam)
            frames.append(renderer.render())
        for name, tt in (("reach", 2.9), ("ceiling", 5.4), ("arc", 8.6)):  # key stills
            if name not in stills and t >= tt:
                renderer.update_scene(rig.data, cam)
                stills[name] = renderer.render()

    os.makedirs("build", exist_ok=True)
    imageio.mimsave("build/form_sim.gif", frames, fps=fps, loop=0)
    for name, img in stills.items():
        imageio.imwrite(f"build/form_sim_{name}.png", img)
    print(f"demo: {len(frames)} frames -> build/form_sim.gif  (+ key stills form_sim_*.png)")
    print(f"model: {rig.model.nbody - 1} bodies, {rig.model.nq} DOF, ceiling {rig.ceiling:.2f} m")
    print(f"IK is exact at rest (see --selftest); the {np.mean(lag)*1e3:.0f} mm mean / "
          f"{np.max(lag)*1e3:.0f} mm peak here is base/arm SLEW LAG chasing a moving target.")
    _plot_error(lag, rig.ceiling)


def _plot_error(err, ceiling):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    fig, ax = plt.subplots(figsize=(7, 2.6))
    ax.plot(np.arange(len(err)) * DT, np.array(err) * 1e3, lw=1.4, color="#4c9be8")
    ax.set_xlabel("time (s)"); ax.set_ylabel("TCP slew lag (mm)")
    ax.set_title(f"whole-body tracking of a moving target (lag, not IK error); ceiling {ceiling:.2f} m")
    ax.grid(alpha=0.3); fig.tight_layout()
    fig.savefig("build/form_sim_error.png", dpi=110)
    print("       tracking plot -> build/form_sim_error.png")


def selftest() -> int:
    rig = Rig()
    print(f"model: {rig.model.nbody - 1} bodies, {rig.model.nq} DOF, ceiling {rig.ceiling:.2f} m")

    # Track a fixed, reachable target; assert it converges exactly and nothing goes non-finite.
    goal = np.array([1.3, 0.4, 0.5])
    rig.set_target(goal)
    for _ in range(2000):
        rig.track_target()
        if not np.all(np.isfinite(rig.data.qpos)):
            print("SELFTEST: FAIL (non-finite state)")
            return 1
    err = float(np.linalg.norm(rig.tcp_pos() - goal))

    # Honest ceiling: a target well above it must be flagged unreachable.
    rig.set_target(np.array([1.3, 0.4, rig.ceiling + 0.3]))
    rig.track_target()
    ceiling_ok = not rig.reachable

    # Cross-check: MuJoCo's TCP equals the kinematics FK (the model really matches the chain).
    rig.set_target(goal)
    for _ in range(2000):
        rig.track_target()
    q_arm = rig.data.qpos[rig.arm_qadr]
    base = (rig.data.qpos[rig.qadr["base_x"]], rig.data.qpos[rig.qadr["base_y"]],
            rig.data.qpos[rig.qadr["base_yaw"]])
    lift = rig.data.qpos[rig.qadr["lift"]]
    fk_tcp = rig.form.fk(base, lift, q_arm)[:3, 3]
    fk_gap = float(np.linalg.norm(fk_tcp - rig.tcp_pos()))

    print(f"reachable target: TCP error = {err*1e3:.4f} mm  (want < 1 mm)")
    print(f"above-ceiling target flagged unreachable: {ceiling_ok}")
    print(f"MuJoCo TCP vs kinematics FK: {fk_gap*1e3:.2e} mm  (model matches the chain)")
    ok = err < 1e-3 and ceiling_ok and fk_gap < 1e-6
    print("SELFTEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    if _MODE == "selftest":
        raise SystemExit(selftest())
    elif _MODE == "demo":
        run_demo()
    else:
        run_interactive()
