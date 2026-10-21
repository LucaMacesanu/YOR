# VR Teleop + IK Reference

Reference doc for `robot/teleop/yor_teleop.py` (the VR/advanced teleop client) and the IK math it triggers
on the robot side, so that joint-space `q_targets` can be reconstructed offline from recorded cartesian
end-effector action states without re-running the live teleop stack.

## 1. The `yor_teleop.py` script

`robot/teleop/yor_teleop.py` is the VR/advanced teleop client (`python robot/teleop/yor_teleop.py`, per
`CLAUDE.md`). It:

- Runs a background thread that subscribes over ZMQ (`SUB` socket, topic `oculus_controller`, host/port
  from `robot/config.py`'s `VR_IP`:5555) and parses incoming Oculus controller messages via
  `parse_controller_state()` (`robot/teleop/oculus_msgs.py`) into a `ControllerState` dataclass (buttons,
  triggers, thumbsticks, per-hand raw position/rotation).
- Converts each hand's raw VR pose into a per-arm SE3 **cartesian end-effector target** via a clutch
  mechanism (§5, §10).
- Sends those targets to the physical robot over `commlink.RPCClient("localhost", 5557)`, which proxies the
  call to `robot/yor.py`'s `YOR.set_left_ee_target` / `set_right_ee_target` / `set_bimanual_ee_target`.
- Separately drives the swerve-drive base and the lift from thumbsticks/hand-triggers (§8) — those paths do
  not go through IK at all.
- Optionally records an episode to CSV for data collection (poses, gripper values, lift commands).

## 2. IK dependency explicitly written

`mink` is the only IK library imported directly:

- `yor_teleop.py`: `import mink` — used for its `SE3`/`SO3` Lie-group math when building the cartesian
  target (§5, §10). It does **not** call `mink.solve_ik` itself — that happens downstream, robot-side.
- `robot/arm/ik_solver.py` (where the actual solve happens): builds on `mink.Configuration`,
  `mink.FrameTask`, `mink.PostureTask`, `mink.ConfigurationLimit`, `mink.solve_ik`, `mink.SE3`/`mink.SO3`.
- `mujoco` is used underneath (`mujoco.MjModel.from_xml_path`) to build the kinematic model that mink solves
  against — the MJCF in §9.
- `mink.VelocityLimit` and `mink.EqualityConstraintTask` are imported/referenced in `ik_solver.py` but are
  **not active** on the code path `yor_teleop.py` actually exercises (see §6, §8).

## 3. Dependency versions

| Package | `pyproject.toml` | `uv.lock` (resolved, sim-only venv) | `requirements.txt` (frozen physical-robot conda/ROS env) |
|---|---|---|---|
| `mink` | unpinned | `1.0.0` (plain PyPI, no fork/git pin) | **absent** — see note below |
| `mujoco` | `>=3.2.7` | `3.4.0` | `3.3.4` |
| `numpy` | unpinned | `2.2.6` (py<3.11) / `2.4.1` (py>=3.11) | `1.21.5` |
| `scipy` | unpinned, `nav` extra only | `1.15.3` / `1.17.0` (transitive only) | `1.8.0` |
| `pyzmq` | unpinned | `27.1.0` | `22.3.0` |
| `loop-rate-limiters` | `>=1.1.0` | `1.2.0` | `1.2.0` |
| `ruckig` | not declared | absent | `0.12.2` |
| `pinocchio` | not declared | absent | `pin==2.7.0` (pinocchio's actual PyPI/conda package name) |

Notes:

- `mink` resolves from plain PyPI in `uv.lock` (`source = { registry = "https://pypi.org/simple" }`) — it
  is **not** vendored or pulled from a git fork/commit, unlike some other repo dependencies
  (`requirements.txt` does contain git-pinned packages elsewhere, e.g. `home_robot`, `pytorch3d`,
  `segment-anything`, but `mink` isn't one of them).
- `ruckig` and `pinocchio` are installed outside the `uv`-managed venv, via the conda environment described
  in `CLAUDE.md` (`conda install -c conda-forge pinocchio ...`, `pip install ... ruckig`), which is why
  they don't appear in `uv.lock`.
- **Reproducibility gap**: `mink` does not appear anywhere in `requirements.txt`. On the physical robot it
  is installed only via `pip install -e .` resolving pyproject.toml's unpinned `mink` dependency from
  PyPI — the exact version actually running on hardware is whatever was latest at install time, not
  necessarily `1.0.0`. Worth pinning explicitly if exact IK reproduction matters.

## 4. IK seed

There is **no fixed numeric q0 array** in the mink IK path.

- `SingleArmIK.init(q)` (`robot/arm/ik_solver.py`) seeds `mink.Configuration` from whatever `q` is passed
  in:
  ```python
  def init(self, q):
      current_q = self.configuration.q.copy()
      current_q[self.dof_ids] = q
      self.configuration.update(current_q)
      self.posture_task.set_target_from_configuration(self.configuration)
      self.initalized_ = True
  ```
- `ArmNode.init()` (`robot/arm/arm.py`) calls this **once at startup**, passing the arm's *live joint
  feedback* from nerolib/hardware — not a canonical home pose.
- The `PostureTask` target is fixed to that same configuration via `set_target_from_configuration()` at
  that moment — it is set once at init and does **not** track a moving target afterward.
- `SingleArmIK.get_home_q()` reads a MuJoCo `<keyframe><key name="home" .../>` from the MJCF, but
  `nero-welded-base-and-lift.mjcf` (§9) has **no `<keyframe>` block at all** — this method would raise if
  called against it, and indeed it is never called on the hardware path (only exercised against different,
  older MJCFs that do define a `home` key, e.g. `arm-upright.mjcf`).
- A separate, unrelated hardcoded array exists purely for **nerolib's own hardware homing**
  (`ArmNode.__init__`'s `home_position`):
  ```python
  self.home_position = (
      [1.8050, 1.2755, -1.8566, 1.3845, -0.1778, -0.0746, 0.0524]
      if is_left_arm
      else [-1.8050, 1.2755, 1.8566, 1.3845, -0.1778, -0.0746, 0.0524]
  )
  ```
  This is **not** consumed by mink/IK at all — don't conflate it with the IK seed above.

**Practical implication for offline reconstruction** (feeds into §10): since live data collection seeded
from whatever the arm's actual joint state was at boot/init time, and IK then ran sequentially with mink's
configuration persisting frame-to-frame, the correct offline seed is the *first recorded joint state* in
the episode, followed by *warm-starting each subsequent IK solve from the previous solve's output* — not
re-seeding from scratch every step.

## 5. Frame and tool offsets

- The mink `FrameTask` target frame is the MJCF `<site>` named `left_arm_ee` / `right_arm_ee` directly
  (`frame_type="site"`). There is **no additional runtime tool/wrist offset** applied between the
  `ee_target` SE3 passed to `set_ee_target` and the FrameTask target — any tool-tip geometry is baked into
  where that site is placed in the MJCF, not adjustable at runtime.
- **Upstream** of IK, `yor_teleop.py` applies a fixed rotation-only frame remap:
  ```python
  self.H = mink.SE3.from_rotation(mink.SO3.from_matrix(np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])))
  ```
  plus a clutch-based incremental delta (full formula in §10) to turn raw VR controller poses into the
  `ee_target` SE3 sent to `set_bimanual_ee_target`.
- Additionally, `oculus_msgs.py`'s `left_SE3`/`right_SE3` properties apply a left-handed→right-handed
  coordinate fix on the raw VR pose before any of the above:
  ```python
  M = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])
  rotation_mat = M @ from_quat(self.left_local_rotation) @ M.T
  translation = self.left_local_position * np.array([1, 1, -1])
  ```

**Important caveat**: if the collected "action states" already *are* those `ee_target` SE3 values (i.e.,
captured at or after the `set_bimanual_ee_target` call), then the `M` handedness fix, `H` remap, and clutch
math are **already baked in** and must **not** be reapplied. They only need to be reapplied if
reconstructing from raw VR controller telemetry instead of from the already-computed cartesian targets.

## 6. Solver config

`SingleArmIK.solve_ik` (`robot/arm/ik_solver.py`) — the class actually used by `set_bimanual_ee_target`:

```python
def solve_ik(self, T_wt: mink.SE3, max_iter: int = 10, pos_eps: float = 1e-3, rot_eps: float = 1e-3):
    self.end_effector_task.set_target(T_wt)
    is_solved = False
    for _ in range(max_iter):
        vel = mink.solve_ik(
            self.configuration, self.tasks, self.solver_dt, solver="daqp", damping=1e-5, limits=self.limits
        )
        self.configuration.integrate_inplace(vel, self.solver_dt)
        err = self.end_effector_task.compute_error(self.configuration)
        if np.linalg.norm(err[:3]) <= pos_eps and np.linalg.norm(err[3:]) <= rot_eps:
            is_solved = True
            break
    return self.configuration.q[self.dof_ids], is_solved
```

- `solver_dt = 0.01` s (`ArmNode` default).
- Up to `max_iter = 10` iterations per call, early-exit once `norm(err[:3]) <= pos_eps = 1e-3` (m) and
  `norm(err[3:]) <= rot_eps = 1e-3` (rad).
- `FrameTask(position_cost=1.0, orientation_cost=0.1, lm_damping=1.0)`.
- `PostureTask(cost = [1e-1] (lift) + [1e-3]*7 (left arm) + [1e-3]*7 (right arm))` — a 15-dim cost over the
  **full** model configuration (lift + both arms), even though a given `SingleArmIK` instance only actively
  drives one arm's 7 `dof_ids`.
- `limits = [mink.ConfigurationLimit(model)]` only — `mink.VelocityLimit` is referenced in a comment but
  not enabled.
- Each arm (`left_arm`, `right_arm`) has its **own independent** `SingleArmIK` / `mink.Configuration`
  instance, both built from the same MJCF. Despite the RPC method being named `set_bimanual_ee_target`, the
  two arms are solved **completely independently** — there is no cross-arm coupling task.
  - Contrast with the separate `BimanualArmIK` class in the same file, which *does* do true coupled
    two-arm IK in one shared configuration, single `mink.solve_ik` call per tick, `damping=1e-3`, and
    `mink.VelocityLimit` enabled — but that class is only used by an unrelated dora/Piper subsystem
    (`bimanual_arm_node.py`, `bimanual_arm_mujoco_node.py`), **not** by `yor_teleop.py`.

## 7. Pose IK or differential IK

An **iterated damped-least-squares/QP differential IK run to approximate convergence per call**: each
`solve_ik()` invocation loops up to 10 single-step `mink.solve_ik` (velocity) + `integrate_inplace`
iterations with early-exit on a tight pose-error tolerance — functionally a full-pose IK solve
(Gauss-Newton-style), implemented on top of mink's differential-IK primitive, rather than either a single
open-loop differential tick or a closed-form pose IK.

Both position (weight `1.0`) and orientation (weight `0.1`) are targeted — full 6-DOF pose, not
position-only.

## 8. How the lift is handled

The lift is **entirely outside the cartesian/IK pipeline**:

- The MJCF's `lift_joint` has no live `<actuator>` — only a commented-out placeholder in
  `nero-welded-base-and-lift.mjcf` (§9) — so mink/MuJoCo cannot actuate it on this model regardless.
- Physically it's driven by a separate Raspberry Pi Pico microcontroller over USB serial
  (`robot/base_motor.py`'s `PicoLift`: plain-text `up`/`down`/`home`/`stop` commands sent to
  `/dev/ttyACM0`, height read back via regex on `"Height: <mm>"` telemetry), exposed through
  `robot/yor.py`'s RPC surface (`lift_up`/`lift_down`/`lift_stop`/`lift_home`/`get_lift_height`/
  `set_lift_position`/`lift_to_height`, all forwarding to `Base`/`PicoLift`).
- In `yor_teleop.py`, pulling **both hand triggers together** toggles a base/lift-control mode. While
  active:
  ```python
  if controller_state.left_hand_trigger > 0.5 and not controller_state.right_hand_trigger > 0.5:
      self.yor.lift_up()
      lift_target_cmd = 1
  elif controller_state.right_hand_trigger > 0.5 and not controller_state.left_hand_trigger > 0.5:
      self.yor.lift_down()
      lift_target_cmd = -1
  else:
      self.yor.lift_stop()
  ```
  This is a discrete bang-bang command stream, **not** a Cartesian/IK-derived target.
- On data-collection reset (`--reset_base`), recorded `lift_cmd` is replayed as a hardcoded absolute
  height: `set_lift_position(np.array([0.0]))` if the last command was up, or
  `set_lift_position(np.array([0.41]))` if down.
- The MuJoCo simulation entry point (`robot/yor_mujoco.py`) has **no lift support at all** — no
  `lift_*`/`set_bimanual_ee_target` methods exist on its RPC surface, only per-arm EE/joint targets.
- A separate, unrelated `robot/lift.py` defines a TalonFX/Phoenix6-CAN-based `Lift` class — nothing in the
  repo imports it; it appears to be legacy/unused code. The currently wired lift hardware is the serial
  `PicoLift` above.

**Practical implication**: if the collected data includes a lift dimension, treat it as its own independent
up/down/stop (or absolute-height) signal — never route it through the arm IK/`q_target` math.

## 9. MJCF description

`robot/yor-description/nero-welded-base-and-lift.mjcf` — the file `ArmNode`/`SingleArmIK` load to build the
IK model for both arms:

```xml
<mujoco model="cone-e-nero">
  <compiler angle="radian" meshdir="assets_nero"/>
  <option integrator="implicitfast" cone="elliptic" impratio="10" gravity="0 0 -9.81" timestep="0.005" />

  <default>
    <default class="robot">
      <default class="visual">
        <geom material="visualgeom" type="mesh" contype="0" conaffinity="0" group="2" density="0"/>
      </default>
      <default class="collision">
        <geom type="capsule" condim="3" contype="0" conaffinity="1" priority="1" group="3" solref="0.005 1"
          friction="1 0.01 0.01" />
      </default>
    </default>

    <default class="nero">
      <joint frictionloss="0.3" armature="0.005" />
      <position inheritrange="1" forcerange="-100 100" />
      <default class="collision_gripper">
        <geom type="mesh" group="3" friction="2 0.005 0.0001" density="100" />
      </default>
    </default>

    <default class="telescope">
      <joint type="slide" axis="0 0 1" damping="50" stiffness="0.02" armature="0.01" range="0 0.208" frictionloss="1.5"/>
      <general ctrlrange="0 0.416" gainprm="150" biasprm="0 -100 -10" forcerange="-56 56"/>
    </default>
  </default>

  <asset>
    <material name="gray_mat" rgba="0.59 0.59 0.59 1" />
    <material name="light_medium_gray_mat" rgba="0.85 0.85 0.85 1" />
    <material name="base_material" rgba="1 0 0 1" />
    <material name="Lift_Middle_material" rgba="0.9 0.9 0.9 1" />

    <mesh name="base_link" file="base_link.STL" />
    <mesh name="link1" file="Link1.STL" />
    <mesh name="link2" file="Link2.STL" />
    <mesh name="link3" file="Link3.STL" />
    <mesh name="link4" file="Link4.STL" />
    <mesh name="link5" file="Link5.STL" />
    <mesh name="link6" file="Link6.STL" />
    <mesh name="link7" file="Link7.STL" />
    <mesh name="stand_v2" file="stand_v2.STL" />

    <mesh name="base_profile_short" file="base_profile_short.stl" />
    <mesh name="Part_2" file="Part_2.stl" />
    <mesh name="MK4_Colson_Hub_4_X_1.5" file="MK4_Colson_Hub_4_X_1.5.stl" />
    <mesh name="Part_2_2" file="Part_2_2.stl" />
    <mesh name="MK4_Colson_Hub_4_X_1.5_2" file="MK4_Colson_Hub_4_X_1.5_2.stl" />
    <mesh name="Part_2_3" file="Part_2_3.stl" />
    <mesh name="MK4_Colson_Hub_4_X_1.5_3" file="MK4_Colson_Hub_4_X_1.5_3.stl" />
    <mesh name="Part_2_4" file="Part_2_4.stl" />
    <mesh name="MK4_Colson_Hub_4_X_1.5_4" file="MK4_Colson_Hub_4_X_1.5_4.stl" />
    <mesh name="Lift_Middle" file="Lift_Middle.stl" />
    <mesh name="Lift_Top" file="Lift_Top.stl" />
  </asset>

  <worldbody>
    <body name="base" pos="0 0 0.104775" euler="0 1.57 0" childclass="robot">
       <geom name="base_visual" type="sphere" size="0.01" class="visual" material="base_material"/>
       <body name="base_profile_short">
          <geom mesh="base_profile_short" class="visual" material="base_material"/>
          <geom mesh="base_profile_short" class="collision"/>

          <body name="Lift_Middle" pos="-0.5094 -0.20955 0.0003" quat="0 -0.707107 0 0.707107">
             <joint name="lift_joint" class="telescope"/>
             <geom mesh="Lift_Middle" class="visual" material="Lift_Middle_material"/>
             <geom mesh="Lift_Middle" class="collision"/>

             <body name="Lift_Top" pos="0 0 0">
               <geom mesh="Lift_Top" class="visual" material="Lift_Middle_material"/>
               <geom mesh="Lift_Top" class="collision"/>

               <!-- Left Arm (7 DOF) -->
                <body name="left_arm_base_link" pos="-0.005 -0.125 0.0575" euler="1.5708 0 0" childclass="nero" gravcomp="1">
                  <inertial pos="-0.00963615 0.00016787 0.0448601" mass="0.77" diaginertia="0.0006669 0.00054501 0.00051509" />
                  <geom mesh="base_link" class="visual" material="light_medium_gray_mat"/>
                  <geom mesh="base_link" class="collision"/>

                  <body name="left_arm_link1" pos="0 0 0.138" quat="1.0 0.0 0.0 0.0" gravcomp="1">
                    <inertial pos="-0.00077 -0.00006 -0.00686" mass="0.77" diaginertia="0.0006669 0.00054501 0.00051509" />
                    <joint name="left_arm_joint1" axis="0 0 1" range="-2.7 2.7"/>
                    <geom mesh="link1" class="visual" material="gray_mat"/>
                    <geom mesh="link1" class="collision"/>

                    <body name="left_arm_link2" pos="0 0 0" quat="0.000033 0.000033 0.707105 -0.707108" gravcomp="1">
                      <inertial pos="-0.00011 -0.06072 0.00148" mass="0.49" diaginertia="0.00053808 0.00029435 0.00040266" />
                      <joint name="left_arm_joint2" axis="0 0 1" range="-1.74 1.74"/>
                      <geom mesh="link2" class="visual" material="light_medium_gray_mat"/>
                      <geom mesh="link2" class="collision"/>

                      <body name="left_arm_link3" pos="0 -0.31 0" quat="-0.000003 0.000003 -0.707108 0.707105" gravcomp="1">
                        <inertial pos="0.00014 0.00211 -0.04853" mass="0.67" diaginertia="0.00342528 0.0033924 0.00026169" />
                        <joint name="left_arm_joint3" axis="0 0 1" range="-2.75 2.75"/>
                        <geom mesh="link3" class="visual" material="gray_mat"/>
                        <geom mesh="link3" class="collision"/>

                        <body name="left_arm_link4" pos="0 0 0" quat="-0.000003 0.000003 -0.707108 0.707105" gravcomp="1">
                          <inertial pos="-0.00013 -0.05482 0.00132" mass="0.41" diaginertia="0.00031331 0.00013204 0.0002393" />
                          <joint name="left_arm_joint4" axis="0 0 1" range="0 2.14"/>
                          <geom mesh="link4" class="visual" material="light_medium_gray_mat"/>
                          <geom mesh="link4" class="collision"/>

                          <body name="left_arm_link5" pos="0 -0.27001 0" quat="0.499998 0.500000 -0.500000 0.500002" gravcomp="1">
                            <inertial pos="0.00303 0.00617 -0.11778" mass="0.56" diaginertia="0.0008249 0.00078548 0.00021015" />
                            <joint name="left_arm_joint5" axis="0 0 1" range="-2.75 2.75"/>
                            <geom mesh="link5" class="visual" material="gray_mat"/>
                            <geom mesh="link5" class="collision"/>

                            <body name="left_arm_link6" pos="0 0 0" quat="0.499998 0.500000 -0.500000 0.500002" gravcomp="1">
                              <inertial pos="0.00014 -0.0027 -0.00025" mass="0.31" diaginertia="0.00008181 0.00007176 0.00007906" />
                              <joint name="left_arm_joint6" axis="0 0 1" range="-0.73 0.95"/>
                              <geom mesh="link6" class="visual" material="light_medium_gray_mat"/>
                              <geom mesh="link6" class="collision"/>

                              <body name="left_arm_link7" pos="0 -0.0235 0" euler="1.5708 0 -1.5708" gravcomp="1">
                                <inertial pos="-0.00014 -0.0001 -0.00275" mass="0.000001" diaginertia="1.1e-07 1.1e-07 2e-07" />
                                <joint name="left_arm_joint7" axis="0 0 1" range="-1.57 1.57"/>
                                <geom mesh="link7" class="visual" material="gray_mat"/>
                                <geom mesh="link7" class="collision"/>

                                <body name="left_arm_end_effector" pos="0 0.033 -0.0235" gravcomp="1">
                                  <inertial pos="-0.01208 -0.00025 0.00032" mass="0.05" diaginertia="2.697e-05 4.311e-05 2.479e-05" />
                                  <geom mesh="stand_v2" euler="-1.57 0 -1.57" class="visual" material="gray_mat"/>
                                  <geom mesh="stand_v2" euler="-1.57 0 -1.57" class="collision"/>
                                  <site name="left_arm_ee" pos="0 0 0.0"/>
                                </body>
                              </body>
                            </body>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>

               <!-- Right Arm (7 DOF) -->
                <body name="right_arm_base_link" pos="-0.005 0.125 0.0575" euler="-1.5708 0 0" childclass="nero" gravcomp="1">
                  <inertial pos="-0.00963615 0.00016787 0.0448601" mass="0.77" diaginertia="0.0006669 0.00054501 0.00051509" />
                  <geom mesh="base_link" class="visual" material="light_medium_gray_mat"/>
                  <geom mesh="base_link" class="collision"/>

                  <body name="right_arm_link1" pos="0 0 0.138" quat="1.0 0.0 0.0 0.0" gravcomp="1">
                    <inertial pos="-0.00077 -0.00006 -0.00686" mass="0.77" diaginertia="0.0006669 0.00054501 0.00051509" />
                    <joint name="right_arm_joint1" axis="0 0 1" range="-2.7 2.7"/>
                    <geom mesh="link1" class="visual" material="gray_mat"/>
                    <geom mesh="link1" class="collision"/>

                    <body name="right_arm_link2" pos="0 0 0" quat="0.000033 0.000033 0.707105 -0.707108" gravcomp="1">
                      <inertial pos="-0.00011 -0.06072 0.00148" mass="0.49" diaginertia="0.00053808 0.00029435 0.00040266" />
                      <joint name="right_arm_joint2" axis="0 0 1" range="-1.74 1.74"/>
                      <geom mesh="link2" class="visual" material="light_medium_gray_mat"/>
                      <geom mesh="link2" class="collision"/>

                      <body name="right_arm_link3" pos="0 -0.31 0" quat="-0.000003 0.000003 -0.707108 0.707105" gravcomp="1">
                        <inertial pos="0.00014 0.00211 -0.04853" mass="0.67" diaginertia="0.00342528 0.0033924 0.00026169" />
                        <joint name="right_arm_joint3" axis="0 0 1" range="-2.75 2.75"/>
                        <geom mesh="link3" class="visual" material="gray_mat"/>
                        <geom mesh="link3" class="collision"/>

                        <body name="right_arm_link4" pos="0 0 0" quat="-0.000003 0.000003 -0.707108 0.707105" gravcomp="1">
                          <inertial pos="-0.00013 -0.05482 0.00132" mass="0.41" diaginertia="0.00031331 0.00013204 0.0002393" />
                          <joint name="right_arm_joint4" axis="0 0 1" range="0 2.14"/>
                          <geom mesh="link4" class="visual" material="light_medium_gray_mat"/>
                          <geom mesh="link4" class="collision"/>

                          <body name="right_arm_link5" pos="0 -0.27001 0" quat="0.499998 0.500000 -0.500000 0.500002" gravcomp="1">
                            <inertial pos="0.00303 0.00617 -0.11778" mass="0.56" diaginertia="0.0008249 0.00078548 0.00021015" />
                            <joint name="right_arm_joint5" axis="0 0 1" range="-2.75 2.75"/>
                            <geom mesh="link5" class="visual" material="gray_mat"/>
                            <geom mesh="link5" class="collision"/>

                            <body name="right_arm_link6" pos="0 0 0" quat="0.499998 0.500000 -0.500000 0.500002" gravcomp="1">
                              <inertial pos="0.00014 -0.0027 -0.00025" mass="0.31" diaginertia="0.00008181 0.00007176 0.00007906" />
                              <joint name="right_arm_joint6" axis="0 0 1" range="-0.73 0.95"/>
                              <geom mesh="link6" class="visual" material="light_medium_gray_mat"/>
                              <geom mesh="link6" class="collision"/>

                              <body name="right_arm_link7" pos="0 -0.0235 0" euler="1.5708 0 -1.5708" gravcomp="1">
                                <inertial pos="-0.00014 -0.0001 -0.00275" mass="0.000001" diaginertia="1.1e-07 1.1e-07 2e-07" />
                                <joint name="right_arm_joint7" axis="0 0 1" range="-1.57 1.57"/>
                                <geom mesh="link7" class="visual" material="gray_mat"/>
                                <geom mesh="link7" class="collision"/>

                                <body name="right_arm_end_effector" pos="0 0.033 -0.0235" gravcomp="1">
                                  <inertial pos="-0.01208 -0.00025 0.00032" mass="0.05" diaginertia="2.697e-05 4.311e-05 2.479e-05" />
                                  <geom mesh="stand_v2" euler="-1.57 0 -1.57" class="visual" material="gray_mat"/>
                                  <geom mesh="stand_v2" euler="-1.57 0 -1.57" class="collision"/>
                                  <site name="right_arm_ee" pos="0 0 0.0"/>
                                </body>
                              </body>
                            </body>
                          </body>
                        </body>
                      </body>
                    </body>
                  </body>
                </body>

             </body>
          </body>
       </body>
    </body>
  </worldbody>

  <actuator>
    <position name="left_arm_joint1_pos" joint="left_arm_joint1" class="nero" kp="15" kv="1.5"/>
    <position name="left_arm_joint2_pos" joint="left_arm_joint2" class="nero" kp="15" kv="1.5"/>
    <position name="left_arm_joint3_pos" joint="left_arm_joint3" class="nero" kp="15" kv="1.5"/>
    <position name="left_arm_joint4_pos" joint="left_arm_joint4" class="nero" kp="15" kv="1.5"/>
    <position name="left_arm_joint5_pos" joint="left_arm_joint5" class="nero" kp="15" kv="1.5"/>
    <position name="left_arm_joint6_pos" joint="left_arm_joint6" class="nero" kp="15" kv="1.5"/>
    <position name="left_arm_joint7_pos" joint="left_arm_joint7" class="nero" kp="15" kv="1.5"/>

    <position name="right_arm_joint1_pos" joint="right_arm_joint1" class="nero" kp="15" kv="1.5"/>
    <position name="right_arm_joint2_pos" joint="right_arm_joint2" class="nero" kp="15" kv="1.5"/>
    <position name="right_arm_joint3_pos" joint="right_arm_joint3" class="nero" kp="15" kv="1.5"/>
    <position name="right_arm_joint4_pos" joint="right_arm_joint4" class="nero" kp="15" kv="1.5"/>
    <position name="right_arm_joint5_pos" joint="right_arm_joint5" class="nero" kp="15" kv="1.5"/>
    <position name="right_arm_joint6_pos" joint="right_arm_joint6" class="nero" kp="15" kv="1.5"/>
    <position name="right_arm_joint7_pos" joint="right_arm_joint7" class="nero" kp="15" kv="1.5"/>

    <!--position name="Lift" joint="lift_joint" class="telescope" --> <!-- Dummy/Placeholder for lift if we had it, omitting for now or adding if needed -->
  </actuator>

  <contact>
    <exclude body1="left_arm_base_link" body2="left_arm_link1"/>
    <exclude body1="left_arm_link5" body2="left_arm_link6"/>
    <exclude body1="left_arm_link6" body2="left_arm_link7"/>
    <exclude body1="left_arm_link7" body2="left_arm_end_effector"/>

    <exclude body1="right_arm_base_link" body2="right_arm_link1"/>
    <exclude body1="right_arm_link5" body2="right_arm_link6"/>
    <exclude body1="right_arm_link6" body2="right_arm_link7"/>
    <exclude body1="right_arm_link7" body2="right_arm_end_effector"/>

    <exclude body1="Lift_Top" body2="left_arm_base_link"/>
    <exclude body1="Lift_Top" body2="right_arm_base_link"/>
  </contact>

</mujoco>
```

Notes tied to earlier sections: the `lift_joint` (§8) has no live actuator here, only the commented-out
placeholder above; and there is no `<keyframe>`/`home` key defined (§4), so `SingleArmIK.get_home_q()`
would raise if ever called against this file.

## 10. Reconstructing `q_targets` from collected cartesian action states

This mirrors exactly what `ArmNode.set_ee_target` → `SingleArmIK.solve_ik` do, per arm, per recorded action
state.

1. **Build the model.** Load the MJCF in §9 and build one `mink.Configuration` per arm (left/right are
   independent — §6). Each arm only actuates its own 7 joints (`left_arm_joint1..7` /
   `right_arm_joint1..7`); the IK target frame is the `<site>` `left_arm_ee` / `right_arm_ee`.

2. **Seed once per episode, per arm** (§4): initialize each arm's configuration from that episode's first
   recorded joint state (or the earliest measured joint state available) — not a canonical/home pose — and
   set the `PostureTask` target from that same initial configuration. This target is fixed for the whole
   episode; it does not move as the episode progresses (mirrors `init()` being called once at startup, not
   per timestep).

3. **Confirm the frame convention of the recorded data** (§5): determine whether the recorded cartesian
   action states are already the post-handedness-fix, post-`H`, post-clutch `ee_target` SE3 values (i.e.
   captured at or after the `set_bimanual_ee_target`/`set_ee_target` call). If so, use them directly as the
   `FrameTask` target with no further transform. Only apply the VR handedness fix (`M = diag(1,1,-1)`), the
   `H` frame remap, and the clutch-delta formula
   (`X_Cdelta = X_Cinit⁻¹·X_Ctarget`; `X_Rdelta = H⁻¹·X_Cdelta·H`;
   `target_pos = X_ee_init.translation() + X_Rdelta.translation()`;
   `target_rot = X_ee_init.rotation() @ X_Rdelta.rotation()`) if reconstructing from raw VR controller
   telemetry instead of from the already-computed cartesian targets.

4. **Solve sequentially, per arm, warm-starting from the previous solution** (§4, §6, §7) — do not re-seed
   fresh from the episode start on every timestep, and do not solve timesteps out of order or in parallel.
   For each timestep `t`, in order:
   - Set the `FrameTask` target to that timestep's `ee_target` SE3 (position weight `1.0`, orientation
     weight `0.1`, `lm_damping=1.0`).
   - Run up to 10 iterations of: velocity-IK solve over tasks
     `[FrameTask, PostureTask(cost=[1e-3] per this arm's 7 joints)]`, `solver="daqp"`, `damping=1e-5`,
     limited by `ConfigurationLimit`, then integrate the resulting velocity into the configuration at
     `dt=0.01`.
   - Break early once position error ≤ `1e-3` m and orientation error ≤ `1e-3` (rad).
   - The resulting configuration's values at this arm's 7 `dof_ids` are the `q_target` for timestep `t`.
   - Carry this same configuration (not a reset one) into the solve for timestep `t+1`.

5. **Repeat independently for the other arm** — separate configuration and warm-start chain, since the two
   arms have no cross-arm coupling in the live system (§6).

6. **Do not run the lift through this pipeline** (§8). If the collected data has a lift dimension, treat it
   as its own independent up/down/stop (or absolute-height) signal alongside the two arms' `q_targets`,
   never as part of the arm IK.

7. **Fidelity caveat.** mink's `PostureTask`/`ConfigurationLimit`/damping make this a nonlinear,
   warm-start-dependent solve. Exact bit-for-bit reproduction of the original online `q_targets` requires
   reconstructing the *same sequential warm-start chain* the live run used — solving frames out of order,
   in parallel, or from a different seed will generally converge to a different (but still valid, low
   pose-error) IK solution for the same cartesian target, since the posture regularizer and
   joint-limit-respecting QP admit multiple nearby solutions.
