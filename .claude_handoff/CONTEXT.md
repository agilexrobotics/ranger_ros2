# ranger_ros2 sim port — running context

This file is the append-only running journal of the ROS 1 → ROS 2 Jazzy
port of the AgileX Ranger Mini v3 model. It exists so that a fresh
session (or a fresh teammate) can re-bootstrap by reading just this file
plus the matching `round_NN_*.md` handoff in this same directory.

## Workspace layout

- Workspace root:  `~/agilex_ws/`
- Repo:            `~/agilex_ws/src/ranger_ros2/` (this repo, multi-package)
- Sibling vendor:  `~/agilex_ws/src/ugv_sdk/` (do not modify)
- Target ROS:      ROS 2 Jazzy on Ubuntu 24.04
- Target sim:      Gazebo Harmonic (gz-sim 8.x)

## Phase plan

| Phase | Branch                       | Package                              | Status              |
|-------|------------------------------|--------------------------------------|---------------------|
| 1     | `phase-1-description`        | `ranger_mini_v3_description`         | DONE (R01, R02 fix) |
| 2     | `phase-2-ros2-control`       | `ranger_mini_v3_sim` (ros2_control)  | not started         |
| 3     | `phase-3-gazebo-bringup`     | `ranger_mini_v3_sim` (Gazebo launch) | not started         |
| 4     | `phase-4-messenger-node`     | `ranger_mini_v3_sim_bringup`         | not started         |
| 5     | `phase-5-interface-parity`   | (real-driver topic parity)           | not started         |

**Branch base policy (architect decision, R02):** All phase branches
are based on `jazzy`, NOT `humble` or `main` (no `main` exists).
Phase 1 was rebased from `humble` → `jazzy` in R02.

## Sacred constants (do not relitigate)

- Wheel radius `0.09 m`, wheelbase `0.50 m`, track `0.38 m`.
- Steering joint axis is `(0, 0, -1)` — keep this. Real driver assumes
  positive command = CW from above.
- Asymmetric `rr_steering_joint` origin from the original ROS 1 model
  is fixed to `(-0.25, -0.19, -0.1)`. Do not regress.
- Real-driver topic interface that sim must match:
  - sub: `/cmd_vel`           (geometry_msgs/Twist)
  - pub: `/odom`              (nav_msgs/Odometry)
        `/system_state`       (ranger_msgs/SystemState)
        `/motion_state`       (ranger_msgs/MotionState)
        `/actuator_state`     (ranger_msgs/ActuatorStateArray)
        `/battery_state`      (sensor_msgs/BatteryState)
  - tf: `odom -> base_link` (gated by `publish_odom_tf`)
- Mode switching (sim must mirror real driver):
  lateral-y dominant → parallel; pure ω → spinning; otherwise dual-Ackermann.

## Hard non-goals

Do NOT modify (without an explicit prompt instruction): `ranger_base/`,
`ranger_bringup/`, `ranger_msgs/`, `ugv_sdk/`, anything outside
`~/agilex_ws/src/ranger_ros2/`. We DEPEND on `ranger_msgs` but never
edit it.

---

## Round log

### 2026-05-11 — Round 01 — Phase 1 description package import

- **Branch:** `phase-1-description` (created from `humble`; see deviation
  note below — the bootstrap nominally specifies `main`, but no `main`
  branch exists in the pre-existing repo).
- **Package added:** `ranger_mini_v3_description/` — pure URDF/xacro +
  meshes, no Gazebo / ros2_control dependencies. Reusable by
  nav2/MoveIt/RViz.
- **Commits on branch:**
  - `a876c9b` `feat(sim): add ranger_mini_v3_description package (phase 1)`
  - + `docs(handoff): round 01 description package import`
- **Files created (this round):**
  - `ranger_mini_v3_description/{package.xml, CMakeLists.txt, README.md}`
  - `ranger_mini_v3_description/urdf/ranger_mini_v3.xacro` (161 lines)
  - `ranger_mini_v3_description/launch/display.launch.py`
  - `ranger_mini_v3_description/rviz/display.rviz`
  - `ranger_mini_v3_description/meshes/{ranger_base,steering_wheel,wheel_v3}.dae`
    (binary, ≈37.8 MiB total)
  - `ranger_mini_v3_description/config/` (empty, **not** in git — see
    open question 2 in round_01 handoff)
  - `.claude_handoff/{BOOTSTRAP_ACK.md, round_01_description.md, CONTEXT.md}`
- **Verification done:** xacro parses (275-line URDF, exit 0); colcon
  builds the package alone in ~1.2 s; `ros2 pkg prefix` finds it under
  `install/`; `ros2 launch ... --print` produces a valid launch
  description. RViz visual verification deferred to the operator (see
  "Manual launch steps for the operator" in `round_01_description.md`).
- **Deviations:** branched from `humble`, not `main` (no `main` exists);
  empty `config/` dir from the zip is not git-tracked; commit author
  comes from the host's git config (`solanrewaju2020@fau.edu`),
  unrelated to the placeholder maintainer in `package.xml`.
- **Open questions for architect:** (1) base-branch policy — keep
  `humble`, switch to `jazzy`, or rename to `main`? (2) what to do
  about the empty `config/` dir vs. the `install(DIRECTORY ... config
  ...)` line in CMakeLists; (3) update `package.xml` maintainer field
  or leave as a per-deployer placeholder? Full detail in
  `round_01_description.md`. **All three resolved in R02.**

### 2026-05-12 — Round 02 — Phase 1 ground-plane fix + open-question resolutions

- **Branch:** still `phase-1-description`, now rebased onto `jazzy`
  (clean — no conflicts; 11 commits replayed).
- **Architect resolutions applied:**
  - Q1 (branch base): target is `jazzy`, not `humble`. Future phases
    branch from `jazzy`.
  - Q2 (empty `config/`): dropped `config` from
    `install(DIRECTORY ...)` and removed the empty dir; it returns in
    Phase 2 with actual content.
  - Q3 (maintainer): updated `package.xml` to
    `Shuaib Olanrewaju <solanrewaju2020@fau.edu>` per host git config.
- **The actual fix:** RViz Fixed Frame changed from `base_link` to
  `base_footprint`. The URDF was correct — `base_footprint` already
  sits at ground level under `base_link` via the
  `base_footprint_joint`. Round 01 just had RViz pointed at the wrong
  frame, which made the wheels appear to render below the grid.
- **TF tree is unchanged.** `base_link` remains the URDF root for
  real-driver TF parity (the real driver publishes `odom -> base_link`,
  so the sim's messenger node must too in Phase 4).
- **Nav2 interop note:** consumers can use `base_link` as the body
  frame (matching the real driver), or treat `base_footprint` as the
  ground-projected pose; the static `base_footprint_joint` link makes
  both available. No additional `odom -> base_footprint` static
  broadcaster needed unless a downstream stack specifically requires
  it.
- **Commits added on branch (this round):**
  - `dd72af7` `fix(sim): ground rviz on base_footprint; clean up phase 1`
  - + `docs(handoff): round 02 phase 1 ground-plane fix`
- **Files added/modified this round:**
  - `ranger_mini_v3_description/CMakeLists.txt` (drop `config` from
    install dirs)
  - `ranger_mini_v3_description/package.xml` (maintainer)
  - `ranger_mini_v3_description/rviz/display.rviz` (Fixed Frame)
  - `ranger_mini_v3_description/config/` removed (was empty)
  - `.claude_handoff/.gitignore` (new — excludes editor metadata so
    `git add -A` stays clean across rounds)
  - `.claude_handoff/round_02_phase1_fixes.md` (new)
- **Verification:** xacro still parses (275 lines, exit 0); rebuild
  ~0.4 s; installed `display.rviz` confirmed to carry
  `Fixed Frame: base_footprint`. RViz visual re-check deferred to the
  operator (steps in `round_02_phase1_fixes.md`).
- **Deviations:** the `git rebase jazzy` invocation replayed 9
  upstream `humble`-only commits (smalleha + agilexrobotics) onto
  `phase-1-description`. They modify `ranger_base/`, `ranger_bringup/`,
  and root `README.md`, which are bootstrap-protected dirs. I did not
  author or edit them; they came in via the literal rebase command. A
  local `jazzy` branch had to be created (`git branch jazzy
  origin/jazzy`) for the literal command to work. Source-fix commit
  used `git add <files>` rather than `-A` so the handoff `.gitignore`
  stays in the handoff commit.
- **Open questions for architect:** (1) should the 9 humble-only
  upstream commits be dropped from `phase-1-description`? They are
  outside this port's scope. Detail in `round_02_phase1_fixes.md`.
  (2) confirm the maintainer identity for the long term.

## 2026-05-12 — Rounds 03 / 03b / 03c: Phase 1 reset + remote topology

- Reset `phase-1-description` to S-abk's `origin/jazzy` tip
  (89c3308), which is 5 commits ahead of `upstream/jazzy` (a1bc206)
  because the user had already done a humble→jazzy port of Ranger
  Mini v3 real-robot support in April 2026 (commit a5609b7).
- Dropped 9 humble-only commits that touched real-robot packages;
  most of their substance (CalculateSteeringAngle hardening,
  TwistCmdCallback Ackermann/side-slip logic, v3 launch files,
  MotorState fields) is already present in S-abk's a5609b7.
- Added repo-level .gitignore for editor sidecars, OS junk,
  Python cache, defensive colcon dirs.
- Remote topology:
    origin   = git@github.com:S-abk/ranger_ros2.git   (SSH)
    upstream = https://github.com/agilexrobotics/ranger_ros2.git
  Sim work pushes only to origin. Upstream is fetch-only for future
  AgileX driver updates.
- Policy clarification: "do not modify ranger_base/ranger_bringup/
  ranger_msgs" applies to our changes only. Inherited state from
  prior user work on those packages is the baseline we build on.
- For Phase 4 messenger-node implementation: the authoritative
  Twist callback / kinematic logic to mirror lives in S-abk's
  `ranger_base/src/ranger_messenger.cpp` from a5609b7, NOT in the
  older humble version.
- Handoff strategy unchanged: .claude_handoff/ tracked on phase
  branches; phases squash-merge into jazzy at integration time.


## 2026-05-12 — Round 04c: Squash-merge phase-1 into jazzy + Phase 2 complete

- Squash-merged phase-1-description into jazzy. Fork's jazzy
  now contains ranger_mini_v3_description package + repo
  .gitignore. phase-1-description branch retained on origin
  for forensic history; no further development on it.
- Eliminates the cross-branch --symlink-install dangling
  symlink trap from Round 04b. Phase 3, 4, 5 develop in a
  single source tree against jazzy.
- phase-2-ros2-control rebased onto updated jazzy; phase-2
  commit got a new SHA (parent changed). Force-pushed.
- Phase 2 verification complete: ros2_control block expands,
  gz_ros2_control plugin tag present, controller YAML resolves,
  8 command_interfaces + 16 state_interfaces as expected.
- gz_ros2_control plugin filename verified against installed
  binaries (see round handoff for actual filename and whether
  xacro needs updating in Phase 3).
- Phase 2 itself does NOT start Gazebo or controller_manager
  — that's Phase 3.


## 2026-05-12 — Round 04d: Phase 3 prerequisites

- Operator installed ros-jazzy-ros-gz-sim, ros-jazzy-ros-gz-bridge,
  ros-jazzy-controller-manager, ros-jazzy-joint-state-broadcaster,
  ros-jazzy-position-controllers, ros-jazzy-velocity-controllers,
  ros-jazzy-gz-ros2-control (the sudo step is operator-side per
  policy decision in this round).
- Verified gz sim 8.x runs headless (gz sim -s -r empty.sdf)
  and ros_gz_bridge can bridge /clock end-to-end.
- Verified gz_ros2_control plugin class names against the
  installed libgz_ros2_control-system.so. See round_04d handoff
  for exact strings and whether xacro needs updating in Phase 3.
- Pruned stale Copilot-style remote branch.


## 2026-05-12 — Round 05: Phase 3 Checkpoints A+B

- Fixed gz_ros2_control plugin class-name: GazeboSimROS2ControlPlugin
  (was ControlSystem; mismatch found in R04d).
- Added worlds/empty_ground.sdf and launch/gazebo.launch.py to
  ranger_mini_v3_sim. Launch starts gz sim (headless default),
  robot_state_publisher, /clock bridge, and spawns the robot
  at (0,0,0.32). No controllers yet.
- Phase 3 round 06 will add controller spawners and verify
  joint commands move the robot.
- Phase 3 work remains on phase-2-ros2-control branch (small
  additive work to phase-2's package, no new package created).


## 2026-05-12 — Round 06: Phase 3 Checkpoints C+D

- Added gazebo_full.launch.py composing gazebo.launch.py +
  sequential spawning of all 9 controllers via OnProcessExit
  chain.
- Confirmed all 9 controllers load and activate: 1
  joint_state_broadcaster, 4 *_steering_position_controller,
  4 *_wheel_velocity_controller.
- /joint_states flows at ~100 Hz, TF resolves for all
  continuous joints, raw Float64MultiArray commands to
  controller /commands topics move the corresponding joints.
- Phase 3 done. Phase 4 (ranger_sim_messenger node) is next:
  subscribe /cmd_vel, implement 4WS kinematics with the four
  motion modes, publish to the 8 controller command topics,
  publish /odom + /system_state + /motion_state +
  /actuator_state + /battery_state to match the real driver.


## 2026-05-12 — Rounds 07 / 07b / 07c: Phase 4 setup + DUAL_ACKERMAN complete

- Squash-merged phase-2-ros2-control into jazzy (R07).
- URDF dimensions aligned with real-robot RangerMiniV3Params:
  wheelbase 0.494 m, track 0.364 m (R07).
- Created phase-4-messenger branch off updated jazzy.
- Built ranger_msgs (R07b), ugv_sdk (R07c), ranger_base,
  ranger_bringup as part of full workspace sweep. All
  siblings now in install/ — no more "missing dep" surprises.
- Created ranger_mini_v3_sim_messenger ament_python package
  and implemented sim_messenger node:
  - Subscribes /cmd_vel, publishes /odom + 8 controller cmds
  - DUAL_ACKERMAN mode math ported from real driver
  - Per-wheel split: inner = atan(W/2 / R),
                      outer = atan(W/2 / (R+T)),
                      front/rear mirror-symmetric
  - RK4 odometry integrator (10 substeps per dt)
- PARALLEL / SPINNING modes recognized but commands zeroed
  + one-shot warning. Round 08 implements them.
- Per-wheel speed uniform (linear.x / wheel_radius) in R07;
  ICR-aware per-wheel scaling deferred to Round 08 if visible
  slip becomes a problem.
- KNOWN BUG (R07c): steering-sign convention — messenger
  outputs positive wheel angles for left-turn cmd_vel, but URDF
  axis (0,0,-1) interprets positive as CW (right turn). Forward
  drive is fine; arc tests turn the wrong direction in gz while
  /odom integrates the commanded direction. See round 07c
  handoff Open Questions for fix options. Recommend Option A
  (sign-flip in messenger).


## 2026-05-12 — Round 08: Fix URDF axis + PARALLEL/SPINNING modes

- **CORRECTION to earlier bootstrap rule:** URDF steering axis
  is now (0, 0, 1), not (0, 0, -1). The earlier rule was
  based on incomplete understanding of the real driver's
  internal angle convention. Standard ROS REP-103 applies:
  positive joint command = CCW about z = left turn.
- Implemented PARALLEL mode: common steering angle =
  atan2(linear.y, linear.x), speed = hypot. Side-slip
  sub-case (x=0,y!=0) handled with last_nonzero_x.
- Implemented SPINNING mode: tangent-to-radial wheel angles
  (with joint-range wrap + velocity sign flip on wrap);
  wheel speeds sized for commanded body angular velocity.
- /odom integration uses ParallelModel and SpinningModel
  per real driver's kinematics_model.hpp.
- Fixed inherited R07 bug: calculate_steering_angle had a
  ZeroDivisionError on pure-spin commands (linear.x=0).
  Smalleha's div-by-zero guard backported to Python.
- All four motion modes now functional. R09 adds the mock
  state publishers (/system_state, /motion_state, etc.).


## 2026-05-12 — Round 09: Phase 4 complete (state publishers)

- Added /system_state, /motion_state, /actuator_state,
  /battery_state publishers to sim_messenger. Topic types
  and field population match real driver exactly
  (ranger_base/src/ranger_messenger.cpp L195-283).
- Subscribed to /joint_states to populate ActuatorStateArray
  with real gz values for motor_angles and motor_speeds.
- Static defaults for fields not modeled in sim:
    battery 24V, driver temp 35°C, motor temp 40°C,
    current 0A (motor) / -1A (battery), SoC 1.0.
- Cleaned up stale xacro comment about (0,0,-1) convention
  (corrected in R08 but the rationale comment was left
  behind).
- Added PARALLEL side-slip sign-combination unit tests
  (R08 OQ4). All 6 colcon tests pass.
- **Phase 4 complete.** Sim now has full interface parity
  with the real driver: same topic names, types, motion-
  mode semantics. Application code (teleop, nav2, behavior
  trees) is sim/real-portable.

Remaining work:
- Squash-merge phase-4-messenger to jazzy.
- Phase 5: interface-parity audit against the real driver
  (turn on the real driver, run sim+real side-by-side,
  confirm topics + QoS match).
- Phase 6 (optional): polish — sensors, worlds, sim time
  tuning, ros_gz_bridge static transforms cleanup.


## 2026-05-12 — Rounds 10 + 11 + 11b: /clock QoS hardening

- R10 observed /odom stuck (gz moving, messenger frozen).
- R11 baseline couldn't reproduce: mean_dt = 0.0200s,
  /odom and gz agreed within 5mm. R10 symptom appears
  state-dependent.
- R11b applied defensive QoS fix anyway: explicit RELIABLE
  + KEEP_LAST(1) on /clock via parameter_bridge YAML
  config form.
- Added permanent tick_diag INFO log (every 50 ticks) for
  future clock-issue visibility.
- All four motion modes still functional post-fix.

Phase 4 work complete. Next: squash-merge phase-4-messenger
to jazzy.


## 2026-05-12 — Round 13: Sim README + R12 correction

- Added ranger_mini_v3_sim/README.md documenting the sim
  launch file hierarchy. The key clarification: use
  gazebo_full.launch.py (controllers loaded), not the
  shorter-named gazebo.launch.py (no controllers, robot
  will appear stationary).
- Added a prominent warning docstring at the top of
  gazebo.launch.py pointing users to gazebo_full.launch.py.
- Appended an architect correction to R12's handoff: the
  "transient DDS state" conclusion was wrong; the actual
  cause was using the wrong launch file.

Phase 4 is genuinely complete and functional. Recommended
next: squash-merge phase-4-messenger to jazzy.



## Standing principle (added after R13 retro)

**When a user reports a symptom, the first diagnostic step is
"what command did you actually type?" — not "let me run my own
commands and see if I can reproduce."**

R12 spent a full round chasing a phantom DDS state issue.
Claude Code's diagnostic ran the correct launch command;
the user had been running the wrong one. The verification
showed everything working but proved nothing about the
user's experience. The two were testing different things.

Future blocked / failing rounds that follow a user-reported
symptom should:
1. First ask the user for the literal commands they typed and
   the verbatim error / observation they saw.
2. Reproduce on the user's exact command sequence before
   running anything else.
3. Only then run independent diagnostics if step 2 doesn't
   reveal the cause.

Cost: one short user message. Benefit: avoiding the failure
mode where a confounded variable (different launch command,
different env, different terminal state) leads to a
confident wrong diagnosis.
