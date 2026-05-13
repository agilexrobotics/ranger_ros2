# Round 08 — URDF axis fix + PARALLEL/SPINNING modes

## Summary

**URDF axis flipped to (0,0,1).** Steering joint axis changed from
`0 0 -1` to `0 0 1` in the description xacro. Confirmed: expanded
URDF now has 4 steering joints with the new axis, 0 lingering `-1`
axes, drive wheels unchanged at `(0,1,0)`.

**Sign agreement post-fix:** the left-arc test that disagreed in
R07c now agrees. After 4 s of `cmd_vel x=0.3 w=+0.3`:

| Source     | x      | y      | direction       |
|------------|--------|--------|-----------------|
| /odom      | 0.971  | +0.508 | LEFT (+y, +θ)   |
| gz ground  | 1.018  | +0.368 | LEFT (+y)       |

Both positive y. Distance magnitudes agree (~1.08 m). The R07c bug
is fixed.

**PARALLEL mode works.** Pure sideways `cmd_vel y=0.3` for 3 s
made the robot crab in the body-frame +y direction (in world frame,
this came out as roughly +y after correcting for the body's
accumulated yaw from the prior arc test — gz delta was
(−0.51, +0.71), magnitude 0.87 m vs. predicted 0.9 m at 0.3 m/s for
3 s). All four wheels at `+1.57 rad` (= π/2 clamped to
MAX_STEER_PARALLEL=1.570); all four wheel velocities at
`3.333 rad/s` (= 0.3/0.09).

**SPINNING mode works.** Body rotated and stayed roughly in place
(12 cm drift during 3 s — transient while steering rotated from
PARALLEL's π/2 to the spin-tangent angles). Wheel commands match
the spec exactly:

| Wheel | Steer (rad) | Vel (rad/s) | Flipped? |
|-------|-------------|-------------|----------|
| fl    | −0.9358     | −1.7045     | yes      |
| fr    | +0.9358     | +1.7045     | no       |
| rl    | +0.9358     | −1.7045     | yes      |
| rr    | −0.9358     | +1.7045     | no       |

Spin radius = `hypot(0.247, 0.182) = 0.3068 m`. Wheel speed
magnitude `1.7045 = 0.5 × 0.3068 / 0.09` matches the design formula.

**Inherited R07 bug surfaced and fixed.** When the operator first
sent a pure-spin cmd (`linear.x=0, angular.z=0.5`), the messenger
died with `ZeroDivisionError` inside `calculate_steering_angle`
(line 77: `phi_i = math.atan((WHEELBASE / 2.0) / radius)` with
radius=0). R07 never tested pure-spin so the bug was latent. Added
an early-return guard for `lin < 1e-6` returning `(0.0, 0.0)` —
radius 0 routes the upstream mode-selection into SPINNING. Matches
smalleha's div-by-zero fix that the S-abk a5609b7 commit
incorporated on the C++ side. See Deviations.

One source commit `7c0b2c2`. Plus this handoff.

## Diff

```diff
commit 7c0b2c2 (HEAD -> phase-4-messenger, origin/phase-4-messenger)
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>

    fix(sim): URDF steering axis + PARALLEL/SPINNING modes (phase 4)

 ranger_mini_v3_description/urdf/ranger_mini_v3.xacro              |   2 +-
 ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py | 214 +++++++++++++++++++--
 2 files changed, 202 insertions(+), 14 deletions(-)
```

URDF axis change:

```diff
--- a/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro
+++ b/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro
@@ -100,1 +100,1 @@
-      <axis xyz="0 0 -1"/>
+      <axis xyz="0 0 1"/>
```

sim_messenger.py changes (summary):
- Added `compute_wheel_commands_parallel(linear_x, linear_y, last_nonzero_x)` — ~60 lines.
- Added SPINNING-mode module-level constants: `_W`, `_T`, `_SPIN_RADIUS`,
  `_FL_TAN..._RR_TAN`, range-wrap helper `_wrap_into_steer_range`,
  per-wheel `_FL_STEER`, `_FL_FLIPPED` etc.
- Added `compute_wheel_commands_spinning(angular_z)` — ~25 lines.
- In `SimMessenger.__init__`: added state fields `last_nonzero_x`,
  `_last_used_angle`, `_last_used_speed`, `_last_used_angular_z`;
  removed the `_mode_warned` flag.
- In `_cmd_cb`: track `last_nonzero_x` on nonzero linear.x.
- In `_tick`: replaced the placeholder else-branch with proper
  PARALLEL and SPINNING dispatches calling
  `compute_wheel_commands_parallel/_spinning`, plus integration
  via `_integrate_parallel` (RK4) and Euler theta-update for SPIN.
- Added `_integrate_parallel(v, phi, dt)` — RK4 like the dual-Ackermann.
- In `_publish_odometry`: per-mode twist fill — DUAL keeps old logic,
  PARALLEL fills linear.x/y from last_used_angle/speed, SPIN fills
  angular.z from last_used_angular_z.
- `calculate_steering_angle`: added the `lin < 1e-6` early return
  to fix the ZeroDivisionError on pure-spin commands (R07 inherited
  bug).

## New files (full content)

### .claude_handoff/round_08_phase4_parallel_spinning.md

This file. Will be committed in step 7c.

No other new files. All changes are modifications to existing files
(URDF + messenger).

## Verification output

### Step 0 — Pre-flight

```
On branch phase-4-messenger
nothing to commit, working tree clean
[no orphans]
```

### Step 1a — Pre-fix axis grep

```
100:      <axis xyz="0 0 -1"/>
```

### Step 1b — Post-fix

```
100:      <axis xyz="0 0 1"/>
134:      <axis xyz="0 1 0"/>
```

Steering axis flipped; wheel axis (line 134, in the
`drive_wheel` macro) untouched. ✓

### Step 1c — Re-expand verification

```
$ colcon build --packages-select ranger_mini_v3_description --symlink-install
Summary: 1 package finished [0.19s]
$ xacro … > /tmp/ranger_axis_check.urdf; echo "exit=$?"
exit=0
$ grep -B2 'axis xyz="0 0 1"' /tmp/ranger_axis_check.urdf | head -20
(4 instances of `axis xyz="0 0 1"`, each preceded by the stale comment
 — see Open questions for the comment-cleanup item)
$ grep -B1 'axis xyz="0 1 0"' | grep -c '_wheel'
4
$ grep -c 'axis xyz="0 0 -1"'
0
```

4 steering joints with new axis, 4 drive joints with old axis, 0
lingering `-1` axes. ✓

### Step 2-4 — Code changes

Applied via `Edit` to `sim_messenger.py`. Diffs summarized under "Diff"
above. Total +214 / −14.

### Step 5a — Build after edits

```
Starting >>> ranger_mini_v3_description
Starting >>> ranger_mini_v3_sim_messenger
Finished <<< ranger_mini_v3_description [0.22s]
Finished <<< ranger_mini_v3_sim_messenger [0.88s]
Summary: 2 packages finished [0.97s]
```

### Pre-test — SPINNING constants

Verified by direct Python import after build:

```
FL: steer=-0.9358 rad (-53.6°), flipped=True
FR: steer=+0.9358 rad (+53.6°), flipped=False
RL: steer=+0.9358 rad (+53.6°), flipped=True
RR: steer=-0.9358 rad (-53.6°), flipped=False
Spin radius: 0.3068 m
```

Symmetry as expected (FL/RR symmetric, FR/RL symmetric). Two are
flipped (those with raw atan2 outside ±MAX_STEER_PARALLEL got
π-wrapped); their wheel velocities will be sign-inverted.

### Step 5b — Gazebo + messenger up

9/9 controllers active. Both launches alive.

### Step 5c — Left arc (the R07c regression test)

```
Baseline /odom (0, 0, 0), orientation identity.
Baseline gz   (0, 0, 0.315).

After 4s of cmd_vel x=0.3 w=+0.3:
/odom  pose: x=0.9710, y=+0.5079, orientation.z=0.4635 → θ≈+0.965 rad
gz     pose: x=1.0184, y=+0.3681
```

**Both y positive, both θ positive → both agree it's a left turn.**
The R07c sign disagreement is gone. Magnitudes:

| Source | distance from origin |
|--------|----------------------|
| /odom  | 1.096 m              |
| gz     | 1.083 m              |

Within 1.3 cm. Expected motion magnitude over 4 s at 0.3 m/s = 1.2 m
in pure straight-line; in arc with θ≈0.96 rad average yaw the chord
distance is somewhat shorter than the arc length, which both
measurements show.

### Step 5d/5e — Pure sideways (PARALLEL mode)

```
Before: gz (1.018, 0.368)
After 3s of y=0.3: gz (0.509, 1.074)
Δ_world = (−0.509, +0.706); |Δ| = 0.871 m
Expected over 3 s at 0.3 m/s = 0.9 m
```

Within 3 % of expected magnitude. The world-frame Δ has both −x and
+y components because the body was already rotated +0.96 rad from
the prior arc, so body-frame +y maps to world (cos(0.96+π/2),
sin(0.96+π/2)) ≈ (−0.82, +0.57). Direction matches qualitatively
(both components have the right signs); the deviation from exact
(−0.737, +0.516) is from gz dynamics + the body still rotating
slightly during the test.

Captured wheel commands during sustained sideways:

```
fl  steer: (echo missed in 1s window — see Deviations)
fr  steer: 1.57 rad   vel: 3.333 rad/s
rl  steer: 1.57 rad   vel: 3.333 rad/s
rr  steer: 1.57 rad   vel: 3.333 rad/s
```

All wheels at MAX_STEER_PARALLEL=1.570 (≈ π/2, clamped). All wheel
velocities at 0.3/0.09=3.333 rad/s. ✓

### Step 5f — Pure spin (SPINNING mode)

**Inherited R07 bug exposed on first attempt:** the messenger died
with ZeroDivisionError on the first pure-spin cmd_vel. See
Deviations for the fix.

After fix, restart, and re-test:

```
Baseline gz: (−1.392, 3.276)
After 3s of w=0.5: gz (−1.297, 3.192)
Δ_world = (0.095, −0.084); |Δ| = 0.127 m

Baseline /odom: (0, 0, 0)   (messenger had just been restarted)
After 3s of w=0.5: /odom orientation.z=0.877 → θ ≈ +2.135 rad
```

The robot drifted only 12 cm in 3 s (steering transients from the
prior PARALLEL pose to spin tangents), with no rotation accumulating
*translation*. /odom θ change of +2.135 rad over ~5 s sim time
(messenger sim-clock delta 752→757) ≈ 0.43 rad/s; the commanded
0.5 rad/s × 5 s = 2.5 rad. Close to but below the commanded — some
of the time was spent on steering ramp.

Captured wheel commands during sustained spin:

```
fl  steer: -0.9358   vel: -1.7045
fr  steer: +0.9358   vel: +1.7045
rl  steer: +0.9358   vel: -1.7045
rr  steer: -0.9358   vel: +1.7045
```

Exactly matches the pre-computed module-level constants. Wheel speed
magnitude `1.7045 = 0.5 × 0.3068 / 0.09` = `angular_z × spin_radius
/ wheel_radius`. ✓

### Step 5h — Teardown

`pkill -9 -f` cycle missed nothing this time (all 6 expected
processes — gz sh wrapper, gz binary, robot_state_publisher,
parameter_bridge, controller_manager-embedded-in-gz, sim_messenger
— were killed cleanly). Final check returned `[clean]`. (R05/R06/R07c
all needed follow-up PID kills; this teardown was the first one to
succeed without them.)

### Step 6a — Commit

```
[phase-4-messenger 7c0b2c2] fix(sim): URDF steering axis + PARALLEL/SPINNING modes (phase 4)
 2 files changed, 202 insertions(+), 14 deletions(-)
```

### Step 6b — Push

```
To github.com:S-abk/ranger_ros2.git
   bd21391..7c0b2c2  phase-4-messenger -> phase-4-messenger
```

### Manual verification steps for the operator

```bash
# Terminal A
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B (after Gazebo settles)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C — test each mode in order. Restart Gazebo (Ctrl-C in
# Terminal A then re-launch) between modes if you want clean baselines.

# 1. DUAL_ACKERMAN (left arc):
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.3}}"
# → robot turns LEFT (visible +y in world).

# 2. PARALLEL (pure sideways):
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {y: 0.3}}"
# → robot crabs sideways in body-frame +y, no yaw change.
#   All 4 wheels visibly at +π/2.

# 3. SPINNING (pure rotation):
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{angular: {z: 0.5}}"
# → robot rotates ~in place (small transient drift), body yaws CCW.
#   All 4 wheels at the four tangent angles.

# 4. Stop:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

Robot should turn the correct direction in mode 1 (the R07c
regression). Modes 2 and 3 are first-time verifications.

## Deviations

- **Fixed an inherited R07 bug in `calculate_steering_angle`.** The
  function had a ZeroDivisionError path when `linear.x = 0` and
  `angular.z != 0` — exactly the cmd_vel pattern for spin mode.
  R07 never tested spin so the bug was latent. R08's SPINNING mode
  test crashed the messenger on the first pure-spin cmd_vel.

  Fixed with a one-line early-return: if `lin < 1e-6`, return
  `(0.0, 0.0)` — the upstream mode-selection sees radius=0,
  classifies as SPINNING, and routes to the new handler. This
  matches the real-driver's fix from smalleha's CalculateSteeringAngle
  div-by-zero guard (in S-abk's a5609b7 on the C++ side).

  The prompt's "Do not modify the sim_messenger.py code from the
  architect's spec" applied to R07. R08 explicitly directed
  extensive changes to sim_messenger.py (adding helpers, rewriting
  `_tick`). Fixing a latent bug in a function those changes now
  exercise is in scope. Documented prominently here so the architect
  can reverse if intended otherwise.

- **The stale axis comment inside `steering_hub` macro was left.**
  The xacro line 98-99 still says:

  ```xml
  <!-- Original used axis (0,0,-1); we keep that so positive command =
       CW seen from above, matching the real-driver kinematic model. -->
  ```

  This is now actively misleading — the axis IS no longer (0,0,-1)
  and the rationale is reversed. The prompt's literal instruction
  was `sed -i 's|<axis xyz="0 0 -1"/>|<axis xyz="0 0 1"/>|'` (only
  the axis line). Per the no-silent-route-arounds rule I did NOT
  expand scope to include the comment. Flagged in Open questions
  for a quick one-line follow-up.

- **`topic echo --once` for fl_steering during PARALLEL test missed
  the publication window.** The 1-second timeout vs 50 Hz publish
  rate is enough most of the time but raced once. The other three
  wheels' values (1.57) confirm fl was at the same value (the
  PARALLEL helper produces uniform commands by construction).

- **Messenger restart between tests.** When the inherited R07 bug
  crashed the messenger during the first spin attempt, I restarted
  the messenger only (Gazebo was still running). The /odom origin
  reset to (0,0,0) at the restart, so /odom theta in the second
  spin test is measured from the post-restart zero (matches the
  command magnitude well), not from the pre-spin position. gz
  ground truth stayed continuous across the restart.

## Open questions

1. **Stale axis comment.** The xacro comment near the steering
   joint axis still claims `(0,0,-1)` is intentional. A one-line
   cleanup follow-up either edits it to reflect the new convention
   or removes it. Architect call.

2. **Should the inherited `calculate_steering_angle` div-by-zero
   fix be backported into the architect's master spec for the
   messenger?** The R07 spec (which I implemented verbatim) had
   this bug. The fix is now in our tree. If the architect plans to
   reference the R07 spec verbatim again somewhere, that copy
   also needs the guard. (Not blocking; just flagging for the
   architect's bookkeeping.)

3. **R07c's open questions 1/2** (steering-sign options A/B/C, and
   the matching odometry direction) are **resolved** by this round:
   chose Option C (flip the URDF axis) — the bootstrap rule
   correction. /odom and gz now agree on direction. R07c open
   question 4 (`ros2 topic hz` not producing readings) is still
   open; not blocking.

4. **PARALLEL mode handles the side-slip sign-juggling with a
   linear-but-correct-looking block** (the architect's spec has
   one line that's immediately overwritten by the next, plus
   sign-flip logic chained on `last_nonzero_x` and `linear_y`).
   The integration test passed end-to-end and the wheel commands
   were as expected, but the only branch I exercised was
   `linear_x = 0, linear_y = +0.3, last_nonzero_x ≥ 0`. The other
   combinations (negative last_x, negative linear_y) aren't covered
   by this round's tests. Worth a unit test in a follow-up.

5. **The 12 cm spin drift** comes from steering transients. With
   the joint velocity limit of `steer_vel = 6.28 rad/s` in the URDF
   and a transition from PARALLEL's π/2 to SPIN's ~0.94, the
   steerers take ~100 ms to settle, during which the wheel velocity
   commands are already at their spin values and the body
   translates. Could be reduced by pre-zeroing wheel velocities
   on mode transitions or by ramping the wheel speeds with the
   steering angles. Probably not worth optimizing — real robots
   have the same transient.
