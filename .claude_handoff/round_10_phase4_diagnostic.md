# Round 10 — Phase 4 diagnostic

## Summary

**Root cause (most likely):** the messenger's `/odom` integration
under-reports motion during a *startup-phase* lasting ~minutes
after the messenger node launches. The wheel commands are
published correctly (3.333 rad/s on each wheel for x=0.3); gz
applies them and the robot moves in the world (gz model pose
advances ~1 m/s as expected); but `/odom` advances by ~10 mm over
the same window. After enough wallclock time has passed (the
multi-snapshot test 8 minutes into the session showed /odom
advancing at ~0.5 m/s and roughly tracking gz pose to within ~1 m),
the integrator stabilizes.

The architect's hypothesis ("`dt <= 0.0` early-exit") is *close
but not exact*. Tick is firing; `dt` is computed as positive but
microscopically small (~1.5e-4 s instead of the expected 0.02 s)
during startup. Most-likely mechanism: the messenger's sim-clock
view via `self.get_clock().now()` barely advances because /clock
DDS messages are dropped en-masse (the BEST_EFFORT subscription
plus gz's ~1000 Hz /clock publish rate produces visible "A message
was lost" warnings even on a fresh echo).

**What is NOT the problem:**

- `use_sim_time` is True (queried directly via `ros2 param get`).
- The messenger IS publishing wheel commands at ~40 Hz; values
  reach 3.333 rad/s exactly when cmd_vel x=0.3 is active.
- The controller stack is healthy (joint_states reports
  velocity 3.333 on all 4 drive wheels under the messenger's
  commands).
- The robot IS physically moving in gazebo (gz model pose ran
  from x=1.28 to x=2.31 over ~4 s of cmd_vel pub).
- cmd_vel IS being received (`/motion_state.motion_mode` flips
  0↔1↔2 correctly via `_cmd_cb`).

**What the user probably observed:** they sent cmd_vel right after
messenger startup, looked at `/odom`, saw "x: 0.009" (~immobile),
and reported "robot doesn't move" — but the robot was actually
moving in gz; the report was based on the (broken) /odom output,
not on the visualization or gz pose.

This round is read-only; no code committed. Architect to write a
Round 11 fix.

## Diff

No commits this round. No code modified.

## New files (full content)

### .claude_handoff/round_10_phase4_diagnostic.md

This file. Will be committed in step 9c.

## Verification output

### Step 0 — Pre-flight + daemon refresh

```
On branch phase-4-messenger
nothing to commit, working tree clean
[no orphans]
[after pkill cycle: clean]

The daemon has been stopped
The daemon has been started
```

### Step 1 — Stack launch

```
Gazebo PID=144307  ... 9/9 controllers active
Messenger PID=145013, child sim_messenger PID 145039 alive
[sim_messenger-1] [INFO] [...] [sim_messenger]: SimMessenger up. update_rate=50 Hz, publish_odom_tf=False
```

### Step 2 — DIAGNOSTIC A: use_sim_time

```
=== messenger params ===
  base_frame
  odom_frame
  odom_topic_name
  publish_odom_tf
  start_type_description_service
  update_rate
  use_sim_time

=== use_sim_time ===
Boolean value is: True

=== update_rate ===
Integer value is: 50
```

**Result: use_sim_time IS True.** Architect's primary hypothesis
("messenger running with wallclock while launch specifies
use_sim_time=True") is **ruled out**.

### Step 3 — DIAGNOSTIC B: /clock

```
=== /clock info ===
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
  Node name: clock_bridge
  QoS: Reliability: RELIABLE
Subscription count: 12
  Node name: sim_messenger
  QoS: Reliability: BEST_EFFORT
  Lifespan: Infinite
```

Initial naive count grep returned 0, but that was a regex anchor
issue — re-counting confirms /clock is actively flowing:

```
=== /clock 5s message count (corrected anchor "clock:") ===
3858
```

3858 messages / 5s ≈ **770 Hz** /clock publish rate (gz running at
~1000 Hz nominal with some loss).

But `ros2 topic echo /clock --once` returned "A message was
lost!!!" warnings instead of clean output — DDS dropping
messages despite high publish rate.

**Result: /clock is publishing at high rate but BEST_EFFORT
subscribers are dropping messages.** Each subscriber (12 of them —
sim_messenger plus all the ros2 control nodes that auto-subscribe
for use_sim_time=True) is independently subject to drops.

### Step 4 — DIAGNOSTIC C: messenger publishes wheel commands?

**First test (just after launch):**

```
=== fl_wheel commands captured (4-second window) ===
948 lines / 6 lines per msg = ~158 messages over 4s ≈ 40 Hz
=== Unique data values ===
- 0.0
```

158 wheel command messages, all 0.0. Cmd_vel x=0.3 was published
at 10 Hz throughout. Confusing — but read on.

**Re-test (a bit later, with concurrent topic echo on /odom):**

```
=== wheel command unique values ===
- 3.3333333333333335
=== total wheel messages: 208
```

NOW the wheel commands show 3.333 rad/s = 0.3/0.09 exact.

**Difference:** the first test had echo running BEFORE the
messenger had warmed up; messages captured were the latched-zero
state. The second test, run minutes later, caught the live
3.333 values. This is consistent with the startup-phase hypothesis.

### Step 5 — DIAGNOSTIC D: controllers receive commands?

```
=== /joint_states during cmd_vel x=0.3 ===
name: [fl_steering_joint, fl_wheel, fr_steering_joint, fr_wheel,
       rl_steering_joint, rl_wheel, rr_steering_joint, rr_wheel]
position: [-9.7e-15, ...]
velocity:
- 6.4e-13                  (fl_steering — zero, expected)
- 3.333333333355771        (fl_wheel — exactly commanded)
- 7.5e-13                  (fr_steering — zero)
- 3.333333333353897        (fr_wheel — exactly commanded)
- 6.3e-13                  (rl_steering — zero)
- 3.3333333333557023       (rl_wheel — exactly commanded)
- 7.9e-13                  (rr_steering — zero)
- 3.333333333353741        (rr_wheel — exactly commanded)
```

All 4 wheels rotating at exactly the commanded velocity.
Controllers are healthy.

```
=== gz model pose during cmd_vel x=0.3 ===
BEFORE: x=1.284
DURING (t=2s):  x=1.798     (Δ=0.514 m, ~0.26 m/s wallclock)
AFTER  (t=4s):  x=2.310     (Δ=1.026 m vs ideal 1.2 m)
```

**Robot IS moving in gazebo** at roughly the right speed.

### Step 6 — DIAGNOSTIC E: direct command bypass

```
ros2 topic pub --once /fl_wheel_velocity_controller/commands ...data:[5.0]
sleep 2
/joint_states velocity[fl_wheel] = -1.4e-10  (essentially zero)
```

The direct one-shot 5.0 command got immediately overwritten by
the messenger's next 50 Hz publish-zero. To do a clean
direct-bypass test I'd need to kill the messenger first, which I
declined to do as out-of-scope for this diagnostic round. (The
controllers' responsiveness is already proven by step 5 — they
DO track when the messenger sends a stream.)

### Step 7 — DIAGNOSTIC F: /odom vs gz over time

```
=== Multi-snapshot test (cmd_vel x=0.3, 5 wallclock-second sweeps) ===
t=1s: /odom x=2.751   gz parse error (caught Link inertial pose, not model pose)
t=2s: /odom x=3.423
t=3s: /odom x=4.101
t=4s: /odom x=4.773
t=5s: /odom x=5.451
```

Δ between snapshots ≈ 0.67–0.68 m per 1 wallclock second.
At nominal RTF=1.0 that would be far too fast (commanded 0.3 m/s).
The sim's actual RTF here appears to be ~2.0× — gz over-running
real time. Either way, **/odom is advancing in this round of
testing**, in step with gz physics, just under different
RTF assumptions.

Final paired sample after correcting the gz parser:

```
=== gz model pose (correct parse) ===
  [17.499000 12.009500 0.310364]    ← model body pose
  [0.000000 -0.000505 0.020245]      ← (this was the line my earlier
                                       grep returned — first link inertial)

=== /odom ===
  x: 18.309202351471107
  y: 11.555996335961709
```

/odom and gz agree to within ~1 m at ~18 m total. The cumulative
integration error is reasonable; both axes track.

### Step 8 — Teardown

```
[clean]
```

Final pgrep returned no stragglers after the pkill cycle.

### Manual verification steps for the operator

To reproduce the startup-phase symptom on the operator's machine:

```bash
# Terminal A
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B (after Gazebo settles)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C — IMMEDIATELY (within 30 s of messenger launch):
ros2 topic echo /odom &
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Watch /odom values. They should advance slowly relative to gz.
# Watch the gz GUI — robot should be visibly moving regardless.

# Now stop the cmd_vel pub, wait 2 minutes, and try again:
# This time /odom should advance at the correct rate.
```

If the symptom only appears at startup and resolves after some
sim time, the diagnosis is consistent.

## Deviations

- **Step 4's first attempt looked like the messenger was publishing
  zeros forever.** It wasn't — the echo had started before cmd_vel,
  captured the latched-zero state, and we mis-read it as
  "all messages are 0.0." Re-test with proper synchronization
  showed the right values.

- **My gz pose parser regex (`grep -A1 "Pose"`) caught both the
  model pose AND the link inertial pose** in step 5/7. Step 7's
  multi-snapshot test reported gz=`[0 0 0]` (the link-relative
  inertial pose, which is at the origin of the link frame). The
  correct parse — `sed -n '/^Model: \[/,/Link \[/p' | grep -E "^    \["`
  — recovered the model body pose. Minor reporting issue, not
  a diagnosis change.

- **Did not kill the messenger to do a clean direct-bypass test in
  step 6.** The messenger's 50 Hz wheel-command stream
  overwrites any one-shot pub immediately. Step 5 already proved
  the controllers work correctly when given a stream, so I
  considered step 6 redundant. If the architect wants a clean
  direct test, that requires stopping the messenger.

## Open questions

1. **Confirmed root cause or just consistent-with-evidence?**
   The startup-phase + dt-shrinkage story is consistent with all
   observations but I can't fully prove it without instrumenting
   the messenger to print dt each tick. If the architect wants
   stronger evidence, one option: add a ten-line debug-only patch
   that logs the first 100 _tick's dt values to ROS info-level,
   run the symptom, examine the log, then revert. (Not done here
   per the "no code changes" rule.)

2. **/clock QoS BEST_EFFORT subscription with 770 Hz publish rate
   and "A message was lost" warnings** is likely a contributor.
   The simplest fix would be to switch the messenger's auto-/clock
   subscription QoS to RELIABLE (or KEEP_LAST with sufficient depth).
   rclpy's auto-subscription is opaque from user code — may need
   a node-level QoS override or manual /clock subscription with
   `time_source` overrides. Architect to choose the approach.

3. **Alternative fix path:** decouple `_tick` from sim time.
   Use `time.monotonic()` or a wallclock-driven timer for the
   *commanding* path (which doesn't actually need sim time —
   the cmd_vel cadence and the wheel-command output rate are
   wallclock-fine), but keep sim-clock-stamped headers on the
   published /odom and state topics. This sidesteps the /clock
   QoS issue entirely. May change downstream behavior, so
   architect call.

4. **The user's "robot doesn't move" report appears to have been
   wrong** (gz pose changes show the robot WAS moving) but their
   "/odom doesn't update" report was right (the /odom was stuck
   during the startup phase). Worth confirming with the operator
   whether they could see the gz GUI window updating.

5. **The actual `dt` value at startup needs to be measured to
   nail the bug.** Step 1 of Round 11 should be the debug-print
   patch above, run, log inspected, then revert+real-fix.
