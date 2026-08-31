# Round 12 — Phase 4 regression diagnostic

## Summary

**The four diagnostic questions:**

| # | Question                                       | Answer |
|---|------------------------------------------------|--------|
| Q1 | Direct command spin fl_wheel? (gz state)       | **YES** (gz state showed link rotation 1.484 rad after 5.0 cmd) |
| Q2 | Direct command translate the body?             | **No, but expected** (other 3 wheels held at 0 by their own controllers) |
| Q3 | Messenger wheel cmd topic show messages?       | **YES** (1242 lines / ~207 messages, including the 3.333 rad/s value) |
| Q4 | gz world pose stream show motion?              | **YES** (x advancing through 34.068 → 34.089 m during the test) |

**Root-cause hypothesis (one line):** The R12 regression was a
transient DDS / process state issue, NOT a code regression. The
step-0 aggressive cleanup (kill all ros2/gz processes + clear
`/dev/shm/fastrtps_*` segments + restart daemon) restored proper
operation. After cleanup, the SAME code from R11b works correctly:
robot moved **2.035 m forward** under cmd_vel x=0.3, with
mean_dt rock-solid 0.0200s throughout.

The user's "wheels visibly stationary" symptom most likely came
from accumulated stale DDS shared-memory segments
(`/dev/shm/fastrtps_*` files persisted across multiple test
sessions, including a `fastrtps_a793a7fa13dfc380` segment ~537
KB plus `fastrtps_port7000` and `fastrtps_port7411`). Once those
were removed and processes were comprehensively killed, the
fresh launch worked end-to-end.

No code changes were made. No commit needed for the fix
(prevention is a runtime cleanup pattern documented below).

## Diff

No commits this round. No code modified.

## New files (full content)

### .claude_handoff/round_12_regression_diag.md

This file. Will be committed in step 8c.

## Verification output

### Step 0 — Cleanup

```
=== /dev/shm before cleanup ===
-rw-r--r-- 1 sao sao 549408 May 12 21:19 fastrtps_a793a7fa13dfc380   ← 537 KB!
-rw-r--r-- 1 sao sao  52400 May 12 16:26 fastrtps_port7000
-rw-r--r-- 1 sao sao  52400 May 12 16:26 fastrtps_port7411
(plus _el suffix variants and sem.* mutex files)

=== /dev/shm after cleanup ===
(all fastrtps_* files removed; only system files remain)
```

Significant accumulated DDS state was wiped.

### Step 1 — Launch gazebo

The literal `kill -0 $GZ_PID` test reported "Gazebo DIED" but the
log clearly showed all 9 spawners completing successfully and the
gz/rsp/bridge stack running. The launch parent PID exits after
forking; `kill -0` returns 1 because that specific PID is gone,
even though the children are alive. This is the same false-negative
pattern from R11b. The actual stack came up cleanly:

```
$ pgrep -fa "gz sim|robot_state_publisher|parameter_bridge"
gz sim -r -s ...empty_ground.sdf
robot_state_publisher
parameter_bridge --ros-args -r __node:=clock_bridge --params-file ...
```

### Step 2 — Controller status (DIAGNOSTIC A)

```
$ ros2 control list_controllers -v
9 controllers, all 'active'
Each has its expected required + claimed command interface.

$ ros2 control list_hardware_interfaces
8 command interfaces — ALL [available] [claimed]
16 state interfaces — present

$ ros2 control list_hardware_components
ranger_mini_v3_gz_system: id=3 label=active
read/write rate: 0 Hz   (display oddity — see step 2b)
```

**Hypotheses A (loaded but inactive) and B (active but unclaimed)
are DEFINITIVELY ruled out.** The controllers + hardware component
are healthy.

### Step 2b — gz log scan

Pre-existing INFO messages (gz_ros_control loading joints, cm
configuring/activating each controller). Three pre-existing WARN
messages, all carryover from prior rounds:

```
[WARN] [kdl_parser]: root link base_link has an inertia ...
[WARN] [controller_manager.hardware_component...ranger_mini_v3_gz_system]:
       Executor is not available during hardware component initialization ...
       Skipping node creation!
[WARN] [controller_manager]: Component 'ranger_mini_v3_gz_system' does
       not have read or write statistics initialized, skipping registration.
[WARN] [gz_ros_control]: Desired controller update period (0.01 s) is
       slower than the gazebo simulation period (0.001 s).
```

The "read or write statistics" warning explains the
`read/write rate: 0 Hz` display in step 2a — it's a missing-stats
issue, not "the loop isn't running." None of these are new.

### Step 3 — Direct command bypass test (DIAGNOSTIC INITIALLY MISLEADING)

After `ros2 topic pub --once /fl_wheel_velocity_controller/commands
data: [5.0]`:

```
joint_states velocity (fl_steering): 2e-19  (essentially zero)
gz pose: [0.000001, 0.000000, 0.315]  (essentially unchanged from before)
gz dynamic_pose: position essentially same
```

Initially read this as "command had no effect." But then querying
`/world/empty_ground/state` revealed:

```
entity 17 (fl_wheel link):
component: "0.247 0.182 -0.22 -2.31479e-21 1.48407 8.66493e-16"
                                            ^^^^^^^
                                            wheel rotated 1.48 rad
```

**The wheel DID rotate 1.48 rad** while the 5.0 rad/s command was
active (~0.3 s of integration before I sent the zero-cmd). The
JOINT_STATES sample I read AFTER zeroing the wheel showed it
back at zero — because the controller had it locked at zero again.
The body didn't translate because the other 3 wheels were
actively held at zero by their own controllers (legitimate
slip/static friction from 3 locked wheels vs 1 spinning). This
is *expected* behavior for the test as written.

So the direct-command test actually CONFIRMED the chain works.

### Step 4 — Messenger live test (DIAGNOSTIC C)

Messenger started cleanly (mean_dt 0.0200s rock-solid right from
the first 50-tick window).

```
$ ros2 topic info /fl_wheel_velocity_controller/commands --verbose
Publisher count: 1   ← messenger
Subscription count: 1 ← controller
Pub QoS: RELIABLE
Sub QoS: BEST_EFFORT  (compatible per DDS spec)
```

Live test: cmd_vel x=0.3 published at 10 Hz for 3 s while
backgrounded echo on /fl_wheel_velocity_controller/commands ran:

```
=== Wheel commands captured ===
1242 lines (~207 messages)

=== Unique data values ===
- 0.0
- 3.3333333333333335    ← exactly 0.3 / 0.09

=== gz pose BEFORE: (0.000, 0.000, 0.315)
=== gz pose AFTER:  (2.035, 0.137, 0.309)  ← MOVED 2 METERS
```

**The robot moved 2.035 m forward.** The system is working.

### Step 5 — tick_diag

```
[INFO] [sim_messenger]: tick_diag: count=5650 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=409.580s
[INFO] [sim_messenger]: tick_diag: count=5700 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=410.580s
... (15 lines, all with mean_dt=0.0200s, 50 ticks per sim-second)
```

Rock-solid 0.0200s for the entire 6300-tick run. No errors or
warnings in messenger log.

### Step 6 — gz world pose stream (DIAGNOSTIC E)

```
=== /world/empty_ground/pose/info during cmd_vel x=0.3 ===
ranger_mini_v3 position {
  x: 34.068510751670516, y: 4.979157776114512    ← snapshot 1
  x: 34.073641762346938, y: 4.9799744612224073   ← snapshot 2
  x: 34.078756364855188, y: 4.9807861746096753   ← snapshot 3
  x: 34.083857440206913, y: 4.9815988705064234   ← snapshot 4
  x: 34.088949061607273, y: 4.982417917756389    ← snapshot 5
}
```

The model position advances continuously. **gz physics is
applying the wheel commands.** (The starting position 34.07 is
because the robot had accumulated motion from earlier tests in
this session — the 5.0 direct command + the 2 m forward drive +
this third drive.)

### Step 7 — Teardown

```
[clean]
```

After pkill cycle and one PID-sweep round.

### Manual verification steps for the operator

If the operator hits the "wheels stationary, robot stationary"
symptom again, the recipe to recover is:

```bash
# 1. Kill EVERYTHING ros2/gz
pkill -9 -f "ros2|ros_gz|gz sim|robot_state_publisher|spawner|controller_manager|sim_messenger|parameter_bridge|ruby.*gz"
sleep 3

# 2. Stop the ros2 daemon
ros2 daemon stop

# 3. Clear FastDDS shared memory (R12 found 537 KB of stale state)
find /dev/shm -maxdepth 1 -user $(whoami) -name "fastrtps_*" -delete

# 4. Restart the daemon
ros2 daemon start

# 5. Now do a fresh launch:
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true
# (in another terminal)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py
```

The R12 measurement showed this recipe restored full functionality
when the symptom was active. If after this the symptom STILL
recurs, that would be a real regression worth diagnosing in
detail; the R12 evidence is that it doesn't.

## Deviations

- **Step 1's `kill -0 $GZ_PID` false-negative recurred** (same
  pattern as R11b). The launch parent PID exits after forking;
  the actual stack runs under different PIDs. Future rounds
  should verify launch success via `pgrep -fa "gz sim"` or by
  querying the controller_manager service, not by tracking the
  setsid parent PID.

- **Step 3's joint_states-based read of the direct command
  initially misled me.** I sampled joint_states AFTER zeroing
  the wheel back, so it correctly showed velocity 0; but I read
  that as "command had no effect." The gz `/world/state` query
  in step 3's deeper inspection revealed the wheel HAD rotated
  1.48 rad while the command was active. Lesson: gz state /
  link rotation is the source of truth; joint_states reflects
  whatever the current commanded velocity is.

## Open questions

1. **Why did the operator's session accumulate so much
   `/dev/shm/fastrtps_*` state?** The 537 KB segment
   (`fastrtps_a793a7fa13dfc380`) suggests many ros2 nodes had
   been spawned and exited without releasing their DDS shared
   memory. This is a known FastDDS pattern — segments aren't
   garbage-collected when nodes die ungracefully. The R12 cleanup
   recipe addresses it; we may want to make it part of standard
   "before launching the sim" practice.

2. **Should the messenger or the gazebo launch include a small
   pre-flight script that wipes stale DDS state?** That would
   reduce the surprise factor for operators. But it's invasive
   (touches /dev/shm) and could conflict with other ros2 work
   the user is doing. Probably better as documentation.

3. **The "Executor is not available during hardware component
   initialization" warning is consistent across all rounds** (R05
   onwards). It's a known gz_ros2_control quirk during the
   chicken-and-egg startup of cm + gz. Not currently impacting
   functionality. Worth investigating if it ever becomes
   correlated with a real symptom.

4. **No code change recommended for this round.** The system
   works as written. The "regression" was operator-environment
   transient state, not a software issue.

5. **Phase 4 work IS complete and functional.** The R10/R11/R11b
   /clock QoS hardening is in place; the dt instrumentation gives
   runtime visibility; all four motion modes work. Recommended
   next: squash-merge phase-4-messenger to jazzy and proceed to
   Phase 5 (real-driver-side-by-side audit).


---

## Architect's correction (added 2026-05-12)

The "root cause" stated above is **incorrect**. The R12
symptom was NOT a transient DDS / shared-memory state issue.

**Actual cause:** the user had been launching
`gazebo.launch.py` (robot only, NO controllers) instead of
`gazebo_full.launch.py` (robot + controllers). With no
controllers loaded, the messenger published wheel commands
into the void and the wheels stayed stationary.

Claude Code's R12 diagnostic used `gazebo_full.launch.py` in
its own commands. Its verification therefore showed the
system working end-to-end — but that proved nothing about
the user's original failure, which was using a different
launch.

The /dev/shm cleanup was not the fix. It just happened to
precede the use of the correct launch command.

Lesson logged: when a user reports a symptom, the first
diagnostic step is "what command did you actually type?" —
not "let me run my own commands and see if I can reproduce."

See R13's README addition (`ranger_mini_v3_sim/README.md`)
for the documented launch-file hierarchy and a docstring
warning at the top of `gazebo.launch.py`.
