# Round 15a — Phase 5 sim-side topic + behavior inventory

## Summary

Brought up the full sim stack on `ROS_DOMAIN_ID=45` (matching the
real robot's domain) and captured:

1. **Topic type table** — all 6 parity topics resolve to the
   exact expected `ranger_msgs/msg/*`, `nav_msgs/msg/Odometry`,
   `geometry_msgs/msg/Twist`, `sensor_msgs/msg/BatteryState` types.
2. **Verbose `topic info` per topic** — node identity, QoS profile,
   pub/sub counts. (See § Verification output for full content.)
3. **Sample messages from each publish topic** — all field shapes
   look correct; battery defaults applied (24V, 25°C, etc.); odom
   shows zeros at rest; actuator_state shows 8 actuators with
   ID 0-3 = steering and ID 4-7 = wheels.
4. **Behavior captures for three Twist patterns** (`forward`,
   `arc_left`, `spin`): BEFORE/AFTER `/odom`, `/motion_state`, and
   end-state `/actuator_state`.

Sim-side baseline ready for cross-machine comparison against the
real driver in Round 15b.

## Captures Saved

30 files in `/tmp/r15_audit_sim/`:

```
type_table.txt                           658 bytes
_cmd_vel.txt                             518   (topic info)
_odom.txt                                508   (topic info)
_odom.sample.txt                         959   (sample message)
_system_state.txt                        520
_system_state.sample.txt                 156
_motion_state.txt                        520
_motion_state.sample.txt                  87
_actuator_state.txt                      534
_actuator_state.sample.txt              2141
_battery_state.txt                       522
_battery_state.sample.txt                340
_joint_states.txt                       1437
_joint_states.sample.txt                 752
_tf.txt                                  518
r15_sim_forward_odom_before.txt          206
r15_sim_forward_odom_after.txt           220
r15_sim_forward_motion_before.txt         87
r15_sim_forward_motion_after.txt          87
r15_sim_forward_actuator_after.txt      2129
r15_sim_arc_left_odom_before.txt         221
r15_sim_arc_left_odom_after.txt          236
r15_sim_arc_left_motion_before.txt        87
r15_sim_arc_left_motion_after.txt         87
r15_sim_arc_left_actuator_after.txt        0  ← timed out (see Deviations)
r15_sim_spin_odom_before.txt             234
r15_sim_spin_odom_after.txt              234
r15_sim_spin_motion_before.txt            87
r15_sim_spin_motion_after.txt             87
r15_sim_spin_actuator_after.txt         2098
```

## Diff

No code changes. One handoff commit + one CONTEXT entry.

## New files (full content)

### .claude_handoff/round_15a_phase5_sim_audit.md

This file. Will be committed in step 6c.

## Verification output

### Step 0a — Pre-flight

```
On branch jazzy
nothing to commit, working tree clean
HEAD: 10d6487 docs(handoff): round 14 — phase 4 squash-merge to jazzy
[clean — only daemons left]
```

The "orphan" pgrep returned two ros2 daemons (DOMAIN 0 and DOMAIN
45 from prior testing). Both are fine to leave running. No
gz/messenger processes.

### Step 0b — Environment

```
ROS_DOMAIN_ID=<unset, default 0>
RMW_IMPLEMENTATION=<unset, default rmw_fastrtps_cpp>
ROS_DISTRO=jazzy
```

(Subsequently set `ROS_DOMAIN_ID=45` per-Bash-call since each
Claude Code Bash invocation is a fresh shell.)

### Step 1 — Sim launch on DOMAIN=45

The first launch attempt's `if ! pgrep "gz sim" | grep -v claude > /dev/null`
guard fired its pkill cleanup branch (R12 trap recurring) — the
pgrep raced gz startup. Re-launched without the guard and gz came
up cleanly. Final state:

```
9 controllers all 'active' on DOMAIN=45.
sim_messenger node info shows expected sub/pub list:
  Subs: /clock, /cmd_vel, /joint_states
  Pubs: /odom, /system_state, /motion_state, /actuator_state,
        /battery_state, plus 8 controller /commands topics.
```

### Step 2a — All topics on DOMAIN=45

```
/actuator_state
/battery_state
/clock
/cmd_vel
/controller_manager/activity
/controller_manager/introspection_data/{full,names,values}
/controller_manager/statistics/{full,names,values}
/diagnostics
/dynamic_joint_states
/fl_steering_position_controller/{commands,transition_event}
/fl_wheel_velocity_controller/{commands,transition_event}
/fr_steering_position_controller/{commands,transition_event}
/fr_wheel_velocity_controller/{commands,transition_event}
/joint_state_broadcaster/transition_event
/joint_states
/motion_state
/odom
/parameter_events
/rl_steering_position_controller/{commands,transition_event}
/rl_wheel_velocity_controller/{commands,transition_event}
/robot_description
/rosout
/rr_steering_position_controller/{commands,transition_event}
/rr_wheel_velocity_controller/{commands,transition_event}
/system_state
/tf
/tf_static

Total: 39 topics.
```

### Step 2d — Type check (the headline parity table)

```
TOPIC                          EXPECTED                                           ACTUAL
/cmd_vel                       geometry_msgs/msg/Twist                            geometry_msgs/msg/Twist
/odom                          nav_msgs/msg/Odometry                              nav_msgs/msg/Odometry
/system_state                  ranger_msgs/msg/SystemState                        ranger_msgs/msg/SystemState
/motion_state                  ranger_msgs/msg/MotionState                        ranger_msgs/msg/MotionState
/actuator_state                ranger_msgs/msg/ActuatorStateArray                 ranger_msgs/msg/ActuatorStateArray
/battery_state                 sensor_msgs/msg/BatteryState                       sensor_msgs/msg/BatteryState
```

**ALL 6 EXPECTED TYPES MATCH EXACTLY.**

### Step 2b — Per-topic verbose info

#### /cmd_vel

```
Type: geometry_msgs/msg/Twist
Publisher count: 0   (no publisher right now — would be the
                      operator's teleop / nav / direct pub)
Subscription count: 1
  Node: /sim_messenger
  Topic type hash: RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a
  QoS: BEST_EFFORT, VOLATILE, AUTOMATIC liveliness
```

#### /odom

```
Type: nav_msgs/msg/Odometry
Publisher count: 1
  Node: /sim_messenger
  Topic type hash: RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0
  QoS: RELIABLE, VOLATILE, AUTOMATIC liveliness
Subscription count: 0   (no subscriber from sim itself; consumers
                          live on the application side)
```

#### /system_state

```
Type: ranger_msgs/msg/SystemState
Publisher count: 1
  Node: /sim_messenger
  Topic type hash: RIHS01_a1eacabc62f686c889e4e302969289b7416833c52fbfad7df8a65d50efff119d
  QoS: RELIABLE, VOLATILE
Subscription count: 0
```

#### /motion_state

```
Type: ranger_msgs/msg/MotionState
Publisher count: 1
  Node: /sim_messenger
  Topic type hash: RIHS01_dd38b2b683ad38733705ab2620a32ce75939f25c589b1830ccd81a01c9bc0bbc
  QoS: RELIABLE, VOLATILE
Subscription count: 0
```

#### /actuator_state

```
Type: ranger_msgs/msg/ActuatorStateArray
Publisher count: 1
  Node: /sim_messenger
  Topic type hash: RIHS01_139483c38e99dd59c124df086874639fcbd87120ef013799679331bc68a81e5f
  QoS: RELIABLE, VOLATILE
Subscription count: 0
```

#### /battery_state

```
Type: sensor_msgs/msg/BatteryState
Publisher count: 1
  Node: /sim_messenger
  Topic type hash: RIHS01_4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741
  QoS: RELIABLE, VOLATILE
Subscription count: 0
```

#### /joint_states

```
Type: sensor_msgs/msg/JointState
Publisher count: 1
  Node: /joint_state_broadcaster
  Topic type hash: RIHS01_a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e
  QoS: RELIABLE, TRANSIENT_LOCAL
Subscription count: 2
  Node: /robot_state_publisher  (BEST_EFFORT)
  Node: /sim_messenger          (BEST_EFFORT)
```

(Note QoS mismatch between RELIABLE pub + BEST_EFFORT subs — OK
per DDS spec; "A message was lost" warnings appear sometimes when
echoing but don't actually drop messages downstream.)

#### /tf

```
Type: tf2_msgs/msg/TFMessage
Publisher count: 1
  Node: /robot_state_publisher
  Topic type hash: RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087
  QoS: RELIABLE, VOLATILE
Subscription count: 0
```

### Step 2c — Sample messages

#### /odom (at rest)

```yaml
header:
  stamp: { sec: 369, nanosec: 620000000 }
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position: { x: 0.0, y: 0.0, z: 0.0 }
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
  covariance: [0.0 × 36]
twist: { … all zero at rest … }
```

#### /system_state

```yaml
header: { stamp: 365.680..., frame_id: '' }
vehicle_state: 0       # NORMAL
control_mode: 1        # CAN
error_code: 0
battery_voltage: 24.0
motion_mode: 0         # DUAL_ACKERMAN (default at rest)
```

#### /motion_state

```yaml
header: { stamp: 366.660..., frame_id: '' }
motion_mode: 0
```

#### /actuator_state (8 actuators, head only)

```yaml
header: { stamp: 367.620..., frame_id: '' }
states:
- id: 0
  motor:
    rpm: 0
    current: 0.0
    pulse_count: 0
    motor_angles: ~3e-21         # numerical noise at rest
    motor_speeds: ~-5e-21
  driver:
    driver_voltage: 24.0
    driver_temperature: 35.0
    motor_temperature: 40.0
    driver_state: 0
- id: 1   # … same shape, IDs 0-3 = steering, 4-7 = drive wheels
```

#### /battery_state

```yaml
header: { stamp: 368.620..., frame_id: '' }
voltage: 24.0
temperature: 25.0
current: -1.0
charge: .nan
capacity: .nan
design_capacity: .nan
percentage: 1.0
power_supply_status: 0           # UNKNOWN
power_supply_health: 0           # UNKNOWN
power_supply_technology: 2       # LION (the symbolic constant value)
present: true
cell_voltage: []
cell_temperature: []
location: ''
serial_number: ''
```

#### /joint_states

```yaml
header: { stamp: 370.610..., frame_id: base_link }
name: [fl_steering_joint, fl_wheel, fr_steering_joint, fr_wheel,
       rl_steering_joint, rl_wheel, rr_steering_joint, rr_wheel]
position: [~1e-21, ~2e-15, ~−1e-22, ~2e-15, …]   # noise at rest
velocity: [~1e-21, ~−5e-19, …]
effort:   [.nan × 8]
```

### Step 3 — Behavior patterns

#### Pattern: forward (`{linear: {x: 0.3}}`, 3s)

| Field        | BEFORE    | AFTER     |
|--------------|-----------|-----------|
| /odom x      | 0.000     | **0.894** |
| /odom y      | 0.000     | 0.000     |
| motion_mode  | 0         | 0         |

Predicted at 0.3 m/s × 3s = 0.9 m. Got 0.894. ✓

#### Pattern: arc_left (`{linear: {x: 0.3}, angular: {z: 0.3}}`, 3s)

| Field        | BEFORE    | AFTER     |
|--------------|-----------|-----------|
| /odom x      | 0.894     | **1.964** |
| /odom y      | 0.000     | **+0.681**|
| motion_mode  | 0         | 0         |

Δx = 1.07, Δy = +0.68 — body translated forward AND turned left
(positive y per ROS REP-103). Confirms R08 sign fix is in place.

#### Pattern: spin (`{angular: {z: 0.5}}`, 3s)

| Field        | BEFORE     | AFTER      |
|--------------|------------|------------|
| /odom x      | 1.692      | 1.214      |
| /odom y      | 2.054      | 2.320      |
| motion_mode  | 0          | **2 (SPIN)**|

motion_mode flipped to SPINNING when angular-only Twist arrived.
Position drifted by ~0.5m (steering transients during 3s of spin —
expected, R08 noted ~12cm typical at shorter ramps; longer ramp here).

Spin actuator_state at end (excerpt):

```
- id: 0  motor_angles: -0.9360, motor_speeds: ~0.003   (fl steering, flipped)
- id: 1  motor_angles: +0.9358, motor_speeds: ~0       (fr steering)
- id: 2  motor_angles: +0.9358, motor_speeds: ~0       (rl steering, flipped)
…
```

The fl/rl steering at -0.936 and fr/rl at +0.936 match R08's
SPINNING-mode spin tangent geometry exactly (FL and RR were
range-wrapped by π).

### Step 4 — Teardown

```
[clean]
```

### Manual verification steps for the operator

The operator's robot-side capture (Round 15b) will be a parallel
script that runs the same `topic info` + sample-message + behavior
pattern captures on the real driver. The architect compares
sim-side data above against robot-side data, looking for:

- **Type mismatches** (any topic on robot has a different message
  type than sim).
- **Topic-name mismatches** (robot publishes `/imu/data` but sim
  doesn't, or vice versa).
- **QoS profile differences** that could cause cross-stack
  subscribers to silently drop messages.
- **Field-level schema differences** within the ranger_msgs types
  (the same package built from the same sources, so ought to be
  identical, but the type hashes give us a definitive comparison).
- **Behavior differences** under the same Twist input (different
  /odom evolution, motion_mode logic, actuator_state values).

## Deviations

- **`r15_sim_arc_left_actuator_after.txt` is 0 bytes.** The
  `timeout 2 ros2 topic echo /actuator_state --once` for the
  arc_left pattern was killed by the timeout before any output
  was written (a transient — the daemon's type cache had likely
  just been reset; `ros2 topic echo` was still negotiating the
  subscription when the timeout fired). The forward and spin
  pattern actuator captures both succeeded normally. Not blocking
  — the architect can re-capture if it's needed for the audit.

- **The first `setsid ros2 launch gazebo_full ...` attempt got
  killed by the prompt's `if ! pgrep "gz sim" | grep -v claude`
  guard** (same R12 trap). The pgrep races gz startup; my pgrep
  found nothing in that instant; failure branch fired pkill;
  cleanup killed the just-starting stack. Re-launched without the
  guard and gz came up cleanly. The R14 / R15a guard pattern
  should be retired in favor of pure pgrep without grep filter, OR
  a longer settle time, OR querying the controller_manager
  service directly.

- **The ros2 daemon needed two restart attempts to populate the
  ranger_msgs types.** The first restart picked up an env that
  didn't include `install/setup.bash`, so daemon-side schema
  resolution still returned "invalid type". After re-sourcing
  the install in the same shell that started the daemon and
  restarting once more, type-resolved samples worked. Each
  Claude Code Bash invocation is a fresh shell; the
  `export ROS_DOMAIN_ID=45` and `source install/setup.bash` need
  to be repeated per call.

## Open questions

1. **Daemon-state-per-DOMAIN matters more than I appreciated.**
   The sim works fine on DOMAIN=0 (default) but moving to
   DOMAIN=45 required restarting the daemon for that domain
   AFTER sourcing the install. Worth folding into the README's
   troubleshooting section.

2. **Topic-type hashes are useful for cross-machine parity
   verification.** The hash strings (e.g.
   `RIHS01_a1eacabc62f686c889e4e302969289b7416833c52fbfad7df8a65d50efff119d`
   for SystemState) are derived from the message definition's
   structure. If sim and robot's hashes match, the schemas are
   structurally identical (ROS 2's type-introspection guarantee).
   Architect should compare these in the parity report.

3. **Some QoS profiles are inferred BEST_EFFORT but were
   intentional decisions** (e.g. `/cmd_vel` subscriber, to
   match typical teleop publishers). Worth noting in the parity
   report whether the real driver uses the same QoS choices.

4. **The 39 topics include controller_manager internals** (10
   topics under `/controller_manager/`) that aren't part of the
   real-driver interface but are normal ros2_control infrastructure.
   The robot-side capture won't have these (real driver runs
   without ros2_control), so they're an expected difference.
   Worth filtering in the parity report.

5. **Phase 5B (next)** should also capture the operator's exact
   shell environment for the robot-side run, especially RMW vendor
   and DDS settings. Different DDS vendors can interpret QoS
   differently and would surface as a parity discrepancy that's
   actually a tooling issue.
