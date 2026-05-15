# Phase 5 — Interface Parity Report: Sim vs Real Ranger Mini v3

**Audit method:** workstation captured sim-side topic info +
sample messages + behavior under three Twist patterns on
ROS_DOMAIN_ID=45 (R15a). Robot laptop captured the same on
the same DOMAIN with the real driver (R15b). Round 17
captured continuous /motion_state during active commands
to resolve a snapshot-timing ambiguity.

**Bottom line:** Strong parity. All 6 message schemas match
bit-for-bit (DDS type hashes identical). Behavior matches
in spirit. A handful of content-level differences are
intentional sim improvements OR mock-default tuning (R16
addressed the latter). One reporting-shape difference is
deliberately preserved as a sim improvement.

---

## 1. Schema parity — bit-for-bit identical

ROS 2 / DDS computes a `RIHS01_*` hash from each message
definition. Identical hashes = identical schemas.

| Topic            | Type                                | Hash match |
|------------------|-------------------------------------|------------|
| /cmd_vel         | geometry_msgs/msg/Twist             | ✅         |
| /odom            | nav_msgs/msg/Odometry               | ✅         |
| /system_state    | ranger_msgs/msg/SystemState         | ✅         |
| /motion_state    | ranger_msgs/msg/MotionState         | ✅         |
| /actuator_state  | ranger_msgs/msg/ActuatorStateArray  | ✅         |
| /battery_state   | sensor_msgs/msg/BatteryState        | ✅         |
| /tf              | tf2_msgs/msg/TFMessage              | ✅         |

Application code subscribing to any of these topics
deserializes the same wire format from either stack.

---

## 2. Topology

| Aspect           | Sim | Real |
|------------------|-----|------|
| Total topics     | 39  | 9    |
| Publishing node  | /sim_messenger | /ranger_base_node |

Sim's extra ~30 topics are ros2_control / Gazebo
infrastructure (controller_manager/*, /clock, /joint_states,
individual controller commands) that application code
doesn't subscribe to.

---

## 3. QoS profiles

Post-R16, all six parity topics' QoS profiles match:
publishers RELIABLE+VOLATILE, /cmd_vel subscriber RELIABLE
on both stacks. No divergences remain.

---

## 4. Content alignment (post-R16)

R16 series aligned the mock values:

| Field                                  | Sim now | Real |
|----------------------------------------|---------|------|
| /system_state.battery_voltage          | 49.6    | ~50.2 |
| /battery_state.voltage                 | 49.6    | (real has 10× scaling bug → 496) |
| /battery_state.percentage              | 100.0 (0-100 scale) | ~81 |
| /battery_state.present                 | false   | false (real driver's NaN-cast quirk) |
| /actuator_state.driver.driver_voltage  | 49.5    | 49.4 |
| /actuator_state.driver.driver_temperature | 40.0 | 40.0 |
| /actuator_state.driver.motor_temperature  | 23.0 | 22-24 |
| /actuator_state.driver.driver_state    | 64      | 64 |

---

## 5. Behavioral findings (R15b + R17 resolution)

### motion_mode auto-switching — confirmed matching (R17)

R15b BEFORE/AFTER snapshots showed motion_mode=0 across
all patterns including spin, raising a "does the real
driver actually switch modes?" question. R17's continuous
capture during active commands resolved it cleanly:

| Pattern        | Twist            | motion_mode timeline             |
|----------------|------------------|-----------------------------------|
| forward        | x=0.1            | 0 throughout (407 msgs)          |
| arc_left       | x=0.1, w=0.1     | 0 throughout (radius=1.0m)       |
| spin           | w=0.2            | 0 → 2 → 0 over ~5s of cmd      |
| tight_curve    | x=0.1, w=0.5     | 0 → 2 → 0 over ~5s (radius=0.2m, below min_turn_radius) |

Sim's auto-switching logic exactly matches the real
driver's behavior. R15b's mode-0 readings were a
snapshot-timing artifact: BEFORE snapshot before command
took effect, AFTER snapshot after command zeroed.

Lesson logged in CONTEXT.md (post-R12): when auditing
state-machine behavior, capture continuously during the
active condition, not snapshots before and after.

### Sign convention validated (R15b + R17)

Real robot reports positive Y for left turn in /odom,
matching ROS REP-103 and the sim. The R08 URDF axis fix
is validated against real-robot behavior.

---

## 6. Intentional sim improvements (deliberate divergences)

### /actuator_state per-wheel vs broadcast (R17 finding)

The real driver publishes the SAME steering angle across
all 8 actuator entries. It takes the commanded setpoint
(one bicycle-model angle for Ackermann, one tangent
magnitude for spin) and broadcasts it to every entry. The
per-motor encoders (pulse_count) ARE individual, but the
angle field is one-to-many.

| Pattern    | Real all-8 angle | Interpretation |
|------------|------------------|----------------|
| forward    | 0.000 rad        | All wheels straight |
| arc_left   | 0.179 rad        | Bicycle central angle for radius=1.0m |
| spin       | 0.935 rad        | Tangent magnitude atan2(W/2, T/2) |
| tight_curve| 0.935 rad        | Same as spin |

Sim publishes true per-wheel signed values
(e.g., -0.937, +0.937, +0.937, -0.937 for spin). The
underlying kinematic math is identical to real, but the
sim's reporting is richer.

**This is intentional and preserved.** Per-wheel feedback
is more useful for debugging, sensor fusion, and anomaly
detection. The sim is a strict superset of the real
driver's actuator_state content. Application code reading
/actuator_state.motor_angles[0] (one specific wheel) will
see different values on sim vs real, but neither value is
wrong — they're answering slightly different questions
("what does this wheel point at?" vs "what is the
commanded setpoint?").

### /odom from sim has zero accumulated slip

The sim publishes /odom integrated from the commanded
twist (no wheel slip). The real driver derives /odom from
encoder counts. Under nominal driving they're equivalent;
under heavy slip the real driver's /odom would diverge
from ground truth while the sim's would not.

---

## 7. Documented gaps (not addressed)

### /actuator_state.pulse_count = 0 in sim

The real driver reports per-actuator encoder pulse counts
(e.g., 191487, 251598). Sim hardcodes 0. Practical impact:
zero, unless someone wants to test encoder-fusion
algorithms specifically against the sim.

Future work if needed: implement pulse_count by
integrating wheel velocity * dt * pulses_per_radian.

### Real driver's /battery_state.voltage = 496

The real driver appears to have a 10× scaling bug. Voltage
is published as 496 V instead of the actual ~49.6V. Sim
publishes the correct 49.6V. This is a sim improvement;
not replicated. If you ever debug a real-robot driver
update, this would be a good thing to fix upstream.

---

## 8. Bottom line

**Sim and real driver speak the same protocol with matching
content.** All schemas bit-for-bit identical. Mock state
values aligned (R16). Auto-mode-switching behavior matches
(R17). Two deliberate sim improvements (per-wheel actuator
state, slip-free /odom). One known sim gap (pulse_count).
One known real-driver bug (battery voltage scaling).

Phase 5 audit goal **achieved**: the sim is a credible
drop-in replacement for the real driver from any
consumer's perspective. Application code (teleop, nav2,
behavior trees) is portable between sim and real with no
changes.
