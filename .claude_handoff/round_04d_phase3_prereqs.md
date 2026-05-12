# Round 04d — Phase 3 prerequisites

## Summary

All seven `ros-jazzy-*` packages confirmed installed by the operator
and ROS-discoverable. Headless gz sim 8.x toolchain smoke test
end-to-end passes: `gz sim -s -r empty.sdf` runs, gz topics
(`/clock`, `/world/empty/...`) appear, `ros_gz_bridge parameter_bridge`
forwards `/clock` to ROS, and `ros2 topic echo /clock --once` returns
a real timestamp (`sec: 34, nanosec: 848000000`). Stale Copilot
branch deleted from origin. **Library inspection of
`libgz_ros2_control-system.so` reveals the Gazebo system-plugin class
is `gz_ros2_control::GazeboSimROS2ControlPlugin`** — our xacro
currently writes `…GazeboSimROS2ControlSystem`. This is a definite
mismatch that Phase 3 must fix; not fixing this round per prompt.

## Diff

This round's commit (after step 5c) appends only handoff artifacts —
no package files modified. The substantive output is the verification
findings, especially the plugin-name discovery that determines a
Phase 3 xacro tweak.

```diff
docs(handoff): round 04d — phase 3 prereqs and toolchain smoke test

 .claude_handoff/CONTEXT.md                    | 14 ++++++
 .claude_handoff/round_04d_phase3_prereqs.md   | (this file)
```

## New files (full content)

### .claude_handoff/round_04d_phase3_prereqs.md

This file. Will be committed in step 5c.

## Verification output

### Step 1b — Verify installation

```
=== dpkg list ===
ros-jazzy-controller-manager 4.44.0-1noble.20260412.063942
ros-jazzy-gz-ros2-control 1.2.17-1noble.20260412.065025
ros-jazzy-joint-state-broadcaster 4.39.0-1noble.20260412.065204
ros-jazzy-position-controllers 4.39.0-1noble.20260412.065512
ros-jazzy-ros-gz-bridge 1.0.22-1noble.20260412.043437
ros-jazzy-ros-gz-sim 1.0.22-1noble.20260412.044601
ros-jazzy-velocity-controllers 4.39.0-1noble.20260412.065514

=== ROS pkg prefixes ===
ros_gz_sim                     /opt/ros/jazzy
ros_gz_bridge                  /opt/ros/jazzy
controller_manager             /opt/ros/jazzy
joint_state_broadcaster        /opt/ros/jazzy
position_controllers           /opt/ros/jazzy
velocity_controllers           /opt/ros/jazzy
gz_ros2_control                /opt/ros/jazzy
```

All 7 packages present and ROS-discoverable.

### Step 2a — Library symbol inspection

```
=== Library file ===
-rw-r--r-- 1 root root 408304 Apr 12 02:50 /opt/ros/jazzy/lib/libgz_ros2_control-system.so

=== Symbols matching GazeboSim or ROS2Control ===
(many; key matches below)
000000000002abd0 T _ZN15gz_ros2_control26GazeboSimROS2ControlPlugin9ConfigureERKm…
0000000000031d60 T _ZN15gz_ros2_control26GazeboSimROS2ControlPlugin9PreUpdate…
0000000000032530 T _ZN15gz_ros2_control26GazeboSimROS2ControlPlugin10PostUpdate…
0000000000028840 T _ZN15gz_ros2_control26GazeboSimROS2ControlPluginC1Ev   (constructor)
…
0000000000062100 D _ZTIN12class_loader4impl18AbstractMetaObjectIN15gz_ros2_control24GazeboSimSystemInterfaceEEE
0000000000062368 D _ZTIN15gz_ros2_control24GazeboSimSystemInterfaceE
```

Demangled mangling tells us the C++ class name length-prefix-encoded:

- `26GazeboSimROS2ControlPlugin` → 26-char identifier
  `GazeboSimROS2ControlPlugin` in namespace `gz_ros2_control`.
- `24GazeboSimSystemInterface` → the hardware-side base class.

Confirmed by demangled strings:

```
=== Demangled strings in library ===
GazeboSimROS2ControlPlugin
N15gz_ros2_control24GazeboSimSystemInterfaceE
N15gz_ros2_control26GazeboSimROS2ControlPluginE
N9pluginlib11ClassLoaderIN15gz_ros2_control24GazeboSimSystemInterfaceEEE
…
```

**The Gazebo system plugin class actually exposed by the library is
`gz_ros2_control::GazeboSimROS2ControlPlugin`** (not `…ControlSystem`).
Our xacro currently writes:

```xml
<plugin filename="gz_ros2_control-system"
        name="gz_ros2_control::GazeboSimROS2ControlSystem">
```

This needs to become, in Phase 3:

```xml
<plugin filename="gz_ros2_control-system"
        name="gz_ros2_control::GazeboSimROS2ControlPlugin">
```

The `filename` (`gz_ros2_control-system` → `libgz_ros2_control-system.so`)
is correct. Only the `name` attribute is wrong.

### Step 2b — Plugin descriptor XMLs

```
=== gz_hardware_plugins.xml ===
<library path="gz_hardware_plugins">
  <class
    name="gz_ros2_control/GazeboSimSystem"
    type="gz_ros2_control::GazeboSimSystem"
    base_class_type="gz_ros2_control::GazeboSimSystemInterface">
    <description>
      ros2_control hardware component to be loaded by the gazebo plugin.
    </description>
  </class>
  <class
    name="ign_ros2_control/IgnitionSystem"
    type="ign_ros2_control::IgnitionSystem"
    base_class_type="gz_ros2_control::GazeboSimSystemInterface">
    <description>
      ros2_control hardware component to be loaded by the gazebo plugin.
      For backward compatibility with old ign* plugin names.
    </description>
  </class>
</library>

=== All gz_ros2_control plugin XMLs ===
/opt/ros/jazzy/share/gz_ros2_control/gz_hardware_plugins.xml
/opt/ros/jazzy/share/gz_ros2_control/package.xml
```

This XML registers **`gz_ros2_control/GazeboSimSystem`** as a
pluginlib class — that's the *hardware-system class* used inside
`<ros2_control><hardware><plugin>...</plugin></hardware></ros2_control>`,
which our xacro already has correct.

The Gazebo system plugin (the outer plugin loaded by `gz sim` from
`<gazebo><plugin filename name>`) is **not** in this pluginlib XML
because it's not loaded by pluginlib. It's loaded directly by gz
sim's plugin loader from the .so file's `GZ_ADD_PLUGIN` registration.
Hence the only authoritative source for the outer plugin's class
name is the `nm`/`strings` output in step 2a.

### Step 2c — Plugin registration symbols

```
=== Plugin registration symbols ===
(no matches)
```

`GZ_ADD_PLUGIN` macros expand into anonymous-namespace static
constructors that don't surface through `nm -D` symbol filters for
`GZ_ADD|IGNITION_ADD|RegisterPlugin`. Step 2a's class-symbol
discovery is what we rely on.

### Step 3a — gz sim headless launch

```
Started gz sim with PID=90092
gz sim is running OK

=== gz topics ===
/clock
/gazebo/resource_paths
/stats
/world/empty/clock
/world/empty/dynamic_pose/info
/world/empty/pose/info
/world/empty/scene/deletion
/world/empty/scene/info
/world/empty/state
/world/empty/stats
/world/empty/light_config
/world/empty/material_color

=== gz_sim.log tail ===
(empty — no errors)
```

`/clock` published by gz; world-namespaced topics for empty world
appear. Process alive.

### Step 3b — `ros_gz_bridge` help

```
Bridge a collection of ROS2 and Gazebo Transport topics and services.

  parameter_bridge [<topic@ROS2_type@Ign_type> ..]  [<service@ROS2_srv_type[@Ign_req_type@Ign_rep_type]> ..]

Topics: The first @ symbol delimits the topic name from the message types.
Following the first @ symbol is the ROS message type.
The ROS message type is followed by an @, [, or ] symbol where
    @  == a bidirectional bridge, 
    [  == a bridge from Gazebo to ROS,
    ]  == a bridge from ROS to Gazebo.
```

### Step 3c — End-to-end `/clock` bridge

```
Started bridge with PID=90465
Bridge is running OK

=== ROS topics ===
/clock
/parameter_events
/rosout

=== /clock info ===
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
Subscription count: 0

=== /clock first message (5s timeout) ===
clock:
  sec: 34
  nanosec: 848000000
---
```

**End-to-end pipeline confirmed:** gz sim publishes gz `/clock` →
`parameter_bridge` translates to ROS `/clock` → `ros2 topic echo`
receives a real timestamp. The toolchain works.

### Step 3d — Teardown

The original PID-based teardown was insufficient — the PIDs the
parent shells captured (`90092`, `90465`) were the wrappers; the
actual long-running processes had different PIDs (`90109` for
gz sim, `90482`/`90485` for the bridge halves) once the wrapper
shells exited. A pattern-based `pkill -9 -f` cleanup followed by an
explicit `kill -9 90482 90485` ensured everything terminated:

```
=== Process check (should be empty) ===
[no processes left]
```

(Final `pgrep` after explicit kills returned only the agent's own
bash subprocess, which is expected and not part of the smoke test.)

### Step 4a — Confirm stale branch

```
$ git branch -r | grep -i 'fix-remove\|copilot\|sim-state'
  origin/fix-remove-unused-publish-sim-state-to-ros-10979089828855703371
```

Exactly one match.

### Step 4b — Delete from remote

```
Will delete remote branch: 'fix-remove-unused-publish-sim-state-to-ros-10979089828855703371'
To github.com:S-abk/ranger_ros2.git
 - [deleted]         fix-remove-unused-publish-sim-state-to-ros-10979089828855703371
```

### Step 4c — Verify deletion

```
$ git fetch origin --prune
$ git branch -r | grep -i 'fix-remove'
[branch gone — confirmed]
```

### Manual verification steps for the operator

- <https://github.com/S-abk/ranger_ros2/branches> — confirm the
  Copilot-style branch is no longer listed.
- The remaining branches should be: `air_delta`, `humble`, `jazzy`,
  `old-version`, `phase-1-description`, `phase-2-ros2-control`.

## Deviations

- **Process-teardown deviation in step 3d.** The literal teardown
  used the captured shell `$!` PIDs, which were the wrapper-shell
  PIDs — those exited promptly, leaving the actual gz sim and
  parameter_bridge processes orphaned to PID 1 with different PIDs.
  I had to follow up with pattern-based `pkill -9 -f` and explicit
  `kill -9 <PID>` for the long-lived halves to actually terminate.
  All processes are now gone; this is housekeeping, not a route-around
  of any verification step.

  Lesson for future rounds that background long-running processes:
  capture the PID of the actual binary via `pgrep` *after* the
  background launch settles, or use `setsid` + process group kill,
  rather than `$!` of a `cmd >log 2>&1 &` whose parent shell will
  exit before the child does in some sub-shell-execution contexts.

## Open questions

1. **Phase 3 xacro fix needed.** Change
   `name="gz_ros2_control::GazeboSimROS2ControlSystem"` to
   `name="gz_ros2_control::GazeboSimROS2ControlPlugin"` in
   `ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro`. This was
   the architect's previously-flagged "first thing to re-verify in
   Phase 3" — verified, and confirmed wrong. One-line fix at
   integration time.

2. **No Phase 4 readiness work touched this round.** Phase 4 is the
   ranger_sim_messenger Python node. It needs `ranger_msgs` (already
   a dep on the real driver side) plus `rclpy`, `geometry_msgs`,
   `nav_msgs`, `sensor_msgs`, `tf2_ros`. All of those are in the
   ROS 2 Jazzy desktop install; no extra apt is anticipated. Worth
   a brief verification sweep when Phase 4 starts.

3. **Sudo/install policy not changed by this round.** Operator ran
   the apt install manually outside the chat (architect-approved
   per-round exception); the architect's preferred long-term policy
   (Option A `!`-prefix vs Option B sudoers NOPASSWD vs Option C
   pre-install) is still open from R04d's halted handoff.
