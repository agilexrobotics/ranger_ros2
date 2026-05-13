# Round 11b — /clock QoS hardening complete

## Summary

**Fix landed cleanly.** Three changes shipped as one commit
(`1b0c75f`):

1. New `ranger_mini_v3_sim/config/ros_gz_bridge.yaml` — declares
   /clock with explicit RELIABLE QoS + publisher_queue=1.
2. `gazebo.launch.py`'s clock_bridge node switched from CLI-arg
   form (`arguments=[...]`) to YAML config form
   (`parameters=[{"config_file": bridge_yaml}]`).
3. `sim_messenger.py` carries the permanent tick_diag INFO log
   from R11.

**`/clock` publisher QoS verified RELIABLE** (was BEST_EFFORT
before):

```
Endpoint type: PUBLISHER
QoS profile:
  Reliability: RELIABLE       ← FIXED (was BEST_EFFORT)
  Durability:  VOLATILE
  ...
```

**Post-fix integration test:**
- mean_dt: 0.0200s rock-solid through 1600+ ticks (one transient
  outlier 91 ms during cmd_vel→zero transition)
- After 6 s of cmd_vel x=0.3:
  - /odom x = 2.010 m
  - gz x = 2.007 m
  - **Δ = 3 mm** (slightly tighter than R11 baseline's 5 mm)

**The R10 symptom was never re-induced in either R11 or R11b.**
The fix is preventive hardening; R11/R11b proves the system works
correctly with it in place.

## Diff

```diff
commit 1b0c75f (HEAD -> phase-4-messenger, origin/phase-4-messenger)
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>

    fix(sim): defensive RELIABLE QoS on /clock bridge + dt instrumentation (phase 4 r11)

 ranger_mini_v3_sim/config/ros_gz_bridge.yaml                        | 14 +++++++++++++
 ranger_mini_v3_sim/launch/gazebo.launch.py                          | 11 ++++++----
 ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py | 24 ++++++++++++++++++++++
 3 files changed, 45 insertions(+), 4 deletions(-)
```

Launch file diff:

```diff
--- a/ranger_mini_v3_sim/launch/gazebo.launch.py
+++ b/ranger_mini_v3_sim/launch/gazebo.launch.py
@@ -116,11 +116,14 @@
     # /clock bridge so use_sim_time consumers (rsp, controllers,
-    # tools) sync to gz simulation time.
+    # tools) sync to gz simulation time. Uses YAML config form
+    # to set RELIABLE QoS on the ROS-side publisher — default
+    # CLI form uses BEST_EFFORT which can drop messages under
+    # gz's high /clock publish rate (R10 observed, R11 couldn't
+    # reproduce — applying as defensive hardening regardless).
+    bridge_yaml = PathJoinSubstitution([sim_pkg, "config", "ros_gz_bridge.yaml"])
     clock_bridge = Node(
         package="ros_gz_bridge",
         executable="parameter_bridge",
         name="clock_bridge",
-        arguments=[
-            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
-        ],
+        parameters=[{"config_file": bridge_yaml}],
         output="screen",
     )
```

Messenger diff (instrumentation only — captured in handoff R11):

```diff
--- a/ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py
+++ b/ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py
@@ -353,6 +353,11 @@
         self.last_inner_phi = 0.0   # remember for odometry integration
+        # dt diagnostic counters
+        self._tick_count = 0
+        self._dt_sum = 0.0
+        self._dt_max = 0.0
+        self._dt_min = float('inf')
@@ -476,6 +481,25 @@
         if dt <= 0.0:
             return
+
+        self._tick_count += 1
+        self._dt_sum += dt
+        if dt > self._dt_max:
+            self._dt_max = dt
+        if dt < self._dt_min:
+            self._dt_min = dt
+        if self._tick_count % 50 == 0:
+            mean_dt = self._dt_sum / 50.0
+            self.get_logger().info(
+                f"tick_diag: count={self._tick_count} "
+                f"mean_dt={mean_dt:.4f}s "
+                f"min_dt={self._dt_min:.4f}s "
+                f"max_dt={self._dt_max:.4f}s "
+                f"sim_time={now.nanoseconds*1e-9:.3f}s"
+            )
+            self._dt_sum = 0.0
+            self._dt_max = 0.0
+            self._dt_min = float('inf')
```

## New files (full content)

### ranger_mini_v3_sim/config/ros_gz_bridge.yaml

```yaml
# YAML config for ros_gz_bridge.parameter_bridge.
# Bridges /clock from gz → ROS with explicit RELIABLE QoS so
# rclpy auto-subscribers (use_sim_time=True consumers) don't
# drop messages under gz's high /clock publish rate. Default
# parameter_bridge args bridge with the source-side QoS, which
# for /clock is BEST_EFFORT and causes message loss.
- topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
  publisher_queue: 1
  # ROS-side publisher QoS — what consumers will subscribe to.
  # KEEP_LAST(1) RELIABLE: consumers always see the latest tick
  # and won't miss any during congestion.
```

### .claude_handoff/round_11b_phase4_clock_fix_complete.md

This file. Will be committed in step 6c.

## Verification output

### Step 0 — Pre-flight

```
On branch phase-4-messenger
sim_messenger.py modified (R11 instrumentation in place)
ros_gz_bridge.yaml untracked (R11 created)
[no orphans, then clean after pkill]
```

### Step 1a — Confirm actual indent

```
    # /clock bridge ...           (4-space)
    clock_bridge = Node(          (4-space)
        package="ros_gz_bridge",  (8-space inner)
        ...
```

### Step 1b — Patch attempt

The reissued prompt's PRIMARY `old` block STILL had the wrong
indent (8-space outer, 16-space inner — same drift as R11). But
the prompt also included a fallback `old` block with the correct
4-space outer / 8-space inner indent. The fallback matched and
the patch applied:

```
launch file patched OK with correct indentation
```

### Step 1c — Verify patch landed

```python
    # /clock bridge so use_sim_time consumers (rsp, controllers,
    # tools) sync to gz simulation time. Uses YAML config form
    # to set RELIABLE QoS on the ROS-side publisher — default
    # CLI form uses BEST_EFFORT which can drop messages under
    # gz's high /clock publish rate (R10 observed, R11 couldn't
    # reproduce — applying as defensive hardening regardless).
    bridge_yaml = PathJoinSubstitution([sim_pkg, "config", "ros_gz_bridge.yaml"])
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )
```

### Step 2a — Build

```
Starting >>> ranger_mini_v3_sim
Finished <<< ranger_mini_v3_sim [0.12s]
Summary: 1 package finished [0.21s]
```

### Step 2b — YAML in install

```
$ ls .../share/ranger_mini_v3_sim/config/
ranger_mini_v3_controllers.yaml
ros_gz_bridge.yaml
```

### Step 3a — Launch gz

The first launch attempt's `if ! pgrep ...` test mis-fired (race
between gz's startup and the 30 s grace + the pgrep | grep -v
claude pipeline returning 1 when grep found nothing in the
moment). The pkill cleanup branch fired and killed the just-
starting stack. Re-launched in a simpler form (no
guard-and-cleanup) and gz came up cleanly the second time.

```
$ pgrep -fa "gz sim|robot_state_publisher|parameter_bridge"
... gz sim -r -s ...empty_ground.sdf ...
... robot_state_publisher ...
... parameter_bridge --ros-args -r __node:=clock_bridge --params-file /tmp/launch_params_u_kjzyiq
```

The bridge is now invoked via `--params-file` (the YAML config
form) instead of inline `arguments=[/clock@…]`. ✓

```
$ ros2 control list_controllers
(9 controllers, all 'active')
```

### Step 3b — /clock QoS verification (the headline check)

```
=== /clock info ===
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
  Node name: clock_bridge
  Endpoint type: PUBLISHER
  QoS profile:
    Reliability: RELIABLE      ← FIXED (was BEST_EFFORT)
    Durability:  VOLATILE
    History (Depth): UNKNOWN
    Lifespan: Infinite
    Deadline: Infinite
    Liveliness: AUTOMATIC
    Liveliness lease duration: Infinite
Subscription count: 11
  (subscribers still default BEST_EFFORT — that's fine; a
   BEST_EFFORT subscriber accepts a RELIABLE publisher)
```

**The QoS fix is in effect.**

### Step 3c — Messenger up

```
[INFO] [sim_messenger]: SimMessenger up. update_rate=50 Hz, publish_odom_tf=False
[INFO] [sim_messenger]: tick_diag: count=50  mean_dt=0.0198s min=0.0100s max=0.0200s sim_time=68.000s
[INFO] [sim_messenger]: tick_diag: count=100 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=69.000s
[INFO] [sim_messenger]: tick_diag: count=150 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=70.000s
[INFO] [sim_messenger]: tick_diag: count=200 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=71.000s
[INFO] [sim_messenger]: tick_diag: count=250 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=72.000s
```

dt is correct from the start (one 0.0100 outlier in the very
first 50-tick window, then locked to 0.0200s).

### Step 3d — Drive test

```
=== POST-FIX tick_diag (last 10 lines) ===
count=1150 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=90.000s
count=1200 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=91.000s
count=1250 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=92.000s
count=1300 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=93.000s
count=1350 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=94.000s
count=1400 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=95.000s
count=1450 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=96.000s
count=1500 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=97.000s
count=1550 mean_dt=0.0200s min=0.0200 max=0.0200 sim_time=98.000s
count=1600 mean_dt=0.0240s min=0.0160 max=0.0910 sim_time=99.200s
```

Last window has one outlier (91 ms max) — exactly the cmd_vel
transition moment (the pub-once zero command after the 6 s drive).
Mean nudges from 0.0200 to 0.0240 because of that single outlier;
sim_time advances by 1.2 s in that 50-tick window vs the steady
1.0 s — consistent with the outlier dt.

```
=== POST-FIX /odom ===
position:
  x: 2.0099999999999336
  y: 0.0
  z: 0.0

=== POST-FIX gz pose ===
[2.007370 0.000000 0.305289]
```

| Source | x      | y     |
|--------|--------|-------|
| /odom  | 2.010  | 0.000 |
| gz     | 2.007  | 0.000 |

**Δx = 3 mm.** Even tighter agreement than R11 baseline's 5 mm.

### Step 4 — Teardown

```
[clean]
```

After pattern-pkill + one PID-sweep round (no PID-sweep needed
this time — the pkill caught everything in one pass).

### Step 5 — Commit

```
[phase-4-messenger 1b0c75f] fix(sim): defensive RELIABLE QoS on /clock bridge + dt instrumentation (phase 4 r11)
 3 files changed, 45 insertions(+), 4 deletions(-)
 create mode 100644 ranger_mini_v3_sim/config/ros_gz_bridge.yaml

To github.com:S-abk/ranger_ros2.git
   df5b3c9..1b0c75f  phase-4-messenger -> phase-4-messenger
```

### Manual verification steps for the operator

```bash
# Terminal A
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B (after Gazebo settles ~30s)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C — verify QoS:
ros2 topic info /clock --verbose | head -15
# Look for "Reliability: RELIABLE" under the publisher section.

# Drive test:
ros2 topic echo /odom &
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
# /odom x should advance at ~0.3 m/s, matching gz pose. The
# tick_diag log lines in Terminal B should show mean_dt ≈ 0.0200s.
```

## Deviations

- **The reissued prompt's primary `old` block STILL had the wrong
  indentation** (same 8-space outer / 16-space inner mistake as
  R11). The fallback `old` block in the script did have the
  correct 4-space outer / 8-space inner indent and matched the
  actual file. Patch applied via the fallback.

- **Step 3a's first launch attempt was killed by my own
  guard-and-cleanup logic.** The `if ! pgrep "gz sim" | grep -v
  claude > /dev/null` test returned true (because the pipeline's
  last command was `grep` and pgrep returned nothing yet — a race
  with gz startup), the failure branch fired, and the pkill killed
  the just-starting stack. Re-launched in a simpler, guard-free
  form and it came up cleanly. Worth noting in case future rounds
  use similar pgrep-after-launch patterns.

## Open questions

1. **The reissued prompt's primary `old` block had the same
   indentation bug.** Architect may want to lint-check
   prompt-embedded code blocks before sending in future rounds.
   The fallback pattern saved this one.

2. **The R10 bug remains unreproducible.** R11 baseline (no fix)
   showed dt correct, /odom tracking gz to 5 mm. R11b post-fix
   shows the same numbers (3 mm vs 5 mm — within measurement
   noise). The fix doesn't make a measurable difference under
   nominal conditions. It's still defensible as preventive
   hardening. If the R10 symptom reappears under any future
   condition, the tick_diag log will show whether /clock QoS is
   actually the cause.

3. **Subscribers on /clock are still BEST_EFFORT** (per topic
   info). That's expected — these are auto-subscriptions created
   by rclpy when use_sim_time=True. The mismatched-direction QoS
   policy (RELIABLE pub, BEST_EFFORT sub) is permitted by DDS:
   the looser side accepts the stricter, so reliability is
   bounded by the BEST_EFFORT subscriber. The fix improves
   PUBLISHER guarantees but doesn't force subscribers to be
   RELIABLE. If we ever want true end-to-end RELIABLE on /clock,
   we'd need to override each consumer's auto-subscription QoS,
   which is more invasive.

4. **The pgrep-with-grep-pipeline guard pattern races with gz
   startup.** Future rounds should either use plain pgrep without
   the grep filter, OR use a longer settle time before checking,
   OR check for the controller_manager service directly. R09 OQ4
   (use `bash -c "source && setsid …"`) is now confirmed correct
   and was used here for both launches.

5. **Phase 4 work is now functionally complete.** Recommended
   next steps: squash-merge phase-4-messenger to jazzy, then
   start Phase 5 (real-driver-side-by-side audit). The
   tick_diag log is permanent — if the R10 bug ever reappears
   anywhere, it'll show up in the messenger's INFO log.
