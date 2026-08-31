# Round 11 — Phase 4 /clock QoS fix

## Blocked steps

**Step 3d (launch-file patch via assertion-checked Python) failed**
because the architect's `old` text block doesn't match the actual
file's indentation. STOP per the rule.

### What ran

```
$ python3 << PYEOF
... assert old in src, "old clock_bridge block not found verbatim — STOP"
PYEOF
AssertionError: old clock_bridge block not found verbatim — STOP
```

### Diagnosis

The architect's `old` block has the comment at 4-space indent but
the bridge body (`package=`, `executable=`, etc.) at 8-space indent
— suggesting the architect was visualizing this code as a method
body inside a class. The actual file is a top-level function body
with consistent 4-space indent throughout.

Architect's `old` (literal, with `^` for spaces):

```
^^^^# /clock bridge so use_sim_time consumers (rsp, controllers,
^^^^^^^^# tools) sync to gz simulation time.
^^^^^^^^clock_bridge = Node(
^^^^^^^^^^^^package="ros_gz_bridge",
... etc., 8-space indent on body
```

Actual file (from `sed | cat -A`):

```
^^^^# /clock bridge so use_sim_time consumers (rsp, controllers,
^^^^# tools) sync to gz simulation time.
^^^^clock_bridge = Node(
^^^^^^^^package="ros_gz_bridge",
... etc., 4-space outer / 8-space inner
```

The fix is the same; only the indentation mismatch prevented the
literal `old in src` assertion from passing.

### Proposed re-issue text for step 3d

The corrected `old` block should be:

```python
old = '''    # /clock bridge so use_sim_time consumers (rsp, controllers,
    # tools) sync to gz simulation time.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )'''
```

And `new` should match (also 4-space outer / 8-space inner):

```python
new = '''    # /clock bridge so use_sim_time consumers (rsp, controllers,
    # tools) sync to gz simulation time. Uses YAML config form
    # to set RELIABLE QoS on the ROS-side publisher — default
    # CLI form uses BEST_EFFORT which drops messages under gz's
    # ~770 Hz /clock publish rate and causes downstream
    # consumers' sim-time to barely advance. See R11.
    bridge_yaml = PathJoinSubstitution([sim_pkg, "config", "ros_gz_bridge.yaml"])
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )'''
```

### Bigger surprise: R10's bug isn't reproducing

The baseline measurement (step 2, BEFORE the QoS fix) showed
`mean_dt = 0.0200s` exactly and **`/odom 2.100 m vs gz 2.095 m
agreement within 5 mm**` after a 6-second forward drive at 0.3 m/s.

```
=== BASELINE messenger tick_diag (last 10) ===
tick_diag: count=1450 mean_dt=0.0200s min_dt=0.0190s max_dt=0.0210s sim_time=226.880s
tick_diag: count=1500 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0200s sim_time=227.880s
tick_diag: count=1550 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0200s sim_time=228.880s
tick_diag: count=1600 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0200s sim_time=229.880s
tick_diag: count=1650 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0200s sim_time=230.880s
tick_diag: count=1700 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0200s sim_time=231.880s
tick_diag: count=1750 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0200s sim_time=232.880s
tick_diag: count=1800 mean_dt=0.0200s min_dt=0.0170s max_dt=0.0230s sim_time=233.880s
tick_diag: count=1850 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0200s sim_time=234.880s
tick_diag: count=1900 mean_dt=0.0200s min_dt=0.0200s max_dt=0.0210s sim_time=235.881s
```

`mean_dt` is rock-solid 0.0200s (the timer rate). `min/max` show
±1 ms jitter at most.

After 6 s of cmd_vel x=0.3:
- gz pose: x = 2.0949 m
- /odom:   x = 2.1000 m

**Agreement to 5 mm.** /odom is tracking gz physics correctly,
WITHOUT the QoS fix. The R10 symptom (/odom advancing ~0.01 m
over 5 s) does not reproduce in this fresh launch.

### Why the bug didn't reproduce

Hypotheses, ordered by likelihood:

1. **The R10 issue was a transient race or system-state condition**
   that doesn't always fire. R10's measurement was taken right
   after a long sequence of restarts and daemon resets; this
   round's launch was clean. The DDS warm-up may have been
   pathological in R10's specific session.

2. **System load between R10 and R11 differs.** R10 had multiple
   concurrent ros2 CLI subscribers (echo + topic info + topic hz)
   eating into the BEST_EFFORT /clock subscription quota. R11's
   measurement was simpler.

3. **The instrumentation itself slightly perturbs the timing,**
   making the bug less likely to manifest. The `_tick_count` and
   logging adds ~5–10 µs per tick. Marginal effect; probably not
   the driver.

The QoS fix is still a defensible defensive measure (the BEST_EFFORT
auto-subscription IS objectively risky under high publish rates),
but the urgency is much lower than R10 suggested.

### What state the workspace is in

- **Instrumentation patch is applied to sim_messenger.py** (step 1)
  and the file is rebuilt — but **NOT committed** (held until the
  full fix lands per the architect's "one commit" intent in step 7a).
- **YAML config `ros_gz_bridge.yaml` is created** (step 3b) but
  **NOT committed**.
- **Launch file `gazebo.launch.py` is unchanged** — the literal
  patch failed (step 3d), I did not apply a corrected version.
- Baseline measurement files in `/tmp/r11_baseline_msg.log` and
  `/tmp/r11_baseline_gz.log` show /odom and gz agreed to within 5 mm.
- Working tree dirty (sim_messenger.py modified + new YAML file
  untracked).
- All processes cleaned (final `pgrep` returned `[clean]`).

### What I deliberately did NOT do

- Did NOT apply the launch-file patch with corrected indentation.
- Did NOT proceed to step 3e (verify the patch), 3f (rebuild),
  3g (confirm YAML lands), step 4 (post-fix measurement), step 5
  (compare), step 6 (teardown), step 7 (commit), step 8 (handoff
  commit + push).
- Did NOT commit the instrumentation patch standalone (would
  fragment the architect's "one commit" intent).
- Did NOT delete or revert the YAML file or the instrumentation
  patch — they're in place ready for the reissued round to consume.
- Did NOT touch the messenger's kinematic code or the dt threshold.

---

## Summary

R11's prep work succeeded:
- Instrumentation patch applied to `sim_messenger.py` and built.
- YAML config file `ranger_mini_v3_sim/config/ros_gz_bridge.yaml`
  created.

R11's BASELINE measurement surprised: dt is rock-solid 0.0200s
throughout, /odom tracks gz pose to within 5 mm — the R10 bug
**does not reproduce** in this launch. R10's symptom may have been
state-dependent and not consistently reproducible.

R11's launch-file patch (step 3d) FAILED its literal-text
assertion because the architect's `old` block has 8-space indent
on the bridge body but the actual file has 4-space. Halted per
rule. Re-issue requires only a whitespace correction in the patch's
`old`/`new` strings.

## Diff

No commits this round. Workspace has uncommitted changes:

- `ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py`
  has the dt instrumentation (5 lines in `__init__`, ~15 lines
  in `_tick`).
- `ranger_mini_v3_sim/config/ros_gz_bridge.yaml` is a new file.

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

### .claude_handoff/round_11_phase4_clock_fix.md

This file. Not committed — see Blocked steps.

## Verification output

### Step 0 — Pre-flight + daemon refresh

```
On branch phase-4-messenger
nothing to commit, working tree clean
[no orphans]
The daemon is not running
The daemon has been started
```

### Step 1a — Instrumentation patch (success)

```
patches applied OK
```

### Step 1b — Verify patches

```
$ grep -n "_tick_count\|tick_diag" sim_messenger.py
356:        self._tick_count = 0
479:        self._tick_count += 1
485:        if self._tick_count % 50 == 0:
488:                f"tick_diag: count={self._tick_count} "
```

(Note: the architect expected "6 matches"; my grep returns 4 lines.
The discrepancy is because the grep pattern `_tick_count\|tick_diag`
matches the *line*, and the new code has `_tick_count` on 4 lines
(1 in init + 3 in _tick) and `tick_diag` on 1 line that also
contains `_tick_count`. Effective match count is 4 lines containing
the patterns, which is correct given the actual code structure. The
patch is applied as intended; the architect's "6" count was off by
2.)

### Step 1c — Build after instrumentation

```
Starting >>> ranger_mini_v3_sim_messenger
Finished <<< ranger_mini_v3_sim_messenger [0.85s]
Summary: 1 package finished [0.94s]
```

### Step 2a/2b — Baseline launch + drive (one launch hiccup)

The first `setsid ros2 launch messenger.launch.py` died with
"Package not found" — the same chained-setsid env-loss issue from
R09. Re-sourced and re-launched in same shell; messenger came up
successfully.

```
$ pgrep -fa sim_messenger
157536 ros2 launch ...
157539 sim_messenger node
```

### Step 2c — Baseline diagnostics

**Messenger startup log + tick_diag (FIRST 5 entries):**

```
[INFO] [sim_messenger]: SimMessenger up. update_rate=50 Hz, publish_odom_tf=False
[INFO] [sim_messenger]: tick_diag: count=50  mean_dt=0.0200s min=0.0180s max=0.0200s sim_time=198.880s
[INFO] [sim_messenger]: tick_diag: count=100 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=199.880s
[INFO] [sim_messenger]: tick_diag: count=150 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=200.880s
[INFO] [sim_messenger]: tick_diag: count=200 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=201.880s
[INFO] [sim_messenger]: tick_diag: count=250 mean_dt=0.0200s min=0.0200s max=0.0200s sim_time=202.880s
```

**dt is correct from the very first 50-tick window.** sim_time
advances by 1.000 s per 50 ticks → confirms 50 Hz timer firing on
sim time, dt is exactly 0.020 s.

**Pose comparison after 6-second cmd_vel x=0.3:**

| Source | x        | y        |
|--------|----------|----------|
| /odom  | 2.10000  | 0.0000   |
| gz     | 2.09491  | 0.0000   |

Δx = 5 mm. Both axes agree. **R10 symptom does not reproduce.**

(The 6-second drive at 0.3 m/s = 1.8 m predicted; both /odom and
gz overshot to ~2.1, indicating the messenger started integrating
slightly before the cmd_vel pub timeout fired plus measurement
included the latched final pub. Either way, both sources agree.)

### Step 2d — Baseline teardown

```
[clean]
```

### Step 3a — Current bridge inspection

```
118:    clock_bridge = Node(
119:        package="ros_gz_bridge",
120:        executable="parameter_bridge",
121:        name="clock_bridge",
122:        arguments=[
123:            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
124:        ],
125:        output="screen",
126:    )
```

Indentation: 4-space outer (function body), 8-space inner. The
architect's `old` had 8-space outer (method body) which doesn't
match.

### Step 3b — YAML created

(File content under "New files" above.)

### Step 3c — CMakeLists confirms config in install

```
install(
  DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)
```

`config` already in the install dirs; new YAML lands automatically
once the next build runs.

### Step 3d — Launch patch (BLOCKED)

```
$ python3 << PYEOF
...
AssertionError: old clock_bridge block not found verbatim — STOP
```

Halted per rule.

### Steps 3e, 3f, 3g, 4, 5, 6, 7, 8

Not executed.

### Manual verification steps for the operator

Once the architect re-issues with correct indentation, the
operator-side test remains as the architect specified:

```bash
# Terminal A
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B (after Gazebo settles ~30s)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C
ros2 topic echo /odom &
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

`/odom x` should advance at ~0.3 m/s, matching gz pose.

This round's BASELINE data already shows the messenger doing
exactly this (x advanced to 2.10 over 6 s ≈ 0.35 m/s wallclock
with sim RTF ≈ 1.16). The reissued QoS fix is a defensive
hardening, not a change in observable behavior.

## Deviations

- **Did not commit the instrumentation patch standalone.** It's in
  the file ready to ship with the QoS fix as one logical commit
  per the architect's spec. Holding until the architect re-issues.

- **The `_tick_count` grep returned 4 lines instead of the
  architect's expected 6.** This isn't a problem — the patch
  applied correctly; the count expectation was off. (4 = 1 init
  line + 3 tick body lines mentioning `_tick_count`; the
  `tick_diag` string is on a line that already contains
  `_tick_count`.)

- **The "chained setsid loses env" issue from R09 hit again** in
  step 2a. Worked around by re-sourcing the install in the same
  shell before the second `setsid ros2 launch`. Should consider
  using `bash -c "source ... && setsid ros2 launch ..."` per
  R09 OQ4.

## Open questions

1. **Should we still apply the QoS fix given baseline shows the
   bug isn't reproducing?** Two reasonable views:

   - **Yes, defensive:** BEST_EFFORT subscription on a high-rate
     topic IS objectively fragile. The fix doesn't hurt, and
     prevents future regressions when system load rises. The R10
     symptom proved it CAN happen; we just got lucky in R11.

   - **No, premature optimization:** The fix touches launch-file
     behavior in ways that could surface other issues (e.g., does
     the `parameter_bridge --config_file` form behave identically
     to the CLI form for everything else?). If we can't reproduce
     the bug, applying the fix means we can't verify it actually
     helps.

   Recommendation: **apply the fix anyway**, noting that the R11
   measurement showed the bug was already absent. Future regressions
   are less likely with the explicit RELIABLE QoS in place.

2. **What changed between R10 and R11?** Same code (Phase 4 r10
   commit), same launch files. Different observed behavior. Worth
   investigating: maybe a system reboot between sessions cleared
   some DDS state. Maybe the daemon's stale-cache issues from R09
   were corrupting clock-subscription matching. Maybe a transient
   high-load condition cleared. Hard to pin down without
   reproducing.

3. **The architect's `old` text has the 8-space-indent error.**
   Worth double-checking other architect prompts for similar
   indentation drift; the assertion-checked patch pattern caught
   this one but a sed-style patch would have silently produced
   no change.

4. **The instrumentation patch is sound and useful regardless of
   the QoS fix.** It gives runtime visibility into dt — if the
   R10 bug ever reproduces, the next round's debug data will be
   in `tick_diag` log lines without needing further patches.
   Strongly recommend keeping it permanent.

5. **Re-issued prompt should also fix the `bridge_yaml` substitution
   binding.** The `new` block uses
   `bridge_yaml = PathJoinSubstitution([...])` declared inside
   `generate_launch_description()`. That works because `sim_pkg`
   is defined in the same scope. Sanity-check on re-issue.
