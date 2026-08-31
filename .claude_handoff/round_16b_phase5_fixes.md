# Round 16b — Phase 5 fixes (mock values + QoS) after rebase

## Summary

R15a's branch divergence resolved cleanly via rebase (R15a handoff
re-parented onto operator's README fix; new SHA `b33f29c`).
Standing principle for future rounds (pre-flight rebase) added to
CONTEXT and committed as `6fc4853`. Then applied all 8 R16
substantive patches via assertion-checked Python:

- 6 SIM_* constants (battery voltage 49.6, percentage scale to
  0..100, driver voltage 49.5, driver temp 40, motor temp 23,
  driver_state 64).
- BatteryState.present field flipped True → False to match real
  driver's NaN-cast-to-bool quirk.
- /cmd_vel subscriber QoS flipped BEST_EFFORT → RELIABLE to match
  real driver subscriber.

Build clean. Standalone smoke test confirmed every changed value
appears in published messages exactly as expected. Full integration
test confirmed motion still works (drive 0.3 m/s for 3s →
/odom x = 1.122 m). Single source-fix commit `ce125d3` plus this
handoff.

## Diff

Three commits this round on jazzy:

1. **(rebase)** `b33f29c` — `docs(handoff): round 15a — phase 5
   sim-side topic + behavior inventory`. Re-parented from prior
   `20962ec` onto `31f08dc`.

2. `6fc4853` — `docs(handoff): standing principle — rebase before
   round work`. 1 file (CONTEXT.md), 18 insertions.

3. `ce125d3` — `fix(sim): align mock state values + cmd_vel QoS
   with real driver (phase 5 r16)`. 1 file
   (`sim_messenger.py`), +10/-10.

The substantive diff (sim_messenger.py changes summarized):

```
SIM_BATTERY_VOLTAGE  24.0  →  49.6
SIM_BATTERY_SOC       1.0  →  100.0
SIM_DRIVER_VOLTAGE   24.0  →  49.5
SIM_DRIVER_TEMP      35.0  →  40.0
SIM_MOTOR_TEMP       40.0  →  23.0
SIM_DRIVER_STATE_OK     0  →  64

batt.present = True   →  False
cmd_qos reliability = BEST_EFFORT  →  RELIABLE
```

## New files (full content)

### .claude_handoff/round_16b_phase5_fixes.md

This file. Will be committed in step 6c.

(Note: `round_16_phase5_fixes.md` is on disk uncommitted from
R16's halt — it documents the halt-cause. Will be staged
alongside this round's handoff in step 6c so the halt history
is preserved in the trail.)

## Verification output

### Step 0 — Rebase

```
$ git status (before)
Your branch and 'origin/jazzy' have diverged, 1 + 1 commits.
$ git pull --rebase origin jazzy
Rebasing (1/1)  Successfully rebased and updated refs/heads/jazzy.
$ git log --oneline -5
b33f29c docs(handoff): round 15a — phase 5 sim-side topic + behavior inventory   ← re-parented
31f08dc Correct grammar in README instructions                                    ← operator's fix
10d6487 docs(handoff): round 14 — phase 4 squash-merge to jazzy
4ee5b97 feat(sim): integrate ranger_mini_v3_sim_messenger + phase 4 work into jazzy
908c94c fix(sim): URDF dimensions match real-robot RangerMiniV3Params

$ git push origin jazzy
   31f08dc..b33f29c  jazzy -> jazzy   (clean fast-forward)
```

### Step 1 — Standing principle

CONTEXT.md got a new section "## 2026-05-14 — Standing principle
(added after R16 halt)" stating: pre-flight must include rebase
check. Future rounds either resolve cleanly via
`git pull --rebase origin <branch>` or halt and surface the
divergence to the architect.

```
[jazzy 6fc4853] docs(handoff): standing principle — rebase before round work
 1 file changed, 18 insertions(+)
   b33f29c..6fc4853  jazzy -> jazzy
```

### Step 2a — Read constants

```
50:SIM_BATTERY_VOLTAGE = 24.0       # V (typical 24V Li-ion pack)
51:SIM_BATTERY_CURRENT = -1.0       # A (negative = discharging)
52:SIM_BATTERY_TEMP    = 25.0       # °C
53:SIM_BATTERY_SOC     = 1.0        # fraction (0..1)
54:SIM_DRIVER_VOLTAGE  = 24.0       # V (driver bus = battery)
55:SIM_DRIVER_TEMP     = 35.0       # °C (warm operating)
56:SIM_MOTOR_TEMP      = 40.0       # °C
57:SIM_DRIVER_STATE_OK = 0          # 0 = no faults
```

### Step 2b — 6 constant patches

```
OK: applied SIM_BATTERY_VOLTAGE = 49.6       # 48V Li-ion pack; real rep…
OK: applied SIM_BATTERY_SOC     = 100.0      # PERCENT (0..100) to match…
OK: applied SIM_DRIVER_VOLTAGE  = 49.5       # V (driver bus = battery; …
OK: applied SIM_DRIVER_TEMP     = 40.0       # °C (real reports 40 at no…
OK: applied SIM_MOTOR_TEMP      = 23.0       # °C (real reports 23 — mot…
OK: applied SIM_DRIVER_STATE_OK = 64         # observed value on real ro…

--- all 6 constant changes applied ---
```

### Step 2c — present field

```
present field updated
```

(Matched the simple-form anchor `'batt.present = True'` since the
actual file has the bare assignment without the multi-line comment
the architect's first anchor described.)

### Step 2d — cmd_qos

```
cmd_qos updated to RELIABLE
```

(Matched the 8-space-indent form, consistent with the actual
function-body indentation in `__init__`.)

### Step 2e — Verify all changes

```
50:SIM_BATTERY_VOLTAGE = 49.6       # 48V Li-ion pack; real reports ~50.2V at full       # V (typical 24V Li-ion pack)
53:SIM_BATTERY_SOC     = 100.0      # PERCENT (0..100) to match real driver; …
54:SIM_DRIVER_VOLTAGE  = 49.5       # V (driver bus = battery; real reports 49.5)
55:SIM_DRIVER_TEMP     = 40.0       # °C (real reports 40 at normal operating temp)
56:SIM_MOTOR_TEMP      = 23.0       # °C (real reports 23 — motors run cool)
57:SIM_DRIVER_STATE_OK = 64         # observed value on real robot during normal operation; …
```

```
766:        batt.present = False  # match real driver NaN-cast quirk (see R15b)   # the real driver sets NaN here but
767-                              # `present` is bool — interpret as
768-                              # "battery present in sim"
```

```
367:        cmd_qos = QoSProfile(
368-            depth=5,
369-            reliability=ReliabilityPolicy.RELIABLE,
370-            history=HistoryPolicy.KEEP_LAST,
371-        )
```

All 8 changes present.

### Step 3a — Build

```
Starting >>> ranger_mini_v3_sim_messenger
Finished <<< ranger_mini_v3_sim_messenger [0.86s]
Summary: 1 package finished [0.95s]
```

### Step 3b — Standalone smoke test

```
=== /system_state ===
header: { stamp: …, frame_id: '' }
vehicle_state: 0
control_mode: 1
error_code: 0
battery_voltage: 49.6                   ← FIXED (was 24.0)
motion_mode: 0

=== /battery_state ===
header: { stamp: …, frame_id: '' }
voltage: 49.6                           ← FIXED (was 24.0)
temperature: 25.0
current: -1.0
charge: .nan
capacity: .nan
design_capacity: .nan
percentage: 100.0                       ← FIXED (was 1.0)
power_supply_status: 0
power_supply_health: 0
power_supply_technology: 2              ← unchanged (LION)
present: false                          ← FIXED (was true)
```

(Initial /system_state echo failed with "invalid type" — stale ros2
daemon cache. `ros2 daemon stop && ros2 daemon start` then retried,
got the values shown.)

### Step 4 — Full integration smoke test

```
=== /odom after 3s drive ===
position:
  x: 1.122            ← MOVED (drive functional; predicted 0.9m at 0.3 m/s × 3s; close)
  y: 0.0
orientation: identity

=== /actuator_state head ===
- id: 0
  motor: { rpm: 0, current: 0.0, pulse_count: 0,
           motor_angles: ~2e-20, motor_speeds: ~-1e-19 }
  driver:
    driver_voltage: 49.5                ← FIXED (was 24.0)
    driver_temperature: 40.0             ← FIXED (was 35.0)
    motor_temperature: 23.0              ← FIXED (was 40.0)
    driver_state: 64                     ← FIXED (was 0)
- id: 1  (same shape)
```

All 4 actuator driver values match expected post-fix.

(Same daemon-cache hiccup on /actuator_state initially; same
restart fix.)

### Step 4 — Teardown

```
[clean]
```

### Step 5 — Commit + push

```
[jazzy ce125d3] fix(sim): align mock state values + cmd_vel QoS with real driver (phase 5 r16)
 1 file changed, 10 insertions(+), 10 deletions(-)
   6fc4853..ce125d3  jazzy -> jazzy
```

### Manual verification steps for the operator

If the operator runs sim again, the now-published values match
what they'd see from the real driver:

```
/system_state.battery_voltage      49.6
/battery_state.voltage             49.6
/battery_state.percentage          100.0
/battery_state.present             false
/actuator_state.driver_voltage     49.5
/actuator_state.driver_temperature 40.0
/actuator_state.motor_temperature  23.0
/actuator_state.driver_state       64
```

Application code that gates on these values (low-battery alarm
thresholds, fault watchdogs that look at driver_state bitmasks)
will now behave the same in sim and on real.

## Deviations

- **Cosmetic comment leftovers from anchor mismatches.** Two
  patches' anchors didn't include the trailing comment text:
  - SIM_BATTERY_VOLTAGE: anchor was `'SIM_BATTERY_VOLTAGE = 24.0'`
    but line had `'SIM_BATTERY_VOLTAGE = 24.0       # V (typical
    24V Li-ion pack)'`. Replace produced
    `SIM_BATTERY_VOLTAGE = 49.6  # 48V Li-ion pack; real reports ~50.2V at full       # V (typical 24V Li-ion pack)`
    — Python sees one comment from the first `#`, so functionally
    fine; visually has both new and old comment text concatenated.
  - `batt.present`: simple-form anchor matched (the bare `=True`
    assignment), so the multi-line continuation comment from R09
    is now orphaned below the new line. Lines 766-768:
    ```
    batt.present = False  # ...new comment...   # the real driver sets NaN here but
                          # `present` is bool — interpret as
                          # "battery present in sim"
    ```
    Functionally fine (those continuation lines are just bare
    comments now); cosmetically messy. A follow-up cleanup round
    could squash the orphan comment lines.

- **ros2 daemon cache went stale twice** during smoke testing:
  /system_state initially returned "ranger_msgs/msg/SystemState
  is invalid", and /actuator_state did the same after the second
  launch. Each time `ros2 daemon stop && ros2 daemon start`
  followed by retry produced the correct values. This is a known
  flakiness pattern from R09/R15a; the new standing principle
  (rebase pre-flight) doesn't address daemon staleness.

- **Untracked `.claude_handoff/round_16_phase5_fixes.md` from
  R16's halt** is on disk but not yet staged. The architect's
  step 6c will stage `.claude_handoff/` which picks up both
  this file (round_16b) AND the leftover round_16. The R16
  halt handoff is forensically useful — it documents WHY R16b
  exists. Recommend keeping it.

## Open questions

1. **The cosmetic comment leftovers** are easy to clean up in a
   follow-up round if the architect wants the source pristine.
   Functionally moot.

2. **ros2 daemon staleness** keeps catching us. Worth either:
   (a) the messenger node's launch could trigger a daemon
       restart on first start, or
   (b) the `troubleshooting` section in
       `ranger_mini_v3_sim/README.md` (already mentions daemon
       restart for type cache) could be made more prominent, or
   (c) the architect's prompts could include a daemon-restart
       step before any topic-echo of ranger_msgs types.

3. **R17 — motion_mode behavior divergence investigation.** Per
   the R16 prompt, this is the next round. R15b's findings on
   motion_mode reporting differences will need empirical
   testing on the real robot to drive the fix.

4. **The R16 halt handoff** should be committed with this round
   (rather than orphaned) so the trail is complete. The next
   architect prompt's step 6c does
   `git add .claude_handoff/` which will pick it up
   automatically.
