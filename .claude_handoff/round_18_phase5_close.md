# Round 18 — Phase 5 close

## Summary

R17's continuous-capture investigation (operator-side) closed
the last open question from Phase 5: real driver DOES switch
motion_mode to 2 during spin / tight-curve commands. R15b's
snapshot-only capture had bracketed the transition window;
artifact, not a sim bug.

R18 landed the documentation:
1. **`.claude_handoff/phase5_parity_report.md`** — the full
   parity report, integrating R15a/R15b/R17 findings, never
   previously committed.
2. **`ranger_mini_v3_sim/README.md`** — new "Parity with real
   robot" section between "Motion modes" and "Troubleshooting",
   pointing readers at the parity report and listing the
   intentional sim/real differences and exact matches.
3. **`.claude_handoff/round_17_motion_mode_investigation.md`**
   — retroactive R17 handoff documenting the
   continuous-capture data and findings.
4. **CONTEXT.md update** — Rounds 17/18 entry, Phase 5 closed.

No code changes. Phase 5 is complete; sim is a drop-in
replacement for the real driver from any application's
perspective.

## Diff

Two commits this round:

1. `5a862bd docs(sim): Phase 5 parity report + README parity section (close phase 5)`
   — 2 files, +230/0. Adds the parity report and the README
   section.

2. (after this file) `docs(handoff): round 17 (motion_mode investigation) — phase 5 closed`
   — 3 files (round_17 handoff + round_18 close handoff +
   CONTEXT entry).

## New files (full content)

### .claude_handoff/round_18_phase5_close.md

This file.

### .claude_handoff/round_17_motion_mode_investigation.md

Retroactive handoff capturing R17's data + findings (see
that file for the full detail).

### .claude_handoff/phase5_parity_report.md

189-line audit report covering 8 sections:
1. Schema parity (bit-for-bit identical)
2. Topology
3. QoS profiles
4. Content alignment (post-R16)
5. Behavioral findings (motion_mode auto-switching, sign
   convention)
6. Intentional sim improvements (per-wheel actuator_state,
   slip-free /odom)
7. Documented gaps (pulse_count, real-driver voltage scaling)
8. Bottom line

Already committed in `5a862bd`.

## Verification output

### Step 0 — Pre-flight

```
On branch jazzy
nothing to commit, working tree clean
HEAD: db6c09b docs(handoff): round 16c (halt) + 16d (cleanup complete)
[in sync with origin]
[no orphans, only daemons]
```

### Step 1 — Parity report

189 lines written to `.claude_handoff/phase5_parity_report.md`.
Audit sections capture R15a/R15b structural data + R17 closure
on motion_mode behavior + R17 finding on per-wheel vs broadcast
actuator_state reporting.

### Step 2 — README parity section

Inserted between `## Motion modes` and `## Troubleshooting`:

```
$ grep -n "^## " ranger_mini_v3_sim/README.md
9:## Quick start
35:## Launch file hierarchy
47:## ROS interface (parity with real driver)
67:## Motion modes
80:## Parity with real robot      ← NEW
121:## Troubleshooting
167:## Development history
```

Section content lists 4 intentional sim/real differences
(per-wheel motor_angles, pulse_count, /odom slip, battery
voltage scaling) and 4 things that match exactly (schemas,
mode-switching, sign convention, mock defaults).

### Step 3 — Commit + push

```
[jazzy 5a862bd] docs(sim): Phase 5 parity report + README parity section (close phase 5)
 2 files changed, 230 insertions(+)
 create mode 100644 .claude_handoff/phase5_parity_report.md

   db6c09b..5a862bd  jazzy -> jazzy
```

### Step 4 — R17 handoff

80 lines written to `.claude_handoff/round_17_motion_mode_investigation.md`.

### Manual verification steps for the operator

The README's new "## Parity with real robot" section is
visible at <https://github.com/S-abk/ranger_ros2/blob/jazzy/ranger_mini_v3_sim/README.md>
(after this round's push). It links readers at the parity
report for full detail.

## Deviations

- **Wrote round_18 handoff in addition to round_17.** The
  R18 prompt's step 4 only explicitly creates the round_17
  handoff (capturing R17's investigation retroactively), but
  the closing line says "Upload round_18_phase5_close.md."
  Wrote both for consistency with the per-round handoff
  pattern: round_17 documents R17's data, round_18 documents
  this round's documentation work.

## Open questions

Phase 5 is closed. No outstanding bugs; sim is functional
end-to-end.

Three optional next moves:
1. **Phase 6 (polish)** — top-level repo README pointing at
   the sim sub-README, sensor xacros (lidar / camera /
   IMU plugin tags), additional worlds (warehouse,
   outdoor terrain).
2. **Implement the documented gap** — `pulse_count`
   integration if anyone wants encoder-fusion testing
   against the sim.
3. **Stop and use the sim.** It's done.

No outstanding bugs. The R10/R11 /clock QoS hardening,
R13 launch hierarchy README, R16 mock-value alignments,
and R17 motion_mode validation all add up to a solid base.
