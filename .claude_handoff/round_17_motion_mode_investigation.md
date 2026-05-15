# Round 17 — motion_mode continuous capture investigation

## Summary

Operator ran a custom capture script on the robot laptop that
runs continuous `ros2 topic echo /motion_state` and
`/system_state` during sustained cmd_vel commands. Four
patterns: forward, arc_left, spin, tight_curve.

Result: real driver DOES switch motion_mode (0 → 2 → 0) for
spin and tight_curve. R15b's snapshot-only capture missed
the active window. Sim's auto-switching logic was correct
all along.

Secondary finding: real driver publishes the same broadcast
steering value across all 8 actuator_state entries (not
per-wheel). Sim is more informative; preserved as a
deliberate sim improvement.

See `.claude_handoff/phase5_parity_report.md` for the full
audit report.

## Diff

No code changes this round. R18 committed two
documentation files (parity report + README parity
section).

## New files (full content)

### Capture script (used once, not landed as infrastructure)

Stored only in this handoff for reference; not in the
package tree. To run again: copy from this handoff to the
robot laptop, set DOMAIN=45 + source workspace, run.

### .claude_handoff/round_17_motion_mode_investigation.md

This file.

## Verification output

### motion_mode transition counts (R17 capture)

| Pattern        | Distinct motion_mode values observed |
|----------------|--------------------------------------|
| forward        | {0} — never switched (correct for radius=∞) |
| arc_left       | {0} — never switched (radius=1.0m > 0.476m) |
| spin           | {0, 2} — switched 0→2 at msg 53, back at msg 304 |
| tight_curve    | {0, 2} — switched 0→2 at msg 53, back at msg 303 |

~250 messages of motion_mode=2 during each angular command,
matching the 5-second command duration at /motion_state's
~50 Hz publish rate.

### actuator_state broadcast value (during command)

| Pattern        | All 8 motor_angles = | Math match |
|----------------|----------------------|------------|
| forward        | 0.000 rad            | ✓ wheels straight |
| arc_left       | 0.179 rad            | ✓ bicycle central angle, atan(W·sin(φᵢ)/(W·cos(φᵢ)+T·sin(φᵢ))) for inner=0.244 rad |
| spin           | 0.935 rad            | ✓ tangent magnitude atan2(W/2, T/2) = 0.9358 |
| tight_curve    | 0.935 rad            | Same as spin (mode-2 kinematics) |

Real driver's kinematic math matches sim's exactly; only
the per-wheel reporting differs.

## Deviations

None. R17 was data collection only; no code modified.

## Open questions

Phase 5 is now complete. Optional next moves:
- Phase 6 (polish): top-level repo README, sensor xacros,
  additional worlds. Pure quality-of-life work; not
  required for the sim to be useful.
- Stop here and use the sim.

No outstanding bugs.
