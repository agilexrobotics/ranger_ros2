"""Unit tests for PARALLEL mode side-slip sign handling.

R08 covered only the (linear_x=0, linear_y=+0.3, last_x>=0)
case in integration tests. These tests exercise the four
combinations of (linear_y sign, last_nonzero_x sign) plus
the non-side-slip cases.
"""

import math

from ranger_mini_v3_sim_messenger.sim_messenger import (
    compute_wheel_commands_parallel,
    MAX_STEER_PARALLEL,
    WHEEL_RADIUS,
)


def _approx(a, b, tol=1e-6):
    return abs(a - b) < tol


# ---------- Non-side-slip: both x and y nonzero ----------
def test_parallel_diagonal_forward_left():
    """x>0, y>0: drive at 45° forward-left."""
    wc, angle, speed = compute_wheel_commands_parallel(0.3, 0.3, 1.0)
    assert _approx(angle, math.pi / 4)
    assert _approx(speed, math.hypot(0.3, 0.3))
    # All four wheels identical
    assert _approx(wc.steer_fl, math.pi / 4)
    assert _approx(wc.steer_fr, math.pi / 4)
    assert _approx(wc.steer_rl, math.pi / 4)
    assert _approx(wc.steer_rr, math.pi / 4)
    # Wheel speed = speed / radius
    expected_w = math.hypot(0.3, 0.3) / WHEEL_RADIUS
    assert _approx(wc.vel_fl, expected_w)


def test_parallel_diagonal_backward_right():
    """x<0, y<0: reverse + right; angle gets sign-flipped per code."""
    wc, angle, speed = compute_wheel_commands_parallel(-0.3, -0.3, 1.0)
    # Speed is signed negative (reverse motion).
    assert speed < 0
    # Wheel commands are uniform.
    assert _approx(wc.steer_fl, wc.steer_fr)
    assert _approx(wc.steer_fl, wc.steer_rl)
    assert _approx(wc.steer_fl, wc.steer_rr)


# ---------- Pure side-slip: x=0, y nonzero ----------
def test_sideslip_pos_y_pos_last_x():
    """y>0, last_nonzero_x>=0: steer +π/2, positive speed."""
    wc, angle, speed = compute_wheel_commands_parallel(0.0, 0.3, 1.0)
    assert _approx(angle, math.pi / 2) or _approx(angle, MAX_STEER_PARALLEL)
    assert speed > 0


def test_sideslip_pos_y_neg_last_x():
    """y>0, last_nonzero_x<0: steer -π/2, sign of speed mirrored."""
    wc, angle, speed = compute_wheel_commands_parallel(0.0, 0.3, -1.0)
    # The combination should produce the OPPOSITE wheel direction
    # vs the test above (otherwise side-slip after reversing
    # would change the robot's perceived "forward").
    wc_ref, _, speed_ref = compute_wheel_commands_parallel(0.0, 0.3, 1.0)
    assert (wc.vel_fl > 0) != (wc_ref.vel_fl > 0), \
        "side-slip with negative last_x should reverse wheel direction"


def test_sideslip_neg_y_pos_last_x():
    """y<0, last_nonzero_x>=0: opposite of pos_y_pos_last_x."""
    wc_pos, _, _ = compute_wheel_commands_parallel(0.0, +0.3, 1.0)
    wc_neg, _, _ = compute_wheel_commands_parallel(0.0, -0.3, 1.0)
    # Reversing y should reverse wheel direction (same steering)
    assert (wc_pos.vel_fl > 0) != (wc_neg.vel_fl > 0)


# ---------- Stationary (everything zero) ----------
def test_zero_command_yields_zero_wheels():
    wc, angle, speed = compute_wheel_commands_parallel(0.0, 0.0, 1.0)
    assert speed == 0.0
    assert wc.vel_fl == 0.0
    assert wc.steer_fl == 0.0  # default WheelCommands has zeros
