"""Ranger Mini v3 sim messenger.

Mirrors the ROS interface of ranger_base/src/ranger_messenger.cpp.
Subscribes to /cmd_vel and publishes /odom + per-wheel controller
commands. Round 07 implements DUAL_ACKERMAN only; other modes
set the mode and warn.

Kinematic constants come from RangerMiniV3Params in the real
driver's ranger_params.hpp.
"""

from dataclasses import dataclass
from enum import IntEnum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from ranger_msgs.msg import (
    SystemState, MotionState,
    ActuatorState, ActuatorStateArray,
    DriverState, MotorState,
)


# ============================================================
# Constants — from RangerMiniV3Params (real-driver source of truth)
# ============================================================
WHEELBASE   = 0.494                       # m
TRACK       = 0.364                       # m
WHEEL_RADIUS = 0.09                       # m  (from URDF; not in params)
MAX_LINEAR_SPEED = 1.5                    # m/s
MAX_ANGULAR_SPEED = 4.8                   # rad/s
MIN_TURN_RADIUS = 0.4764                  # m
MAX_STEER_ACKERMANN = 0.601               # rad
MAX_STEER_PARALLEL  = 1.570               # rad


# ============================================================
# State-mock constants (sim defaults; real values come from
# ranger_base/src/ranger_messenger.cpp populated from CAN.)
# ============================================================
SIM_BATTERY_VOLTAGE = 24.0       # V (typical 24V Li-ion pack)
SIM_BATTERY_CURRENT = -1.0       # A (negative = discharging)
SIM_BATTERY_TEMP    = 25.0       # °C
SIM_BATTERY_SOC     = 1.0        # fraction (0..1)
SIM_DRIVER_VOLTAGE  = 24.0       # V (driver bus = battery)
SIM_DRIVER_TEMP     = 35.0       # °C (warm operating)
SIM_MOTOR_TEMP      = 40.0       # °C
SIM_DRIVER_STATE_OK = 0          # 0 = no faults
# Actuator-id mapping (matches real driver's ordering of the
# 8 actuators in ActuatorStateArray; the real CAN frames put
# 4 steering then 4 drive). The map is sim-specific because
# we read from /joint_states; verify against real-robot
# ordering when commissioning.
ACTUATOR_INDEX = {
    # ID 0-3: steering joints
    0: 'fl_steering_joint',
    1: 'fr_steering_joint',
    2: 'rl_steering_joint',
    3: 'rr_steering_joint',
    # ID 4-7: drive wheels
    4: 'fl_wheel',
    5: 'fr_wheel',
    6: 'rl_wheel',
    7: 'rr_wheel',
}


class MotionMode(IntEnum):
    """Mirrors ranger_msgs/msg/MotionState constants."""
    DUAL_ACKERMAN = 0
    PARALLEL      = 1
    SPINNING      = 2
    SIDE_SLIP     = 3


@dataclass
class WheelCommands:
    """Eight scalars: 4 steering angles + 4 wheel velocities.
    Order: fl, fr, rl, rr.
    """
    steer_fl: float = 0.0
    steer_fr: float = 0.0
    steer_rl: float = 0.0
    steer_rr: float = 0.0
    vel_fl: float = 0.0
    vel_fr: float = 0.0
    vel_rl: float = 0.0
    vel_rr: float = 0.0


# ============================================================
# Kinematic helpers (ported from ranger_messenger.cpp)
# ============================================================
def calculate_steering_angle(linear_x: float, angular_z: float):
    """Port of CalculateSteeringAngle.

    Returns (inner_wheel_angle, turn_radius).
    Sign: positive inner-wheel angle = left turn for forward motion.
    """
    lin = abs(linear_x)
    ang = abs(angular_z)
    if ang < 1e-6:
        return 0.0, math.inf
    if lin < 1e-6:
        # Pure spin (no linear): radius 0 forces SPINNING mode upstream.
        # Matches smalleha's div-by-zero guard in the real driver.
        return 0.0, 0.0
    radius = lin / ang
    k = 1 if (angular_z * linear_x) >= 0 else -1
    phi_i = math.atan((WHEELBASE / 2.0) / radius)
    phi_i = min(phi_i, math.radians(40.0))
    return k * phi_i, radius


def inner_to_central(angle: float):
    """Port of ConvertInnerAngleToCentral.

    Translates an inner-wheel Ackermann angle to the equivalent
    bicycle-model central angle (used by the wheelbase-only
    forward-kinematics model).
    """
    phi_i = abs(angle)
    phi = math.atan(
        WHEELBASE * math.sin(phi_i) /
        (WHEELBASE * math.cos(phi_i) + TRACK * math.sin(phi_i))
    )
    return phi if angle >= 0 else -phi


def per_wheel_steering_dual_ackermann(inner_phi: float):
    """Compute the four wheel steering angles in DUAL_ACKERMAN.

    Geometry: front and rear axles mirror each other; left and
    right wheels on each axle are NOT the same (Ackermann
    geometry). The 'inner' wheel (inside the turn) is at the
    larger angle.

    Returns (fl, fr, rl, rr) in radians, each per the URDF
    steering joint axis convention (axis (0,0,-1)).
    """
    if abs(inner_phi) < 1e-6:
        return 0.0, 0.0, 0.0, 0.0

    # k > 0: turning left (positive z rotation of the vehicle).
    # Inside wheels are LEFT side. Outside wheels are RIGHT side.
    sign = 1.0 if inner_phi > 0 else -1.0
    phi_i = abs(inner_phi)

    # Radius from the vehicle center to its instantaneous center of
    # rotation (ICR), computed from the inner angle. The dual-
    # Ackermann assumption is that the ICR sits on the y axis at
    # the vehicle center; both axles steer toward it symmetrically.
    R = (WHEELBASE / 2.0) / math.tan(phi_i)

    # Outer wheel angle: larger turn radius, smaller angle.
    phi_o = math.atan((WHEELBASE / 2.0) / (R + TRACK))

    # Assign: inside (left) wheels get phi_i, outside (right) get phi_o.
    # Front axle: positive steer; rear axle: negative steer (mirror).
    if sign > 0:
        fl, fr, rl, rr = phi_i, phi_o, -phi_i, -phi_o
    else:
        fl, fr, rl, rr = -phi_o, -phi_i, phi_o, phi_i

    return fl, fr, rl, rr


def compute_wheel_commands_dual_ackermann(linear_x: float, inner_phi: float):
    """Compute all 8 commands for DUAL_ACKERMAN mode.

    For wheel velocities Round 07 uses the simplification of
    commanding all four wheels to the same angular velocity:
      w = linear_x / wheel_radius
    This is correct in straight lines; in tight turns each wheel
    should rotate at a slightly different rate. Round 08 may
    refine if visible slip becomes a problem.
    """
    fl_s, fr_s, rl_s, rr_s = per_wheel_steering_dual_ackermann(inner_phi)
    w = linear_x / WHEEL_RADIUS

    return WheelCommands(
        steer_fl=fl_s, steer_fr=fr_s, steer_rl=rl_s, steer_rr=rr_s,
        vel_fl=w, vel_fr=w, vel_rl=w, vel_rr=w,
    )


def compute_wheel_commands_parallel(linear_x: float, linear_y: float,
                                    last_nonzero_x: float):
    """Compute all 8 commands for PARALLEL mode.

    All 4 wheels point in the same direction, common angle =
    atan2(linear_y, linear_x). When linear_x == 0 (pure
    side-slip), the angle's sign is taken relative to the LAST
    nonzero linear_x so the robot keeps moving in its
    "established" forward direction.

    Returns (WheelCommands, used_angle, used_speed) for
    optional debug logging.
    """
    # Direction & magnitude
    if linear_x == 0.0 and linear_y == 0.0:
        return WheelCommands(), 0.0, 0.0

    if linear_x == 0.0:
        # Pure side-slip case.
        # Sign convention: see ranger_messenger.cpp L437-461.
        # steer_cmd = atan(y/x) is undefined at x=0; the real driver
        # uses |atan(y/x)| with sign = sign(last_nonzero_x).
        # Effective velocity sign comes from linear_y.
        steer_cmd = math.atan(linear_y / 1e-9 if linear_y > 0 else
                               -linear_y / 1e-9)  # +pi/2 or -pi/2
        # Better, exact: pi/2 for sideways drive, signed by last x
        steer_cmd = math.pi / 2.0
        if last_nonzero_x < 0:
            steer_cmd = -steer_cmd
        speed = abs(linear_y)
        if linear_y < 0:
            speed = -speed if last_nonzero_x >= 0 else speed
        else:
            speed = speed if last_nonzero_x >= 0 else -speed
    else:
        # Standard parallel: angle from atan2, magnitude from hypot.
        steer_cmd = math.atan2(linear_y, linear_x)
        if linear_x < 0:
            steer_cmd = -steer_cmd  # mirror sign for reverse
        vmag = math.hypot(linear_x, linear_y)
        speed = vmag if linear_x >= 0 else -vmag

    # Clamp to parallel-mode max angle
    steer_cmd = max(-MAX_STEER_PARALLEL,
                    min(MAX_STEER_PARALLEL, steer_cmd))

    # All 4 wheels point the same way; all 4 spin at the same rate
    w = speed / WHEEL_RADIUS
    return WheelCommands(
        steer_fl=steer_cmd, steer_fr=steer_cmd,
        steer_rl=steer_cmd, steer_rr=steer_cmd,
        vel_fl=w, vel_fr=w, vel_rl=w, vel_rr=w,
    ), steer_cmd, speed


# ============================================================
# SPINNING-mode geometry (computed once at module load)
# ============================================================
_W = WHEELBASE
_T = TRACK
_SPIN_RADIUS = math.hypot(_W / 2.0, _T / 2.0)


def _wrap_into_steer_range(angle: float):
    """Wrap angle to (-pi, pi], then if outside ±MAX_STEER_PARALLEL,
    flip by π (which reverses wheel direction — physically
    equivalent for a continuous wheel)."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    flipped = False
    if angle > MAX_STEER_PARALLEL:
        angle -= math.pi
        flipped = True
    elif angle < -MAX_STEER_PARALLEL:
        angle += math.pi
        flipped = True
    return angle, flipped


# Tangent-direction steering angles for CCW spin at each wheel.
# The radial vector from origin to each wheel:
#   fl: ( W/2, +T/2)   fr: ( W/2, -T/2)
#   rl: (-W/2, +T/2)   rr: (-W/2, -T/2)
# Tangent (90° CCW of radial) = (-radial.y, radial.x):
#   fl tangent: (-T/2,  W/2)
#   fr tangent: ( T/2,  W/2)
#   rl tangent: (-T/2, -W/2)
#   rr tangent: ( T/2, -W/2)
# Angle = atan2(tangent.y, tangent.x), then wrap into joint range.
_FL_TAN = math.atan2(_W / 2.0, -_T / 2.0)
_FR_TAN = math.atan2(_W / 2.0,  _T / 2.0)
_RL_TAN = math.atan2(-_W / 2.0, -_T / 2.0)
_RR_TAN = math.atan2(-_W / 2.0,  _T / 2.0)
_FL_STEER, _FL_FLIPPED = _wrap_into_steer_range(_FL_TAN)
_FR_STEER, _FR_FLIPPED = _wrap_into_steer_range(_FR_TAN)
_RL_STEER, _RL_FLIPPED = _wrap_into_steer_range(_RL_TAN)
_RR_STEER, _RR_FLIPPED = _wrap_into_steer_range(_RR_TAN)


def compute_wheel_commands_spinning(angular_z: float):
    """Compute all 8 commands for SPINNING mode.

    All 4 wheels point tangent to a circle centered at the
    vehicle origin; each spins at the angular velocity required
    to make the body yaw at the commanded angular_z.
    """
    # Clamp body angular velocity
    w_body = max(-MAX_ANGULAR_SPEED,
                 min(MAX_ANGULAR_SPEED, angular_z))

    # Wheel angular velocity: linear ground speed = w_body * spin_radius;
    # wheel angular velocity = ground_speed / wheel_radius.
    wheel_speed = (w_body * _SPIN_RADIUS) / WHEEL_RADIUS

    # If a wheel's steering was flipped by π during range-wrap,
    # its physical "forward" direction reversed, so its velocity
    # commanded must also flip.
    return WheelCommands(
        steer_fl=_FL_STEER, steer_fr=_FR_STEER,
        steer_rl=_RL_STEER, steer_rr=_RR_STEER,
        vel_fl=(-wheel_speed if _FL_FLIPPED else wheel_speed),
        vel_fr=(-wheel_speed if _FR_FLIPPED else wheel_speed),
        vel_rl=(-wheel_speed if _RL_FLIPPED else wheel_speed),
        vel_rr=(-wheel_speed if _RR_FLIPPED else wheel_speed),
    )


# ============================================================
# The node
# ============================================================
class SimMessenger(Node):
    """Twist-to-controllers messenger for the Ranger Mini v3 sim."""

    def __init__(self):
        super().__init__('sim_messenger')

        # Parameters
        self.declare_parameter('update_rate', 50)        # Hz
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic_name', 'odom')
        self.declare_parameter('publish_odom_tf', False)

        self.update_rate = self.get_parameter('update_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic_name').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value

        # State
        self.motion_mode = MotionMode.DUAL_ACKERMAN
        self.last_twist = Twist()
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.last_inner_phi = 0.0   # remember for odometry integration
        # dt diagnostic counters
        self._tick_count = 0
        self._dt_sum = 0.0
        self._dt_max = 0.0
        self._dt_min = float('inf')
        self.last_nonzero_x = 1.0   # for parallel side-slip sign
        self._last_used_angle = 0.0     # parallel mode odom
        self._last_used_speed = 0.0     # parallel mode odom
        self._last_used_angular_z = 0.0 # spinning mode odom

        # QoS: BestEffort for /cmd_vel (matches typical teleop pubs),
        # Reliable for /odom (downstream usually needs every sample).
        cmd_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Pub/Sub
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, cmd_qos
        )
        self.odom_pub = self.create_publisher(
            Odometry, self.odom_topic, odom_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None

        # Subscribe to /joint_states so ActuatorStateArray can carry
        # realistic motor_angles/motor_speeds. Default ros2_control
        # publishes /joint_states at the controller manager rate.
        self._joint_state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.last_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb,
            self._joint_state_qos,
        )

        # The 4 state publishers — match real driver's topic names
        # and types exactly (ranger_base/src/ranger_messenger.cpp).
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.system_state_pub = self.create_publisher(
            SystemState, '/system_state', state_qos
        )
        self.motion_state_pub = self.create_publisher(
            MotionState, '/motion_state', state_qos
        )
        self.actuator_state_pub = self.create_publisher(
            ActuatorStateArray, '/actuator_state', state_qos
        )
        self.battery_state_pub = self.create_publisher(
            BatteryState, '/battery_state', state_qos
        )

        # Controller command publishers — one per joint.
        # Each controller takes Float64MultiArray with one element
        # (its single joint, per controllers.yaml).
        self.steer_pubs = {
            'fl': self.create_publisher(Float64MultiArray, '/fl_steering_position_controller/commands', 10),
            'fr': self.create_publisher(Float64MultiArray, '/fr_steering_position_controller/commands', 10),
            'rl': self.create_publisher(Float64MultiArray, '/rl_steering_position_controller/commands', 10),
            'rr': self.create_publisher(Float64MultiArray, '/rr_steering_position_controller/commands', 10),
        }
        self.wheel_pubs = {
            'fl': self.create_publisher(Float64MultiArray, '/fl_wheel_velocity_controller/commands', 10),
            'fr': self.create_publisher(Float64MultiArray, '/fr_wheel_velocity_controller/commands', 10),
            'rl': self.create_publisher(Float64MultiArray, '/rl_wheel_velocity_controller/commands', 10),
            'rr': self.create_publisher(Float64MultiArray, '/rr_wheel_velocity_controller/commands', 10),
        }

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self._tick)

        self.get_logger().info(
            f'SimMessenger up. update_rate={self.update_rate} Hz, '
            f'publish_odom_tf={self.publish_odom_tf}'
        )

    # --------------------------------------------------------
    # Twist callback: only updates mode + stores latest Twist
    # --------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        self.last_twist = msg

        if msg.linear.x != 0.0:
            self.last_nonzero_x = msg.linear.x

        # Mode selection — port of TwistCmdCallback (lines 388-414)
        # NOTE: the v1-side-slip branch is skipped (we're v3-only).
        if msg.linear.y != 0.0:
            self.motion_mode = MotionMode.PARALLEL
        else:
            _, radius = calculate_steering_angle(msg.linear.x, msg.angular.z)
            if radius < MIN_TURN_RADIUS:
                self.motion_mode = MotionMode.SPINNING
            else:
                self.motion_mode = MotionMode.DUAL_ACKERMAN

    # --------------------------------------------------------
    # Timer tick: compute & publish wheel commands + odometry
    # --------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        self._tick_count += 1
        self._dt_sum += dt
        if dt > self._dt_max:
            self._dt_max = dt
        if dt < self._dt_min:
            self._dt_min = dt
        if self._tick_count % 50 == 0:
            mean_dt = self._dt_sum / 50.0
            self.get_logger().info(
                f"tick_diag: count={self._tick_count} "
                f"mean_dt={mean_dt:.4f}s "
                f"min_dt={self._dt_min:.4f}s "
                f"max_dt={self._dt_max:.4f}s "
                f"sim_time={now.nanoseconds*1e-9:.3f}s"
            )
            self._dt_sum = 0.0
            self._dt_max = 0.0
            self._dt_min = float('inf')

        msg = self.last_twist
        wc = WheelCommands()    # default zeros

        if self.motion_mode == MotionMode.DUAL_ACKERMAN:
            # Compute steering and wheel velocity.
            inner_phi, _radius = calculate_steering_angle(
                msg.linear.x, msg.angular.z
            )
            # Clamp to max Ackermann angle (per ranger_messenger.cpp L419)
            inner_phi = max(-MAX_STEER_ACKERMANN,
                            min(MAX_STEER_ACKERMANN, inner_phi))
            self.last_inner_phi = inner_phi
            wc = compute_wheel_commands_dual_ackermann(msg.linear.x, inner_phi)

            # Odometry integration: bicycle-model with central angle.
            central = inner_to_central(inner_phi)
            v = msg.linear.x
            # RK4 step (10 substeps for accuracy)
            self._integrate_dual_ackermann(v, central, dt)

        elif self.motion_mode == MotionMode.PARALLEL:
            wc, used_angle, used_speed = compute_wheel_commands_parallel(
                msg.linear.x, msg.linear.y, self.last_nonzero_x
            )
            self._integrate_parallel(used_speed, used_angle, dt)
            self._last_used_angle = used_angle
            self._last_used_speed = used_speed
            self.last_inner_phi = 0.0

        elif self.motion_mode == MotionMode.SPINNING:
            w = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, msg.angular.z))
            wc = compute_wheel_commands_spinning(w)
            # SpinningModel: x,y unchanged; theta += w*dt
            self.theta += w * dt
            self._last_used_angular_z = w
            self.last_inner_phi = 0.0

        else:
            wc = WheelCommands()  # zeros
            self.last_inner_phi = 0.0

        self._publish_wheel_commands(wc)
        self._publish_odometry(msg, now)
        self._publish_state_topics(now)

    # --------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        self.last_joint_state = msg

    # --------------------------------------------------------
    def _integrate_dual_ackermann(self, v: float, phi: float, dt: float):
        """RK4 of DualAckermanModel from kinematics_model.hpp.

        State: (x, y, theta). Control: (v, phi) where phi is the
        central (bicycle) angle.
        """
        def f(state, _t):
            x, y, th = state
            return [
                v * math.cos(phi) * math.cos(th),
                v * math.cos(phi) * math.sin(th),
                2.0 * v * math.sin(phi) / WHEELBASE,
            ]

        state = [self.position_x, self.position_y, self.theta]
        # 10 RK4 substeps
        h = dt / 10.0
        t = 0.0
        for _ in range(10):
            k1 = f(state, t)
            s2 = [state[i] + 0.5 * h * k1[i] for i in range(3)]
            k2 = f(s2, t + 0.5 * h)
            s3 = [state[i] + 0.5 * h * k2[i] for i in range(3)]
            k3 = f(s3, t + 0.5 * h)
            s4 = [state[i] + h * k3[i] for i in range(3)]
            k4 = f(s4, t + h)
            state = [
                state[i] + (h / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
                for i in range(3)
            ]
            t += h
        self.position_x, self.position_y, self.theta = state

    # --------------------------------------------------------
    def _integrate_parallel(self, v: float, phi: float, dt: float):
        """RK4 of ParallelModel from kinematics_model.hpp.

        State: (x, y, theta). Control: (v, phi).
        For parallel/side-slip: vehicle translates at angle (theta+phi),
        yaw unchanged.
        """
        def f(state, _t):
            x, y, th = state
            return [
                v * math.cos(th + phi),
                v * math.sin(th + phi),
                0.0,
            ]
        state = [self.position_x, self.position_y, self.theta]
        h = dt / 10.0
        t = 0.0
        for _ in range(10):
            k1 = f(state, t)
            s2 = [state[i] + 0.5 * h * k1[i] for i in range(3)]
            k2 = f(s2, t + 0.5 * h)
            s3 = [state[i] + 0.5 * h * k2[i] for i in range(3)]
            k3 = f(s3, t + 0.5 * h)
            s4 = [state[i] + h * k3[i] for i in range(3)]
            k4 = f(s4, t + h)
            state = [
                state[i] + (h / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
                for i in range(3)
            ]
            t += h
        self.position_x, self.position_y, self.theta = state

    # --------------------------------------------------------
    def _publish_wheel_commands(self, wc: WheelCommands):
        def pub_one(p, value):
            m = Float64MultiArray()
            m.data = [float(value)]
            p.publish(m)

        pub_one(self.steer_pubs['fl'], wc.steer_fl)
        pub_one(self.steer_pubs['fr'], wc.steer_fr)
        pub_one(self.steer_pubs['rl'], wc.steer_rl)
        pub_one(self.steer_pubs['rr'], wc.steer_rr)

        pub_one(self.wheel_pubs['fl'], wc.vel_fl)
        pub_one(self.wheel_pubs['fr'], wc.vel_fr)
        pub_one(self.wheel_pubs['rl'], wc.vel_rl)
        pub_one(self.wheel_pubs['rr'], wc.vel_rr)

    # --------------------------------------------------------
    def _publish_odometry(self, last_cmd: Twist, now):
        quat = self._yaw_to_quat(self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        # Twist on /odom matches the real driver's convention:
        # publish the COMMANDED twist, not measured wheel-state.
        if self.motion_mode == MotionMode.DUAL_ACKERMAN:
            central = inner_to_central(self.last_inner_phi)
            odom.twist.twist.linear.x = last_cmd.linear.x
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = (
                2.0 * last_cmd.linear.x * math.sin(central) / WHEELBASE
            )
        elif self.motion_mode == MotionMode.PARALLEL:
            phi = self._last_used_angle
            speed = self._last_used_speed
            odom.twist.twist.linear.x = speed * math.cos(phi)
            odom.twist.twist.linear.y = speed * math.sin(phi)
            odom.twist.twist.angular.z = 0.0
        elif self.motion_mode == MotionMode.SPINNING:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self._last_used_angular_z
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id  = self.base_frame
            tf.transform.translation.x = self.position_x
            tf.transform.translation.y = self.position_y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf)

    # --------------------------------------------------------
    def _publish_state_topics(self, now):
        """Publish /system_state, /motion_state, /actuator_state,
        /battery_state to match the real-driver interface.

        Static defaults for fields the sim doesn't model
        (temperatures, voltages, currents). motor_angles and
        motor_speeds in ActuatorStateArray come from /joint_states
        when available (more useful for consumers); zero otherwise.
        """
        stamp = now.to_msg()

        # SystemState
        sys_msg = SystemState()
        sys_msg.header.stamp = stamp
        sys_msg.vehicle_state = SystemState.VEHICLE_STATE_NORMAL
        sys_msg.control_mode  = SystemState.CONTROL_MODE_CAN
        sys_msg.error_code    = 0
        sys_msg.battery_voltage = SIM_BATTERY_VOLTAGE
        sys_msg.motion_mode   = int(self.motion_mode)
        self.system_state_pub.publish(sys_msg)

        # MotionState
        mot_msg = MotionState()
        mot_msg.header.stamp = stamp
        mot_msg.motion_mode  = int(self.motion_mode)
        self.motion_state_pub.publish(mot_msg)

        # ActuatorStateArray (8 actuators)
        act_msg = ActuatorStateArray()
        act_msg.header.stamp = stamp

        # Build a name->(pos, vel) map from the last joint state
        joint_map = {}
        if self.last_joint_state is not None:
            js = self.last_joint_state
            for i, name in enumerate(js.name):
                pos = js.position[i] if i < len(js.position) else 0.0
                vel = js.velocity[i] if i < len(js.velocity) else 0.0
                joint_map[name] = (pos, vel)

        for actuator_id in range(8):
            joint_name = ACTUATOR_INDEX[actuator_id]
            pos, vel = joint_map.get(joint_name, (0.0, 0.0))

            driver = DriverState()
            driver.driver_voltage     = SIM_DRIVER_VOLTAGE
            driver.driver_temperature = SIM_DRIVER_TEMP
            driver.motor_temperature  = SIM_MOTOR_TEMP
            driver.driver_state       = SIM_DRIVER_STATE_OK

            motor = MotorState()
            motor.rpm           = int(vel * 60.0 / (2.0 * math.pi))
            motor.current       = 0.0
            motor.pulse_count   = 0
            motor.motor_angles  = float(pos)
            motor.motor_speeds  = float(vel)

            state = ActuatorState()
            state.id     = actuator_id
            state.driver = driver
            state.motor  = motor
            act_msg.states.append(state)

        self.actuator_state_pub.publish(act_msg)

        # BatteryState
        batt = BatteryState()
        batt.header.stamp = stamp
        batt.voltage      = SIM_BATTERY_VOLTAGE
        batt.temperature  = SIM_BATTERY_TEMP
        batt.current      = SIM_BATTERY_CURRENT
        batt.percentage   = SIM_BATTERY_SOC
        batt.charge          = float('nan')
        batt.capacity        = float('nan')
        batt.design_capacity = float('nan')
        batt.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        )
        batt.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )
        batt.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        )
        batt.present = True   # the real driver sets NaN here but
                              # `present` is bool — interpret as
                              # "battery present in sim"
        self.battery_state_pub.publish(batt)

    # --------------------------------------------------------
    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SimMessenger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
