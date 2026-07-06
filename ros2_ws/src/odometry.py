"""
odometry.py
X-drive forward kinematics + midpoint pose integration.

Consumes cumulative encoder tick counts (already 32-bit extended on the
STM32 side, overflow-safe) for the four wheels and produces:
  - body-frame velocities (vx, vy, omega)
  - accumulated world-frame pose (x, y, theta)

Design notes (see PROJECT_STATE.md decision log, 2026-07-03):
  - dt is computed from actual CAN frame arrival timestamps, not assumed
    fixed at 20ms, to avoid absorbing bus/scheduling jitter as bias.
  - Pose integration uses the midpoint method (heading evaluated at
    t + dt/2) rather than plain Euler, since heading error dominates
    translational error in accumulated odometry drift.
  - MOTOR_MAP / ENCODER_SIGN are pulled out as explicit config so wiring
    changes don't require touching the math.
  - ENCODER_CPR below was empirically calibrated 2026-07-04 (rotate each
    output shaft 10 turns by hand, average the raw CNT delta across all
    four wheels): measured 2779 vs theoretical 2800. ENCODER_SIGN was
    calibrated the same day (all four wheels count down on forward drive
    -> all -1).
"""

import math

# ---- Physical / calibration constants ----
WHEEL_RADIUS_M = 0.025       # 50mm diameter / 2
CENTER_TO_WHEEL_M = 0.115    # R: center to wheel contact point
ENCODER_CPR = 2779           # CALIBRATED 2026-07-04: 4-wheel avg of 10-rev hand turns
                             # (FL 2779.5, FR 2778.3, RL 2778.7, RR 2777.6 -> 2778.5).
                             # Theoretical was 2800 (7 PPR * 4x quad * 100 gear); measured
                             # ~0.8% lower, spread across wheels only ~0.07%.

# Encoder index -> physical wheel position.
#
# This map reflects the ENCODER wiring harness, which is INDEPENDENT of the
# motor/drive wiring. On the drive side (main.c MotorPosition enum,
# ps2_drive_test.py inverse kinematics) index 0/1/2/3 = fl/fr/rl/rr:
#   fl = vx + vy + omega*R
#   fr = vx - vy - omega*R
#   rl = vx - vy + omega*R
#   rr = vx + vy - omega*R
# but CALIBRATED 2026-07-06 (hand-spin each wheel, watch encoder_monitor):
# the REAR encoders are physically swapped -- encoder index 2 sits on the
# RR wheel and index 3 on the RL wheel. Symptom before the fix was vy and
# omega swapped (spin-in-place read as y, strafe read as omega) while vx
# was fine. So indices 2/3 below are DELIBERATELY the opposite of the drive
# order; do NOT "align" them back to the enum without re-checking the
# encoder harness.
MOTOR_MAP = {
    0: "fl",
    1: "fr",
    2: "rr",   # index 2 encoder is physically on the RR wheel
    3: "rl",   # index 3 encoder is physically on the RL wheel
}

# Per-motor encoder count sign correction. Flip an entry to -1 if that
# wheel's raw tick count increases when the wheel is actually spinning
# "backward" relative to the inverse-kinematics convention above (e.g.
# mirrored mounting).
#
# CALIBRATED 2026-07-04: driving straight forward (pure +vx, all four
# wheels command +) made ALL FOUR raw tick counts DECREASE, so all four
# are flipped to -1. NOTE: the pure-vx test fully determines these signs
# (each wheel must read + on forward) -- there is no remaining freedom to
# fix strafe/rotation via ENCODER_SIGN. vy (strafe) and omega (spin)
# direction correctness must still be confirmed by the ground-truth test
# (square path + 360 spin, Task 4). If those come out mirrored it's a
# geometry/wiring issue, NOT re-tunable here.
ENCODER_SIGN = {
    0: -1,
    1: -1,
    2: -1,
    3: -1,
}


def ticks_to_wheel_velocity(delta_ticks: int, dt: float) -> float:
    """Convert a tick delta over dt seconds into linear wheel velocity (m/s)."""
    if dt <= 0:
        return 0.0
    rev_per_sec = (delta_ticks / dt) / ENCODER_CPR
    return rev_per_sec * 2.0 * math.pi * WHEEL_RADIUS_M


class OdometryEstimator:
    """Stateful x-drive odometry: raw encoder ticks -> world-frame pose.

    Call update() once per received encoder CAN frame pair (all 4 motors'
    cumulative tick counts, plus the timestamp they were read at). This
    class knows nothing about CAN/ROS -- feed it plain ticks + timestamp,
    it hands back plain numbers. Wire it up inside can_bridge_node.py.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self._last_ticks = None       # dict[int, int], None until first sample
        self._last_timestamp = None   # float seconds, None until first sample

    def update(self, motor_ticks: dict, timestamp: float):
        """
        motor_ticks: {0: int, 1: int, 2: int, 3: int} cumulative ticks
        timestamp:   seconds (e.g. time.monotonic() at CAN frame arrival)

        Returns (x, y, theta, vx, vy, omega) after this update, or None on
        the very first call (nothing to differentiate against yet).
        """
        if self._last_ticks is None:
            self._last_ticks = dict(motor_ticks)
            self._last_timestamp = timestamp
            return None

        dt = timestamp - self._last_timestamp
        if dt <= 0:
            # Out-of-order or duplicate frame; skip rather than divide by
            # zero or integrate backward in time.
            return None

        wheel_vel = {}
        for motor_idx, position in MOTOR_MAP.items():
            delta = motor_ticks[motor_idx] - self._last_ticks[motor_idx]
            delta *= ENCODER_SIGN[motor_idx]
            wheel_vel[position] = ticks_to_wheel_velocity(delta, dt)

        fl, fr = wheel_vel["fl"], wheel_vel["fr"]
        rl, rr = wheel_vel["rl"], wheel_vel["rr"]

        # Forward kinematics (derived by summing/differencing the four
        # inverse-kinematics equations -- see PROJECT_STATE.md).
        #
        # vy and omega are NEGATED relative to the pure algebraic inverse of
        # the drive-side inverse kinematics. CALIBRATED 2026-07-06 (ground
        # truth, after the RL/RR encoder-map fix): with the plain inverse,
        # physical LEFT strafe read as -vy and physical CCW spin read as
        # -omega, while vx (forward) was already correct. That "vx right,
        # vy+omega both mirror-flipped" pattern is a left-right frame mirror:
        # the drive inverse-kinematics convention has +vy pointing right and
        # +omega going CW. ENCODER_SIGN is fully pinned by the forward test
        # and MOTOR_MAP is pinned by the physical encoder harness, so this
        # frame flip can only live here. Negating vy/omega aligns odometry
        # output to REP-103 (x fwd, y LEFT, z up / CCW positive). vx is left
        # untouched -- it was already REP-103-correct.
        vx = (fl + fr + rl + rr) / 4.0
        vy = -(fl - fr + rr - rl) / 4.0
        omega = -(fl - fr - rr + rl) / (4.0 * CENTER_TO_WHEEL_M)

        # Midpoint pose integration: rotate this step's body-frame
        # displacement by the heading at the *middle* of the interval,
        # not the start, since heading error dominates translational
        # error in accumulated odometry drift.
        theta_mid = self.theta + omega * dt / 2.0

        dx = (vx * math.cos(theta_mid) - vy * math.sin(theta_mid)) * dt
        dy = (vx * math.sin(theta_mid) + vy * math.cos(theta_mid)) * dt

        self.x += dx
        self.y += dy
        self.theta += omega * dt
        # Keep theta in (-pi, pi] for sane logging/plotting.
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self._last_ticks = dict(motor_ticks)
        self._last_timestamp = timestamp

        return self.x, self.y, self.theta, vx, vy, omega


if __name__ == "__main__":
    # Minimal smoke test: physical LEFT strafe for one 20ms step, expecting
    # REP-103 output vy > 0 (and no channel leak: vx = omega = 0).
    #
    # Under the DRIVE inverse kinematics, physical left strafe is the "-vy"
    # wheel pattern (fl=-, fr=+, rl=+, rr=-), because the drive convention's
    # +vy points right. The forward kinematics negates vy/omega to report
    # REP-103 (y = LEFT), so this pattern must come back as vy > 0. We build
    # the RAW tick input by inverting back through ENCODER_SIGN and MOTOR_MAP,
    # so the test stays correct no matter how those are wired (don't hardcode
    # per-index ticks -- that silently breaks when the map/sign changes).
    # This only checks the math is internally consistent -- it is NOT a
    # substitute for the real ground-truth test (square path / 360 degree
    # spin with tape-measure verification) called out in PROJECT_STATE.md.
    STEP = 280
    wheel_target = {"fl": -STEP, "fr": +STEP, "rl": +STEP, "rr": -STEP}
    raw = {i: ENCODER_SIGN[i] * wheel_target[MOTOR_MAP[i]] for i in MOTOR_MAP}

    odo = OdometryEstimator()
    odo.update({i: 0 for i in MOTOR_MAP}, timestamp=0.0)
    result = odo.update(raw, timestamp=0.02)
    x, y, theta, vx, vy, omega = result
    print(f"x={x:.4f} y={y:.4f} theta={theta:.4f} vx={vx:.4f} vy={vy:.4f} omega={omega:.4f}")
    assert abs(vx) < 1e-9 and abs(omega) < 1e-9 and vy > 0, "left-strafe sanity check failed"
    print("smoke test OK: physical left strafe in, REP-103 vy>0 and vx=omega=0 out")
