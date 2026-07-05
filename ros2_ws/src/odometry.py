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
  - ENCODER_CPR below is a *theoretical* value (7 PPR x 4x quadrature x
    100:1 gearbox = 2800), NOT yet empirically calibrated. Treat any
    drift/uncertainty numbers computed with it as provisional until a
    physical calibration (rotate output shaft N turns, compare to raw
    CNT delta) is done.
"""

import math

# ---- Physical / calibration constants ----
WHEEL_RADIUS_M = 0.025       # 50mm diameter / 2
CENTER_TO_WHEEL_M = 0.115    # R: center to wheel contact point
ENCODER_CPR = 2800           # THEORETICAL (7 PPR * 4x quad * 100 gear). TODO: calibrate.

# Motor index -> physical wheel position, matching ps2_drive_test.py's
# inverse kinematics convention:
#   fl = vx + vy + omega*R
#   fr = vx - vy - omega*R
#   rl = vx - vy + omega*R
#   rr = vx + vy - omega*R
MOTOR_MAP = {
    0: "fl",
    1: "fr",
    2: "rl",
    3: "rr",
}

# Per-motor encoder count sign correction. Flip an entry to -1 if that
# wheel's raw tick count increases when the wheel is actually spinning
# "backward" relative to the inverse-kinematics convention above (e.g.
# mirrored mounting). Verify with a pure-vy and pure-omega test drive
# BEFORE trusting odometry output -- do not guess these.
ENCODER_SIGN = {
    0: 1,
    1: 1,
    2: 1,
    3: 1,
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
        # inverse-kinematics equations -- see PROJECT_STATE.md):
        vx = (fl + fr + rl + rr) / 4.0
        vy = (fl - fr + rr - rl) / 4.0
        omega = (fl - fr - rr + rl) / (4.0 * CENTER_TO_WHEEL_M)

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
    # Minimal smoke test: pure +vy motion for one 20ms step, with all four
    # wheels ticking consistently with fl=+, fr=-, rl=-, rr=+ per the
    # inverse-kinematics convention above. This only checks the math is
    # internally consistent -- it is NOT a substitute for the real
    # ground-truth test (square path / 360 degree spin with tape-measure
    # verification) called out in PROJECT_STATE.md.
    odo = OdometryEstimator()
    odo.update({0: 0, 1: 0, 2: 0, 3: 0}, timestamp=0.0)
    result = odo.update({0: 280, 1: -280, 2: -280, 3: 280}, timestamp=0.02)
    x, y, theta, vx, vy, omega = result
    print(f"x={x:.4f} y={y:.4f} theta={theta:.4f} vx={vx:.4f} vy={vy:.4f} omega={omega:.4f}")
    assert abs(vx) < 1e-9 and abs(omega) < 1e-9 and vy > 0, "pure-vy sanity check failed"
    print("smoke test OK: pure +vy in, vx=0 and omega=0 out, as expected")
