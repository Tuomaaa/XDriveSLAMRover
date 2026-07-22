# Week 6: MCL Particle Filter — Design Spec

## Goal

Implement Monte Carlo Localization (Prob Robotics Ch.8) for the X-drive rover
in a known rectangular environment, using encoder odometry for prediction and
two HC-SR04 ultrasonic sensors for measurement update.

## Modules

| File | Responsibility | Dependencies |
|------|---------------|--------------|
| `mcl.py` | Particle filter core (predict, update, estimate, resample) | `map_model.py`, `measurement_model.py`, numpy |
| `mcl_offline.py` | Offline replay: read CSV, run MCL, produce animation GIF | `mcl.py`, matplotlib |
| `mcl_node.py` | Online ROS2 node (phase 2, after offline validation) | `mcl.py`, rclpy |

All files live in `ros2_ws/src/` alongside existing modules.

## `mcl.py` — Core Algorithm

### Class: `MCL`

```python
MCL(
    rect_map: RectMap,
    beam_model: BeamModel,
    sensor_configs: list[tuple[float, tuple[float, float]]],
    num_particles: int = 500,
    alpha: tuple[float, float, float, float] = (0.05, 0.01, 0.01, 0.05),
    rng_seed: int | None = None,
)
```

**State:**
- `particles`: numpy array shape (N, 3) — each row is (x, y, theta)
- `weights`: numpy array shape (N,) — normalized to sum to 1

**Methods:**

`initialize(x, y, theta, xy_spread, theta_spread)` — scatter particles as
Gaussian cloud around a known starting pose. All particles get weight 1/N.

`predict(dx_body, dy_body, dtheta)` — for each particle:
1. Add proportional Gaussian noise to the body-frame delta:
   - `d_trans = sqrt(dx^2 + dy^2)`
   - `dx' = dx + N(0, alpha[0]*d_trans + alpha[1]*|dtheta|)`
   - `dy' = dy + N(0, alpha[0]*d_trans + alpha[1]*|dtheta|)`
   - `dtheta' = dtheta + N(0, alpha[2]*d_trans + alpha[3]*|dtheta|)`
2. Rotate (dx', dy') from body frame to world frame using particle's theta
3. Update particle pose: x += dx_world, y += dy_world, theta += dtheta'
4. Clamp particle inside map bounds (optional, prevents particles escaping walls)

`update(measurements: list[float | None])` — for each particle:
1. Compute expected range via `rect_map.expected_range(particle_pose, ...)`
2. Compute log-likelihood via `beam_model.total_log_likelihood(...)`
3. Convert to weight, normalize across all particles
4. Low-variance resample

`estimate() -> (x, y, theta)` — weighted mean of particles. Theta uses
circular mean: `atan2(sum(w*sin(theta)), sum(w*cos(theta)))`.

### Low-Variance Resampling

Standard Prob Robotics algorithm: draw one random offset r in [0, 1/N),
then walk through cumulative weight distribution stepping by 1/N. O(N),
deterministic given r, less variance than multinomial resampling.

### Smoke Test

`if __name__ == "__main__"`: create a RectMap, place a "robot" at a known
pose, generate synthetic odometry deltas + noisy range readings, run
predict/update for ~50 steps, assert estimate converges to within 5cm / 5deg
of the true pose.

## `mcl_offline.py` — Offline Replay

### Data Collection

A new script or mode in `can_bridge_node.py` logs to CSV while the robot
drives in the KT board enclosure:

```csv
timestamp_s,odom_x,odom_y,odom_theta,right_mm,back_mm,right_valid,back_valid
```

Each row is emitted whenever a complete encoder+ultrasonic update arrives.
The odometry pose comes from `OdometryEstimator`; ultrasonic fields come
from the 0x202 CAN frame.

### Replay Loop

```
for each row in CSV:
    dx, dy, dtheta = body_frame_delta(prev_odom, current_odom)
    mcl.predict(dx, dy, dtheta)
    if right_valid or back_valid:
        mcl.update([right_m if right_valid else None,
                    back_m if back_valid else None])
    record frame for animation
```

Body-frame delta: rotate world-frame displacement (odom_x - prev_x,
odom_y - prev_y) by -prev_theta to get body-frame (dx, dy).

### Animation Output

Matplotlib FuncAnimation saved as GIF:
- Gray rectangle = map boundary
- Blue dots = particles (alpha proportional to weight)
- Red arrow = MCL estimated pose
- Green arrow = raw odometry pose
- Two sensor rays from estimated pose to measured range
- Frame rate: ~10 fps, one frame per MCL step

## Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| num_particles | 500 | sufficient for 1.165m x 1.165m map |
| alpha | (0.05, 0.01, 0.01, 0.05) | Prob Robotics typical, tune with data |
| sigma_hit | (0.0017, 0.0078) | Week 5 calibration |
| w_hit/short/max/rand | 0.94/0.01/0.03/0.02 | Week 5 calibration |
| z_max | 4.0 | HC-SR04 spec |
| lambda_short | 0.5 | low weight, not critical |
| sensor_configs | [(-pi/2, (0,-.09)), (pi, (-.09,0))] | physical measurement |
| map | RectMap(0, 1.165, 0, 1.165) | KT board measured |

## Initialization

Known initial pose (car placed at measured position). Particles scattered
with xy_spread=0.05m, theta_spread=0.1rad. Global localization (uniform
over entire map) is a stretch goal.

## Testing Strategy

1. **Synthetic smoke test** (`mcl.py __main__`): deterministic RNG seed,
   simulated robot in known map, verify convergence
2. **Offline replay** (`mcl_offline.py`): real sensor data, visual inspection
   of particle convergence and MCL vs odometry divergence
3. **Sanity checks**: particles stay inside map, weights sum to 1, estimate
   moves in correct direction

## Phase 2: Online (`mcl_node.py`) — After Offline Validation

Subscribe to `odom` and `/ultrasonic/right` + `/ultrasonic/back` topics.
On each odom update: compute body-frame delta, call `predict()`.
On each ultrasonic update: call `update()`.
Publish estimated pose on `/mcl_pose` and particles on `/mcl_particles`.
Defer to after offline replay works end-to-end.

## Not In Scope

- Global localization (uniform initialization)
- KLD-sampling / adaptive particle count
- Multi-hypothesis tracking (kidnapped robot)
- Non-rectangular maps
- IMU fusion
