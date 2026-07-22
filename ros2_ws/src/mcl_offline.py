"""Replay a recorded MCL log through the particle filter and render a GIF.

The log's odometry lives in the odom frame, which starts at (0, 0, 0)
wherever the rover happened to be switched on. The filter never sees that
origin -- it consumes deltas only -- so the true starting pose in the map
has to be supplied on the command line. It is also what the dead-reckoning
trace is anchored to.

Usage:
    python3 mcl_offline.py /tmp/mcl_run1.csv --start 0.30 0.30 0 \\
        --output mcl_run1.gif
"""

import argparse
import math
from pathlib import Path

from mcl import MCL, SENSOR_CONFIGS, default_beam_model, default_map
from mcl_log import read_log


def body_frame_delta(previous, current):
    """World-frame pose pair -> the (dx, dy, dtheta) the robot felt.

    Rotating the world displacement by -previous_theta undoes the heading
    it was expressed in, leaving the step in the robot's own frame -- which
    is what MCL.predict wants.
    """
    dx_world = current[0] - previous[0]
    dy_world = current[1] - previous[1]
    cos_t = math.cos(previous[2])
    sin_t = math.sin(previous[2])
    dtheta = current[2] - previous[2]
    return (
        dx_world * cos_t + dy_world * sin_t,
        -dx_world * sin_t + dy_world * cos_t,
        math.atan2(math.sin(dtheta), math.cos(dtheta)),
    )


def odom_to_map(odom_pose, odom_origin, start_pose):
    """Express an odom-frame pose in map coordinates.

    The filter only ever consumes deltas, so the odom origin is arbitrary;
    this exists purely to draw the dead-reckoning trace beside the estimate.
    """
    dx, dy, dtheta = body_frame_delta(odom_origin, odom_pose)
    x0, y0, theta0 = start_pose
    heading = theta0 + dtheta
    return (
        x0 + dx * math.cos(theta0) - dy * math.sin(theta0),
        y0 + dx * math.sin(theta0) + dy * math.cos(theta0),
        math.atan2(math.sin(heading), math.cos(heading)),
    )


def replay(entries, mcl, start_pose):
    """Run the filter over a parsed log. Returns one frame dict per step."""
    mcl.initialize(*start_pose)
    origin = (entries[0]['odom_x'], entries[0]['odom_y'],
              entries[0]['odom_theta'])
    previous = origin
    frames = []

    for entry in entries[1:]:
        current = (entry['odom_x'], entry['odom_y'], entry['odom_theta'])
        measurements = [entry['right_m'], entry['back_m']]
        mcl.predict(*body_frame_delta(previous, current))
        mcl.update(measurements)
        previous = current

        frames.append({
            'particles': mcl.particles.copy(),
            'estimate': mcl.estimate(),
            'dead_reckoning': odom_to_map(current, origin, start_pose),
            'measurements': measurements,
            'timestamp': entry['timestamp_s'],
        })

    return frames


def render(frames, rect_map, output, fps=10, stride=2):
    """Animate particles, estimate, and dead reckoning into a GIF."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    shown = frames[::stride]
    figure, axes = plt.subplots(figsize=(6, 6))
    axes.set(
        xlim=(rect_map.x_min - 0.05, rect_map.x_max + 0.05),
        ylim=(rect_map.y_min - 0.05, rect_map.y_max + 0.05),
        xlabel='x (m)', ylabel='y (m)',
    )
    axes.set_aspect('equal')
    axes.plot(
        [rect_map.x_min, rect_map.x_max, rect_map.x_max,
         rect_map.x_min, rect_map.x_min],
        [rect_map.y_min, rect_map.y_min, rect_map.y_max,
         rect_map.y_max, rect_map.y_min],
        color='0.4', linewidth=2)

    cloud = axes.scatter([], [], s=3, alpha=0.25, color='tab:blue',
                         label='particles')
    estimate_dot, = axes.plot([], [], 'o', color='tab:red', markersize=7,
                              label='MCL estimate')
    estimate_heading, = axes.plot([], [], '-', color='tab:red', linewidth=2)
    dead_dot, = axes.plot([], [], 'o', color='tab:green', markersize=6,
                          label='dead reckoning')
    estimate_trail, = axes.plot([], [], '-', color='tab:red', linewidth=1,
                                alpha=0.5)
    dead_trail, = axes.plot([], [], '-', color='tab:green', linewidth=1,
                            alpha=0.5)
    right_ray, = axes.plot([], [], '-', color='tab:orange', linewidth=1,
                           alpha=0.8, label='sensor beams')
    back_ray, = axes.plot([], [], '-', color='tab:orange', linewidth=1,
                          alpha=0.8)
    sensor_rays = (right_ray, back_ray)
    axes.legend(loc='upper right', fontsize=8)

    def draw(index):
        frame = shown[index]
        cloud.set_offsets(frame['particles'][:, :2])

        x, y, theta = frame['estimate']
        estimate_dot.set_data([x], [y])
        estimate_heading.set_data(
            [x, x + 0.08 * math.cos(theta)],
            [y, y + 0.08 * math.sin(theta)])

        dead_x, dead_y, _ = frame['dead_reckoning']
        dead_dot.set_data([dead_x], [dead_y])

        # Beams drawn from the estimated pose out to the measured range.
        # A beam that stops short of the wall means the estimate is too far
        # from that wall; one that overshoots means it is too close.
        for ray, (angle, offset), measured in zip(
                sensor_rays, SENSOR_CONFIGS, frame['measurements']):
            if measured is None:
                ray.set_data([], [])
                continue
            origin_x = x + offset[0] * math.cos(theta) - offset[1] * math.sin(theta)
            origin_y = y + offset[0] * math.sin(theta) + offset[1] * math.cos(theta)
            bearing = theta + angle
            ray.set_data(
                [origin_x, origin_x + measured * math.cos(bearing)],
                [origin_y, origin_y + measured * math.sin(bearing)])

        history = shown[:index + 1]
        estimate_trail.set_data(
            [item['estimate'][0] for item in history],
            [item['estimate'][1] for item in history])
        dead_trail.set_data(
            [item['dead_reckoning'][0] for item in history],
            [item['dead_reckoning'][1] for item in history])

        axes.set_title(f'MCL  t={frame["timestamp"] - shown[0]["timestamp"]:5.1f}s'
                       f'   x={x:.3f} y={y:.3f} '
                       f'theta={math.degrees(theta):+.0f}deg')
        return (cloud, estimate_dot, estimate_heading, dead_dot,
                estimate_trail, dead_trail) + sensor_rays

    animation = FuncAnimation(
        figure, draw, frames=len(shown), interval=1000 // fps, blit=False)
    animation.save(str(output), writer=PillowWriter(fps=fps))
    plt.close(figure)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Replay an MCL log and render a GIF.')
    parser.add_argument('log', type=Path, help='CSV written by can_bridge_node')
    parser.add_argument(
        '--start', nargs=3, type=float, required=True,
        metavar=('X', 'Y', 'THETA_DEG'),
        help='True starting pose in the map: metres, metres, degrees.')
    parser.add_argument('--output', type=Path, default=Path('mcl_replay.gif'))
    parser.add_argument('--particles', type=int, default=500)
    parser.add_argument('--stride', type=int, default=2,
                        help='Render every Nth step (default 2).')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    if args.particles < 1:
        parser.error('--particles must be positive')
    if args.stride < 1:
        parser.error('--stride must be positive')
    return args


def main():
    args = parse_args()
    entries = read_log(args.log)
    start_pose = (args.start[0], args.start[1], math.radians(args.start[2]))

    rect_map = default_map()
    mcl = MCL(rect_map, default_beam_model(), SENSOR_CONFIGS,
              num_particles=args.particles, rng_seed=args.seed)
    frames = replay(entries, mcl, start_pose)

    final = frames[-1]
    x, y, theta = final['estimate']
    dead_x, dead_y, dead_theta = final['dead_reckoning']
    print(f'{len(entries)} rows, {len(frames)} filter steps')
    print(f'final MCL       x={x:.3f} y={y:.3f} '
          f'theta={math.degrees(theta):+.1f}deg')
    print(f'final dead-reck x={dead_x:.3f} y={dead_y:.3f} '
          f'theta={math.degrees(dead_theta):+.1f}deg')
    print(f'separation      {math.hypot(x - dead_x, y - dead_y) * 100:.1f}cm')

    render(frames, rect_map, args.output, fps=args.fps, stride=args.stride)
    print(f'GIF written to {args.output.resolve()}')


def _run_tests():
    # Facing +x, a world +x move is a straight-ahead body move.
    dx, dy, dtheta = body_frame_delta((0.0, 0.0, 0.0), (0.2, 0.0, 0.0))
    assert abs(dx - 0.2) < 1e-9, dx
    assert abs(dy) < 1e-9, dy
    assert abs(dtheta) < 1e-9, dtheta

    # Same world move while facing +90deg: that is a step to the robot's
    # RIGHT, which is -y in REP-103 body coordinates.
    dx, dy, _ = body_frame_delta((0.0, 0.0, math.pi / 2),
                                 (0.2, 0.0, math.pi / 2))
    assert abs(dx) < 1e-9, dx
    assert abs(dy + 0.2) < 1e-9, dy

    # Heading wrap: +170deg -> -170deg is +20deg, not -340deg.
    _, _, dtheta = body_frame_delta((0.0, 0.0, math.radians(170)),
                                    (0.0, 0.0, math.radians(-170)))
    assert abs(dtheta - math.radians(20)) < 1e-9, math.degrees(dtheta)

    # Odom starts at the origin but the rover actually started at
    # (0.3, 0.4) facing +90deg, so 0.2m of odom +x is 0.2m of map +y.
    mapped = odom_to_map((0.2, 0.0, 0.0), (0.0, 0.0, 0.0),
                         (0.3, 0.4, math.pi / 2))
    assert abs(mapped[0] - 0.3) < 1e-9, mapped
    assert abs(mapped[1] - 0.6) < 1e-9, mapped
    assert abs(mapped[2] - math.pi / 2) < 1e-9, mapped

    print('mcl_offline tests passed')


if __name__ == '__main__':
    main()
