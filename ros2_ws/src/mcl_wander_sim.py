"""
mcl_wander_sim.py
Synthetic "rover wandering the box" run, for validating MCL without hardware.

Why this exists: mcl_test.py's scenarios are short and deliberately simple,
and a real recording has no ground truth to score against. This generates a
full-length run with known truth, in the real CSV format, and pushes it
through the shipped replay path (mcl_offline.replay). That combination --
long, realistic, and scoreable -- is what exposed the SIGMA_ROT_FLOOR bug
described below, which every other test passed.

The rover follows two out-of-phase sinusoids around the box centre while its
heading swings, with idle stretches at the start and mid-run. The odometry
handed to the filter is corrupted the way the open-loop chassis actually is
(translation short, steady yaw bias); ranges come from the TRUE pose with the
Week 5 calibrated noise, plus the measured invalid rate.

Usage:
    python3 mcl_wander_sim.py                     # score with current constants
    python3 mcl_wander_sim.py --sweep             # sweep SIGMA_ROT_FLOOR
    python3 mcl_wander_sim.py --gif run.gif       # render the animation
    python3 mcl_wander_sim.py --gif bad.gif --rot-floor 0.002   # the failure
    python3 mcl_wander_sim.py --write-log run.csv # dump the synthetic log
"""

import argparse
import contextlib
import csv
import math
from pathlib import Path
import random

import numpy as np

import mcl
from mcl import MCL, SENSOR_CONFIGS, default_beam_model, default_map
from mcl_log import COLUMNS, build_row
from mcl_offline import render, replay

RATE_HZ = 8.0
DT = 1.0 / RATE_HZ
DURATION_S = 60.0

# Open-loop chassis corruption, matching mcl_test.py's figures.
ODOM_SCALE = 0.97
ODOM_YAW_BIAS_PER_S = 0.016      # ~0.002 rad per 8Hz step
INVALID_RATE = 0.03              # 2.9% measured on the bench
SENTINEL_MM = 65535

# Amplitude 0.30 about the centre keeps x,y in [0.28, 0.88], so both sensors
# read 0.19-0.79m -- inside the band sigma(d) was calibrated over.
CENTRE = 0.5825
AMPLITUDE = 0.30
PERIOD_X = 19.0
PERIOD_Y = 13.0
HEADING_SWING = 0.5              # +/- 0.5 rad = +/- 29 deg
PERIOD_THETA = 23.0

# Operator walking to the pad, then a mid-run pause. Zero motion is what
# collapses an unfloored cloud, and every real recording opens with it.
IDLE_WINDOWS = ((0.0, 3.0), (28.0, 32.0))

SEED_PATH = 20260722
SEED_FILTER = 0


@contextlib.contextmanager
def rotation_floor(value):
    """Temporarily override mcl.SIGMA_ROT_FLOOR.

    predict() reads the constant from module scope on every call, so this
    works. It exists so --sweep can vary the one knob under study without
    editing the library; nothing outside this diagnostic should do it.
    """
    if value is None:
        yield
        return
    original = mcl.SIGMA_ROT_FLOOR
    mcl.SIGMA_ROT_FLOOR = value
    try:
        yield
    finally:
        mcl.SIGMA_ROT_FLOOR = original


def _true_pose(t):
    return (CENTRE + AMPLITUDE * math.sin(2 * math.pi * t / PERIOD_X),
            CENTRE + AMPLITUDE * math.sin(2 * math.pi * t / PERIOD_Y + 1.1),
            HEADING_SWING * math.sin(2 * math.pi * t / PERIOD_THETA))


def _path_time(t):
    """Wall clock minus elapsed idle, so the path freezes during a pause."""
    frozen = 0.0
    for start, end in IDLE_WINDOWS:
        if t >= end:
            frozen += end - start
        elif t > start:
            frozen += t - start
    return t - frozen


def generate(duration_s=DURATION_S, seed=SEED_PATH):
    """Build the synthetic run. Returns (log_rows, truth).

    truth[i] is (t, x, y, theta) for the same instant as log_rows[i], which
    is what makes the replay scoreable.
    """
    rng = random.Random(seed)
    rect_map = default_map()

    rows, truth = [], []
    previous_true = _true_pose(_path_time(0.0))
    odom_x = odom_y = odom_theta = 0.0     # odom frame starts at its origin

    for index in range(int(duration_s * RATE_HZ)):
        t = index * DT
        current_true = _true_pose(_path_time(t))

        # True body-frame delta since the previous frame.
        dx_world = current_true[0] - previous_true[0]
        dy_world = current_true[1] - previous_true[1]
        cos_p, sin_p = math.cos(previous_true[2]), math.sin(previous_true[2])
        dx_body = dx_world * cos_p + dy_world * sin_p
        dy_body = -dx_world * sin_p + dy_world * cos_p
        dtheta = current_true[2] - previous_true[2]
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

        # What the encoders would report, integrated into the odom pose the
        # node logs. The yaw bias accrues whether or not the wheels turn.
        odom_dx = dx_body * ODOM_SCALE
        odom_dy = dy_body * ODOM_SCALE
        cos_o, sin_o = math.cos(odom_theta), math.sin(odom_theta)
        odom_x += odom_dx * cos_o - odom_dy * sin_o
        odom_y += odom_dx * sin_o + odom_dy * cos_o
        odom_theta += dtheta + ODOM_YAW_BIAS_PER_S * DT
        odom_theta = math.atan2(math.sin(odom_theta), math.cos(odom_theta))

        reading = {}
        for name, (angle, offset) in zip(('right', 'back'), SENSOR_CONFIGS):
            expected = rect_map.expected_range(current_true, angle, offset)
            measured = expected + rng.gauss(0.0, 0.0017 + 0.0078 * expected)
            if rng.random() < INVALID_RATE:
                reading[f'{name}_mm'] = SENTINEL_MM
                reading[f'{name}_valid'] = False
            else:
                reading[f'{name}_mm'] = round(measured * 1000)
                reading[f'{name}_valid'] = True

        rows.append(build_row(t, (odom_x, odom_y, odom_theta), reading))
        truth.append((t, *current_true))
        previous_true = current_true

    return rows, truth


def rows_to_entries(rows):
    """Convert built rows into the dicts mcl_offline.replay consumes.

    Deliberately goes through the same field order as COLUMNS rather than
    keeping a parallel in-memory structure, so a change to the CSV contract
    breaks this loudly instead of silently diverging.
    """
    entries = []
    for row in rows:
        record = dict(zip(COLUMNS, row))
        entries.append({
            'timestamp_s': float(record['timestamp_s']),
            'odom_x': float(record['odom_x']),
            'odom_y': float(record['odom_y']),
            'odom_theta': float(record['odom_theta']),
            'right_m': (float(record['right_mm']) / 1000.0
                        if int(record['right_valid']) else None),
            'back_m': (float(record['back_mm']) / 1000.0
                       if int(record['back_valid']) else None),
        })
    return entries


def run_filter(entries, truth, alpha=None, particles=500, seed=SEED_FILTER):
    """Replay and score against truth. Returns (frames, stats)."""
    kwargs = {'num_particles': particles, 'rng_seed': seed}
    if alpha is not None:
        kwargs['alpha'] = alpha
    filter_ = MCL(default_map(), default_beam_model(), SENSOR_CONFIGS, **kwargs)

    start_pose = (truth[0][1], truth[0][2], truth[0][3])
    frames = replay(entries, filter_, start_pose)

    # frames[i] corresponds to entries[i+1], hence truth[i+1].
    mcl_errors, dead_errors, heading_errors, theta_spreads = [], [], [], []
    for index, frame in enumerate(frames):
        _, true_x, true_y, true_theta = truth[index + 1]
        est_x, est_y, est_theta = frame['estimate']
        dead_x, dead_y, _ = frame['dead_reckoning']

        mcl_errors.append(math.hypot(est_x - true_x, est_y - true_y))
        dead_errors.append(math.hypot(dead_x - true_x, dead_y - true_y))
        heading_errors.append(abs(math.degrees(math.atan2(
            math.sin(est_theta - true_theta),
            math.cos(est_theta - true_theta)))))
        theta_spreads.append(
            math.degrees(float(np.std(frame['particles'][:, 2]))))

    rms = lambda values: math.sqrt(sum(v * v for v in values) / len(values))
    stats = {
        'mcl_rms': rms(mcl_errors),
        'dead_rms': rms(dead_errors),
        'mcl_final': mcl_errors[-1],
        'dead_final': dead_errors[-1],
        'mcl_max': max(mcl_errors),
        'heading_mean': sum(heading_errors) / len(heading_errors),
        'heading_final': heading_errors[-1],
        # Spread once the cloud has settled but before the mid-run pause.
        'theta_spread_early': theta_spreads[min(56, len(theta_spreads) - 1)],
    }
    return frames, stats


def print_stats(label, stats):
    ratio = stats['dead_rms'] / stats['mcl_rms']
    print(f'  {label}')
    print(f'    position RMS    MCL {stats["mcl_rms"] * 100:6.1f}cm   '
          f'dead-reck {stats["dead_rms"] * 100:6.1f}cm   ({ratio:.1f}x)')
    print(f'    position final  MCL {stats["mcl_final"] * 100:6.1f}cm   '
          f'dead-reck {stats["dead_final"] * 100:6.1f}cm')
    print(f'    heading error   mean {stats["heading_mean"]:5.1f}deg   '
          f'final {stats["heading_final"]:5.1f}deg')
    print(f'    heading spread at 7s {stats["theta_spread_early"]:5.2f}deg')


def render_with_truth(frames, truth, rect_map, output, fps=12, stride=8,
                      dpi=76):
    """Animate the replay against known ground truth.

    Deliberately separate from mcl_offline.render, which is what you get on
    a real recording: there is no truth to draw there, and no zoom panel
    worth the space. Here the truth exists, so the interesting questions
    are answerable -- is the estimate actually right, and is the cloud
    still alive?

    Three panels, because the overview alone hides both failure modes: at
    box scale a converged cloud is roughly 1% of the frame and simply
    disappears, and a filter that has quietly stopped correcting looks
    identical to one that is working until you plot its error.
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.gridspec import GridSpec

    shown = list(range(0, len(frames), stride))
    true_x = [truth[i + 1][1] for i in range(len(frames))]
    true_y = [truth[i + 1][2] for i in range(len(frames))]

    mcl_error = [math.hypot(frames[i]['estimate'][0] - true_x[i],
                            frames[i]['estimate'][1] - true_y[i]) * 100
                 for i in range(len(frames))]
    dead_error = [math.hypot(frames[i]['dead_reckoning'][0] - true_x[i],
                             frames[i]['dead_reckoning'][1] - true_y[i]) * 100
                  for i in range(len(frames))]
    elapsed = [frames[i]['timestamp'] - frames[0]['timestamp']
               for i in range(len(frames))]

    figure = plt.figure(figsize=(11.5, 6.0))
    grid = GridSpec(2, 2, width_ratios=(1.15, 1.0), figure=figure)
    overview = figure.add_subplot(grid[:, 0])
    zoom = figure.add_subplot(grid[0, 1])
    error_axes = figure.add_subplot(grid[1, 1])

    # ---- overview ----
    overview.set(xlim=(rect_map.x_min - 0.05, rect_map.x_max + 0.05),
                 ylim=(rect_map.y_min - 0.05, rect_map.y_max + 0.05),
                 xlabel='x (m)', ylabel='y (m)')
    overview.set_aspect('equal')
    overview.plot([rect_map.x_min, rect_map.x_max, rect_map.x_max,
                   rect_map.x_min, rect_map.x_min],
                  [rect_map.y_min, rect_map.y_min, rect_map.y_max,
                   rect_map.y_max, rect_map.y_min],
                  color='0.35', linewidth=2)
    # Whole true path up front, faint: the target the traces are judged by.
    overview.plot(true_x, true_y, color='0.55', linestyle=':', linewidth=1.2,
                  label='true path', zorder=1)

    truth_dot, = overview.plot([], [], 'o', color='black', markersize=6,
                               label='true pose', zorder=6)
    mcl_trail, = overview.plot([], [], '-', color='tab:red', linewidth=1.4,
                               alpha=0.75, zorder=4)
    dead_trail, = overview.plot([], [], '-', color='tab:green', linewidth=1.4,
                                alpha=0.75, zorder=3)
    mcl_dot, = overview.plot([], [], 'o', color='tab:red', markersize=7,
                             label='MCL estimate', zorder=5)
    dead_dot, = overview.plot([], [], 'o', color='tab:green', markersize=6,
                              label='dead reckoning', zorder=5)
    heading_line, = overview.plot([], [], '-', color='tab:red', linewidth=2,
                                  zorder=5)
    right_ray, = overview.plot([], [], '-', color='tab:orange', linewidth=1,
                               alpha=0.7, label='sensor beams', zorder=2)
    back_ray, = overview.plot([], [], '-', color='tab:orange', linewidth=1,
                              alpha=0.7, zorder=2)
    overview.legend(loc='upper left', fontsize=7.5, framealpha=0.9)

    # ---- zoom on the cloud ----
    ZOOM_M = 0.06
    zoom.set_aspect('equal')
    zoom.set_title('particle cloud (+/- 6cm)', fontsize=9)
    zoom.tick_params(labelsize=7)
    cloud = zoom.scatter([], [], s=9, alpha=0.35, color='tab:blue',
                         edgecolors='none')
    zoom_truth, = zoom.plot([], [], '+', color='black', markersize=11,
                            markeredgewidth=1.8)
    zoom_mcl, = zoom.plot([], [], 'o', color='tab:red', markersize=6)
    # When the filter loses track the cloud leaves this window entirely and
    # the panel goes blank, which reads as "particles not drawn" rather than
    # "particles nowhere near the truth". Say which it is.
    zoom_note = zoom.text(0.5, 0.80, '', transform=zoom.transAxes,
                          ha='center', va='center', fontsize=9,
                          color='tab:red')

    # ---- error vs time ----
    error_axes.set(xlim=(0, elapsed[-1]),
                   ylim=(0, max(max(dead_error), max(mcl_error)) * 1.1 + 1),
                   xlabel='t (s)', ylabel='position error (cm)')
    error_axes.tick_params(labelsize=7)
    error_axes.yaxis.label.set_size(8)
    error_axes.xaxis.label.set_size(8)
    dead_curve, = error_axes.plot([], [], '-', color='tab:green',
                                  linewidth=1.4, label='dead reckoning')
    mcl_curve, = error_axes.plot([], [], '-', color='tab:red', linewidth=1.4,
                                 label='MCL')
    error_axes.legend(loc='upper left', fontsize=7.5)
    error_axes.grid(alpha=0.25)

    def draw(position):
        index = shown[position]
        frame = frames[index]
        est_x, est_y, est_theta = frame['estimate']
        dead_x, dead_y, _ = frame['dead_reckoning']
        tx, ty = true_x[index], true_y[index]

        truth_dot.set_data([tx], [ty])
        mcl_dot.set_data([est_x], [est_y])
        dead_dot.set_data([dead_x], [dead_y])
        heading_line.set_data([est_x, est_x + 0.09 * math.cos(est_theta)],
                              [est_y, est_y + 0.09 * math.sin(est_theta)])
        mcl_trail.set_data([frames[i]['estimate'][0] for i in range(index + 1)],
                           [frames[i]['estimate'][1] for i in range(index + 1)])
        dead_trail.set_data(
            [frames[i]['dead_reckoning'][0] for i in range(index + 1)],
            [frames[i]['dead_reckoning'][1] for i in range(index + 1)])

        for ray, (angle, offset), measured in zip(
                (right_ray, back_ray), SENSOR_CONFIGS, frame['measurements']):
            if measured is None:
                ray.set_data([], [])
                continue
            origin_x = est_x + offset[0] * math.cos(est_theta) - offset[1] * math.sin(est_theta)
            origin_y = est_y + offset[0] * math.sin(est_theta) + offset[1] * math.cos(est_theta)
            bearing = est_theta + angle
            ray.set_data([origin_x, origin_x + measured * math.cos(bearing)],
                         [origin_y, origin_y + measured * math.sin(bearing)])

        cloud.set_offsets(frame['particles'][:, :2])
        zoom.set_xlim(tx - ZOOM_M, tx + ZOOM_M)
        zoom.set_ylim(ty - ZOOM_M, ty + ZOOM_M)
        zoom_truth.set_data([tx], [ty])
        zoom_mcl.set_data([est_x], [est_y])

        inside = np.count_nonzero(
            (np.abs(frame['particles'][:, 0] - tx) <= ZOOM_M)
            & (np.abs(frame['particles'][:, 1] - ty) <= ZOOM_M))
        if inside:
            zoom_note.set_text('')
        else:
            zoom_note.set_text(
                f'entire cloud is off-panel\n'
                f'{mcl_error[index]:.0f}cm from the true pose')

        mcl_curve.set_data(elapsed[:index + 1], mcl_error[:index + 1])
        dead_curve.set_data(elapsed[:index + 1], dead_error[:index + 1])

        spread = math.degrees(float(np.std(frame['particles'][:, 2])))
        figure.suptitle(
            f't={elapsed[index]:4.1f}s     '
            f'MCL error {mcl_error[index]:5.1f}cm     '
            f'dead reckoning {dead_error[index]:5.1f}cm     '
            f'heading spread {spread:4.2f}deg',
            fontsize=10.5, family='monospace')
        return (truth_dot, mcl_dot, dead_dot, heading_line, mcl_trail,
                dead_trail, right_ray, back_ray, cloud, zoom_truth,
                zoom_mcl, zoom_note, mcl_curve, dead_curve)

    figure.tight_layout(rect=(0, 0, 1, 0.94))
    animation = FuncAnimation(figure, draw, frames=len(shown),
                              interval=1000 // fps, blit=False)
    # dpi trades legibility against file size; 76 keeps a 60-frame run
    # under ~1MB while the zoom panel and error curve stay readable.
    animation.save(str(output), writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(figure)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Synthetic wander run for validating MCL without hardware.')
    parser.add_argument('--gif', type=Path, help='Render the animation here.')
    parser.add_argument('--write-log', type=Path,
                        help='Dump the synthetic log in the real CSV format.')
    parser.add_argument('--rot-floor', type=float,
                        help='Override mcl.SIGMA_ROT_FLOOR for this run.')
    parser.add_argument('--sweep', action='store_true',
                        help='Sweep SIGMA_ROT_FLOOR instead of a single run.')
    parser.add_argument('--particles', type=int, default=500)
    parser.add_argument('--stride', type=int, default=8,
                        help='Render every Nth step (default 8).')
    parser.add_argument('--fps', type=int, default=12)
    parser.add_argument('--dpi', type=int, default=76,
                        help='Render resolution (default 76).')
    args = parser.parse_args()
    if args.particles < 1:
        parser.error('--particles must be positive')
    if args.stride < 1:
        parser.error('--stride must be positive')
    if args.fps < 1:
        parser.error('--fps must be positive')
    if args.dpi < 10:
        parser.error('--dpi must be at least 10')
    if args.rot_floor is not None and args.rot_floor < 0.0:
        parser.error('--rot-floor must be non-negative')
    return args


def main():
    args = parse_args()
    rows, truth = generate()
    entries = rows_to_entries(rows)

    invalid = sum(1 for row in rows if not (int(row[6]) and int(row[7])))
    print(f'{len(rows)} rows at {RATE_HZ:.0f}Hz ({DURATION_S:.0f}s), '
          f'{invalid} frames with an invalid sensor '
          f'({invalid / len(rows) * 100:.1f}%)')
    print(f'true end pose  x={truth[-1][1]:.3f} y={truth[-1][2]:.3f} '
          f'theta={math.degrees(truth[-1][3]):+.1f}deg')

    if args.write_log:
        with args.write_log.open('w', newline='', encoding='utf-8') as sink:
            writer = csv.writer(sink)
            writer.writerow(COLUMNS)
            writer.writerows(rows)
        print(f'log written to {args.write_log.resolve()}')

    if args.sweep:
        print(f'\nSIGMA_ROT_FLOOR sweep (library default is '
              f'{mcl.SIGMA_ROT_FLOOR}):')
        for floor in (0.002, 0.005, 0.010, 0.020, 0.040):
            with rotation_floor(floor):
                _, stats = run_filter(entries, truth, particles=args.particles)
            print_stats(f'floor {floor:.3f}', stats)
        return

    with rotation_floor(args.rot_floor):
        active = mcl.SIGMA_ROT_FLOOR
        frames, stats = run_filter(entries, truth, particles=args.particles)
        print(f'\nSIGMA_ROT_FLOOR = {active}')
        print_stats(f'{args.particles} particles', stats)

        if args.gif:
            render_with_truth(frames, truth, default_map(), args.gif,
                              fps=args.fps, stride=args.stride, dpi=args.dpi)
            print(f'\nGIF written to {args.gif.resolve()}')


def _run_tests():
    """Guard the properties the scoring depends on."""
    rows, truth = generate(duration_s=8.0)
    assert len(rows) == len(truth) == 64, (len(rows), len(truth))

    # Truth and log must describe the same instants, or every score is
    # silently offset by a frame.
    for row, truth_entry in zip(rows[:5], truth[:5]):
        assert abs(float(row[0]) - truth_entry[0]) < 1e-9, (row[0], truth_entry)

    # The rover must stay inside the box, and inside the calibrated range
    # band -- otherwise the scenario tests extrapolated sensor behaviour.
    rect_map = default_map()
    for _, x, y, theta in truth:
        assert rect_map.x_min < x < rect_map.x_max, x
        assert rect_map.y_min < y < rect_map.y_max, y
        for angle, offset in SENSOR_CONFIGS:
            expected = rect_map.expected_range((x, y, theta), angle, offset)
            assert 0.10 < expected < 1.10, expected

    # The first idle window must actually hold the path still, since the
    # noise floor's parked behaviour is one of the things this exercises.
    assert truth[4][1:] == truth[8][1:], 'idle window did not freeze the path'

    entries = rows_to_entries(rows)
    assert len(entries) == len(rows)
    assert entries[0]['timestamp_s'] == 0.0

    # An invalid sensor must arrive as None, never as the 0xFFFF sentinel
    # converted to 65.535 metres.
    for entry in rows_to_entries(generate(duration_s=DURATION_S)[0]):
        for key in ('right_m', 'back_m'):
            assert entry[key] is None or entry[key] < 2.0, entry

    with rotation_floor(0.123):
        assert mcl.SIGMA_ROT_FLOOR == 0.123
    assert mcl.SIGMA_ROT_FLOOR != 0.123, 'rotation_floor did not restore'

    print('mcl_wander_sim tests passed')


if __name__ == '__main__':
    main()
