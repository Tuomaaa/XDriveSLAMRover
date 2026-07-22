"""CSV format shared by the MCL logger (can_bridge_node) and the replay
(mcl_offline), so the two cannot drift apart.

One row per ultrasonic frame (~8Hz), carrying the most recent odometry
pose. One row is therefore exactly one MCL predict+update cycle. Ranges are
stored in millimetres to match the CAN payload and ultrasonic_collect.py;
read_log converts to metres, which is what the filter wants.

Stdlib only on purpose -- the ROS node imports this and should not pull in
numpy or matplotlib to write a CSV.
"""

import csv


COLUMNS = (
    'timestamp_s', 'odom_x', 'odom_y', 'odom_theta',
    'right_mm', 'back_mm', 'right_valid', 'back_valid',
)


def build_row(timestamp, pose, reading):
    """Format one log row.

    pose is (x, y, theta) in metres and radians; reading is the dict
    protocol.decode() returns for CAN 0x202.
    """
    x, y, theta = pose
    return (
        f'{timestamp:.6f}',
        f'{x:.6f}',
        f'{y:.6f}',
        f'{theta:.6f}',
        reading['right_mm'],
        reading['back_mm'],
        int(bool(reading['right_valid'])),
        int(bool(reading['back_valid'])),
    )


def read_log(path):
    """Parse a log file into a list of dicts with metre-valued ranges.

    A sensor whose validity bit is clear comes back as None rather than a
    number, so the filter skips it instead of scoring against the 0xFFFF
    timeout sentinel.
    """
    entries = []
    with open(path, newline='', encoding='utf-8') as source:
        reader = csv.DictReader(source)
        if (reader.fieldnames is None
                or not set(COLUMNS).issubset(reader.fieldnames)):
            raise ValueError(f'{path}: missing required columns {COLUMNS}')
        for row in reader:
            entries.append({
                'timestamp_s': float(row['timestamp_s']),
                'odom_x': float(row['odom_x']),
                'odom_y': float(row['odom_y']),
                'odom_theta': float(row['odom_theta']),
                'right_m': (float(row['right_mm']) / 1000.0
                            if int(row['right_valid']) else None),
                'back_m': (float(row['back_mm']) / 1000.0
                           if int(row['back_valid']) else None),
            })
    if not entries:
        raise ValueError(f'{path}: no rows')
    return entries


def _run_tests():
    import tempfile
    from pathlib import Path

    valid = {'right_mm': 493, 'back_mm': 392,
             'right_valid': True, 'back_valid': True}
    row = build_row(12.5, (0.5825, 0.61, 0.0), valid)
    assert len(row) == len(COLUMNS), row
    assert row[4] == 493 and row[6] == 1, row

    # right_mm carries the sentinel when the echo timed out; right_valid=0
    # is what read_log keys off, and the sentinel must not leak through.
    invalid_right = {'right_mm': 65535, 'back_mm': 392,
                     'right_valid': False, 'back_valid': True}

    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / 'log.csv'
        with path.open('w', newline='', encoding='utf-8') as sink:
            writer = csv.writer(sink)
            writer.writerow(COLUMNS)
            writer.writerow(build_row(12.5, (0.5825, 0.61, 0.0), valid))
            writer.writerow(build_row(12.6, (0.60, 0.58, 0.10), invalid_right))
        entries = read_log(path)

        empty = Path(folder) / 'empty.csv'
        with empty.open('w', newline='', encoding='utf-8') as sink:
            csv.writer(sink).writerow(COLUMNS)
        try:
            read_log(empty)
        except ValueError:
            pass
        else:
            raise AssertionError('an empty log should raise')

    assert len(entries) == 2, entries
    # Every field read_log returns is asserted, and row 0's x and y differ,
    # so neither a swapped coordinate pair nor a dropped timestamp can hide
    # behind a symmetric fixture.
    assert abs(entries[0]['timestamp_s'] - 12.5) < 1e-9, entries[0]
    assert abs(entries[0]['odom_x'] - 0.5825) < 1e-9, entries[0]
    assert abs(entries[0]['odom_y'] - 0.61) < 1e-9, entries[0]
    assert abs(entries[0]['odom_theta']) < 1e-9, entries[0]
    assert abs(entries[0]['right_m'] - 0.493) < 1e-9, entries[0]
    assert abs(entries[0]['back_m'] - 0.392) < 1e-9, entries[0]
    assert abs(entries[1]['timestamp_s'] - 12.6) < 1e-9, entries[1]
    assert abs(entries[1]['odom_x'] - 0.60) < 1e-9, entries[1]
    assert abs(entries[1]['odom_y'] - 0.58) < 1e-9, entries[1]
    assert abs(entries[1]['odom_theta'] - 0.10) < 1e-9, entries[1]
    assert entries[1]['right_m'] is None, entries[1]
    assert abs(entries[1]['back_m'] - 0.392) < 1e-9, entries[1]
    print('mcl_log tests passed')


if __name__ == '__main__':
    _run_tests()
