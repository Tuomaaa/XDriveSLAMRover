"""Collect valid HC-SR04 readings from CAN frame 0x202 into CSV."""

import argparse
import csv
from pathlib import Path
import threading
import time

from protocol import MsgId, encode_heartbeat, decode


def heartbeat_worker(can_bus, stop_event, errors):
    while not stop_event.is_set():
        try:
            can_bus.send(MsgId.HEARTBEAT, encode_heartbeat())
        except Exception as exc:
            errors.append(exc)
            stop_event.set()
            return
        stop_event.wait(0.1)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Collect valid ultrasonic readings from SocketCAN.')
    parser.add_argument(
        '--sensor', choices=('right', 'back', 'both'), required=True)
    parser.add_argument(
        '--distance', type=int, required=True,
        help='Ground-truth sensor-to-target distance in millimeters.')
    parser.add_argument(
        '--samples', type=int, default=100,
        help='Valid samples required per selected sensor (default: 100).')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--channel', default='can0')
    parser.add_argument('--bitrate', type=int, default=500_000)
    args = parser.parse_args()
    if args.distance < 0:
        parser.error('--distance must be non-negative')
    if args.samples <= 0:
        parser.error('--samples must be positive')
    return args


def collect(args):
    from can_interface import CanInterface
    sensors = ('right', 'back') if args.sensor == 'both' else (args.sensor,)
    counts = {sensor: 0 for sensor in sensors}
    args.output.parent.mkdir(parents=True, exist_ok=True)

    can_bus = CanInterface(channel=args.channel, bitrate=args.bitrate)
    stop_event = threading.Event()
    heartbeat_errors = []
    heartbeat = threading.Thread(
        target=heartbeat_worker,
        args=(can_bus, stop_event, heartbeat_errors),
        name='heartbeat',
        daemon=True,
    )
    heartbeat.start()

    try:
        with args.output.open('w', newline='', encoding='utf-8') as output:
            writer = csv.writer(output)
            writer.writerow(
                ('timestamp_s', 'sensor', 'measured_mm', 'ground_truth_mm'))

            while any(counts[sensor] < args.samples for sensor in sensors):
                if heartbeat_errors:
                    raise RuntimeError(
                        f'heartbeat sender failed: {heartbeat_errors[0]}')

                msg = can_bus.recv(timeout=0.1)
                if msg is None or msg.arbitration_id != MsgId.ULTRASONIC:
                    continue

                reading = decode(msg.arbitration_id, bytes(msg.data))
                timestamp = msg.timestamp or time.time()
                for sensor in sensors:
                    if counts[sensor] >= args.samples:
                        continue
                    if not reading[f'{sensor}_valid']:
                        continue

                    measured_mm = reading[f'{sensor}_mm']
                    counts[sensor] += 1
                    writer.writerow(
                        (f'{timestamp:.6f}', sensor, measured_mm, args.distance))
                    output.flush()
                    print(
                        f'[{counts[sensor]}/{args.samples}] '
                        f'{sensor}: {measured_mm} mm (gt: {args.distance} mm)')
    finally:
        stop_event.set()
        heartbeat.join(timeout=1.0)
        can_bus.shutdown()

    print(f'Wrote {sum(counts.values())} samples to {args.output}')


def main():
    args = parse_args()
    try:
        collect(args)
    except KeyboardInterrupt:
        print('\nCollection stopped; completed rows were flushed to disk.')


if __name__ == '__main__':
    main()