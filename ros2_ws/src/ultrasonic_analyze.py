"""Analyze ultrasonic CSV datasets and fit a range-dependent noise model."""

import argparse
import csv
from collections import defaultdict
import glob
from pathlib import Path
import statistics


def parse_args():
    parser = argparse.ArgumentParser(
        description='Summarize and plot ultrasonic measurement noise.')
    parser.add_argument(
        'files', nargs='+',
        help='CSV paths or glob patterns, for example data/right_*.csv')
    parser.add_argument(
        '--output-dir', type=Path, default=Path('ultrasonic_plots'))
    return parser.parse_args()


def expand_inputs(patterns):
    paths = []
    for pattern in patterns:
        matches = [Path(path) for path in glob.glob(pattern)]
        if matches:
            paths.extend(matches)
        else:
            path = Path(pattern)
            if path.is_file():
                paths.append(path)
    unique = list(dict.fromkeys(path.resolve() for path in paths))
    if not unique:
        raise FileNotFoundError('No input CSV files matched')
    return unique


def load_groups(paths):
    groups = defaultdict(list)
    for path in paths:
        with path.open(newline='', encoding='utf-8') as source:
            reader = csv.DictReader(source)
            required = {
                'sensor', 'measured_mm', 'ground_truth_mm', 'timestamp_s'}
            if reader.fieldnames is None or not required.issubset(reader.fieldnames):
                raise ValueError(f'{path}: missing required CSV columns')
            for row in reader:
                sensor = row['sensor'].strip().lower()
                gt_mm = int(row['ground_truth_mm'])
                groups[(sensor, gt_mm)].append(float(row['measured_mm']))
    if not groups:
        raise ValueError('Input files contained no measurements')
    return groups


def summarize(groups):
    summaries = {}
    for key, values in groups.items():
        mean_mm = statistics.fmean(values)
        std_mm = statistics.stdev(values) if len(values) > 1 else 0.0
        summaries[key] = {
            'n': len(values),
            'mean_mm': mean_mm,
            'std_mm': std_mm,
            'bias_mm': mean_mm - key[1],
        }
    return summaries


def fit_noise_model(rows):
    """Fit sigma(d) = sigma_0 + sigma_d*d using distances in meters."""
    xs = [gt_mm / 1000.0 for gt_mm, summary in rows]
    ys = [summary['std_mm'] / 1000.0 for gt_mm, summary in rows]
    if len(xs) < 2 or max(xs) == min(xs):
        return (statistics.fmean(ys), 0.0)

    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(ys)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    slope = sum(
        (x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)
    ) / denominator
    intercept = y_mean - slope * x_mean
    return (max(0.0, intercept), max(0.0, slope))


def print_summary(summaries):
    sensors = sorted({sensor for sensor, _ in summaries})
    fitted = {}
    for sensor in sensors:
        rows = sorted(
            (gt, summary)
            for (row_sensor, gt), summary in summaries.items()
            if row_sensor == sensor
        )
        print(f'\nSensor: {sensor}')
        print('GT(mm)  N    Mean(mm)  Std(mm)  Bias(mm)')
        for gt_mm, summary in rows:
            print(
                f'{gt_mm:6d}  {summary["n"]:3d}  '
                f'{summary["mean_mm"]:8.2f}  '
                f'{summary["std_mm"]:7.2f}  '
                f'{summary["bias_mm"]:+8.2f}'
            )
        sigma_0, sigma_d = fit_noise_model(rows)
        fitted[sensor] = (sigma_0, sigma_d)
        print(
            f'sigma(d) = {sigma_0:.6f} + {sigma_d:.6f} * d  '
            '(d and sigma in meters)'
        )
    return fitted


def generate_plots(groups, summaries, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    sensors = sorted({sensor for sensor, _ in groups})
    for sensor in sensors:
        rows = sorted(
            (gt, summaries[(sensor, gt)])
            for row_sensor, gt in groups
            if row_sensor == sensor
        )

        fig, ax = plt.subplots(figsize=(7, 5))
        all_gt = []
        all_measured = []
        for (row_sensor, gt), values in groups.items():
            if row_sensor == sensor:
                all_gt.extend([gt] * len(values))
                all_measured.extend(values)
        ax.scatter(all_gt, all_measured, s=12, alpha=0.45)
        low = min(all_gt + all_measured)
        high = max(all_gt + all_measured)
        ax.plot([low, high], [low, high], 'k--', label='ideal y=x')
        ax.set(
            title=f'{sensor}: measured vs ground truth',
            xlabel='Ground truth (mm)',
            ylabel='Measured (mm)',
        )
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_dir / f'{sensor}_scatter.png', dpi=160)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 5))
        gt_values = [gt for gt, _ in rows]
        biases = [summary['bias_mm'] for _, summary in rows]
        stds = [summary['std_mm'] for _, summary in rows]
        ax.errorbar(gt_values, biases, yerr=stds, fmt='o-', capsize=4)
        ax.axhline(0.0, color='black', linestyle='--', linewidth=1)
        ax.set(
            title=f'{sensor}: error vs distance',
            xlabel='Ground truth (mm)',
            ylabel='Mean error +/- 1 sigma (mm)',
        )
        fig.tight_layout()
        fig.savefig(output_dir / f'{sensor}_error.png', dpi=160)
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(7, 5))
        for (row_sensor, gt), values in sorted(groups.items()):
            if row_sensor != sensor:
                continue
            residuals = [value - gt for value in values]
            ax.hist(
                residuals, bins='auto', alpha=0.45,
                label=f'{gt} mm (N={len(values)})')
        ax.set(
            title=f'{sensor}: residual distributions',
            xlabel='Measured - ground truth (mm)',
            ylabel='Count',
        )
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_dir / f'{sensor}_histograms.png', dpi=160)
        plt.close(fig)


def main():
    args = parse_args()
    paths = expand_inputs(args.files)
    groups = load_groups(paths)
    summaries = summarize(groups)
    print_summary(summaries)
    generate_plots(groups, summaries, args.output_dir)
    print(f'\nPlots written to {args.output_dir.resolve()}')


if __name__ == '__main__':
    main()