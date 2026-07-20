"""Probabilistic beam models for rectangular-map localization."""

import math


_NEG_INF = float('-inf')
_SQRT_2PI = math.sqrt(2.0 * math.pi)


def _logsumexp(terms):
    finite_terms = [term for term in terms if math.isfinite(term)]
    if not finite_terms:
        return _NEG_INF
    maximum = max(finite_terms)
    return maximum + math.log(sum(math.exp(term - maximum) for term in finite_terms))


def _weighted_log(weight, component_log):
    if weight <= 0.0 or component_log == _NEG_INF:
        return _NEG_INF
    return math.log(weight) + component_log


def _sigma_at(sigma_hit, z_expected):
    if callable(sigma_hit):
        sigma = float(sigma_hit(z_expected))
    elif isinstance(sigma_hit, (tuple, list)):
        if len(sigma_hit) != 2:
            raise ValueError('distance-dependent sigma must be (sigma_0, sigma_d)')
        sigma = float(sigma_hit[0]) + float(sigma_hit[1]) * z_expected
    else:
        sigma = float(sigma_hit)
    if not math.isfinite(sigma) or sigma <= 0.0:
        raise ValueError('sigma_hit must remain finite and positive')
    return sigma


def _normal_cdf(value):
    return 0.5 * (1.0 + math.erf(value / math.sqrt(2.0)))


class _MultiSensorMixin:
    def total_log_likelihood(
        self,
        measurements,
        pose,
        rect_map,
        sensor_configs=None,
    ):
        """Sum sensor likelihoods after ray casting from a candidate pose.

        measurements may be numeric ranges paired with sensor_configs, or
        (range, sensor_angle, sensor_offset) tuples when sensor_configs is None.
        A None measurement is omitted. An infinite range is a max-range return.
        """
        if sensor_configs is None:
            observations = list(measurements)
        else:
            if len(measurements) != len(sensor_configs):
                raise ValueError('measurements and sensor_configs must align')
            observations = [
                (measurement, angle, offset)
                for measurement, (angle, offset)
                in zip(measurements, sensor_configs)
            ]

        total = 0.0
        used = 0
        for observation in observations:
            z_measured, sensor_angle, sensor_offset = observation
            if z_measured is None:
                continue
            z_expected = rect_map.expected_range(
                pose, sensor_angle, sensor_offset)
            score = self.log_likelihood(z_measured, z_expected)
            if score == _NEG_INF:
                return _NEG_INF
            total += score
            used += 1
        return total if used else 0.0


class BeamModel(_MultiSensorMixin):
    def __init__(
        self,
        z_max,
        sigma_hit,
        lambda_short,
        w_hit,
        w_short,
        w_max,
        w_rand,
    ):
        """Create the four-component beam model from Probabilistic Robotics.

        sigma_hit may be a positive constant, a (sigma_0, sigma_d) tuple for
        sigma(d) = sigma_0 + sigma_d*d, or a callable.
        """
        self.z_max = float(z_max)
        self.sigma_hit = sigma_hit
        self.lambda_short = float(lambda_short)
        self.w_hit = float(w_hit)
        self.w_short = float(w_short)
        self.w_max = float(w_max)
        self.w_rand = float(w_rand)

        if not math.isfinite(self.z_max) or self.z_max <= 0.0:
            raise ValueError('z_max must be finite and positive')
        if not math.isfinite(self.lambda_short) or self.lambda_short <= 0.0:
            raise ValueError('lambda_short must be finite and positive')
        weights = (self.w_hit, self.w_short, self.w_max, self.w_rand)
        if any(not math.isfinite(weight) or weight < 0.0 for weight in weights):
            raise ValueError('mixing weights must be finite and non-negative')
        if not math.isclose(sum(weights), 1.0, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError('mixing weights must sum to 1')
        _sigma_at(self.sigma_hit, 0.0)

    def _expected(self, z_expected):
        if math.isnan(z_expected) or z_expected < 0.0:
            return None
        if math.isinf(z_expected):
            return self.z_max
        return min(float(z_expected), self.z_max)

    def _log_hit(self, z, z_expected):
        if z < 0.0 or z > self.z_max:
            return _NEG_INF
        sigma = _sigma_at(self.sigma_hit, z_expected)
        lower = (0.0 - z_expected) / sigma
        upper = (self.z_max - z_expected) / sigma
        normalization = _normal_cdf(upper) - _normal_cdf(lower)
        if normalization <= 0.0:
            return _NEG_INF
        residual = (z - z_expected) / sigma
        return (
            -0.5 * residual * residual
            - math.log(_SQRT_2PI * sigma)
            - math.log(normalization)
        )

    def _log_short(self, z, z_expected):
        if z_expected <= 0.0 or z < 0.0 or z > z_expected:
            return _NEG_INF
        normalization = -math.expm1(-self.lambda_short * z_expected)
        return (
            math.log(self.lambda_short)
            - self.lambda_short * z
            - math.log(normalization)
        )

    def _max_range_log_likelihood(self):
        return _logsumexp((
            _weighted_log(self.w_max, 0.0),
            _weighted_log(self.w_rand, -math.log(self.z_max)),
        ))

    def log_likelihood(self, z_measured, z_expected):
        """Return log p(z | z_expected) for one range observation."""
        z_measured = float(z_measured)
        expected = self._expected(float(z_expected))
        if expected is None or math.isnan(z_measured) or z_measured < 0.0:
            return _NEG_INF
        if math.isinf(z_measured) or z_measured >= self.z_max:
            return self._max_range_log_likelihood()

        terms = (
            _weighted_log(
                self.w_hit, self._log_hit(z_measured, expected)),
            _weighted_log(
                self.w_short, self._log_short(z_measured, expected)),
            _NEG_INF,
            _weighted_log(self.w_rand, -math.log(self.z_max)),
        )
        return _logsumexp(terms)


class GaussianModel(_MultiSensorMixin):
    def __init__(self, z_max, sigma_hit, w_hit=0.95, w_max=0.05):
        """Gaussian hit component plus a discrete max-range component."""
        self.z_max = float(z_max)
        self.sigma_hit = sigma_hit
        self.w_hit = float(w_hit)
        self.w_max = float(w_max)
        if not math.isfinite(self.z_max) or self.z_max <= 0.0:
            raise ValueError('z_max must be finite and positive')
        if (
            self.w_hit < 0.0
            or self.w_max < 0.0
            or not math.isclose(
                self.w_hit + self.w_max, 1.0,
                rel_tol=0.0, abs_tol=1e-9)
        ):
            raise ValueError('w_hit and w_max must be non-negative and sum to 1')
        _sigma_at(self.sigma_hit, 0.0)

    def log_likelihood(self, z_measured, z_expected):
        z = float(z_measured)
        expected = float(z_expected)
        if math.isnan(z) or z < 0.0 or math.isnan(expected) or expected < 0.0:
            return _NEG_INF
        if math.isinf(z) or z >= self.z_max:
            return math.log(self.w_max) if self.w_max > 0.0 else _NEG_INF
        if math.isinf(expected):
            expected = self.z_max
        expected = min(expected, self.z_max)

        sigma = _sigma_at(self.sigma_hit, expected)
        lower = (0.0 - expected) / sigma
        upper = (self.z_max - expected) / sigma
        normalization = _normal_cdf(upper) - _normal_cdf(lower)
        residual = (z - expected) / sigma
        hit_log = (
            -0.5 * residual * residual
            - math.log(_SQRT_2PI * sigma)
            - math.log(normalization)
        )
        return _weighted_log(self.w_hit, hit_log)


def plot_beam_model(z_expected, z_max, model, output='beam_model.png'):
    """Plot the continuous beam density and the max-range spike."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    sample_count = 600
    ranges = [z_max * index / sample_count for index in range(sample_count)]
    density = [
        math.exp(model.log_likelihood(value, z_expected))
        for value in ranges
    ]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(ranges, density, label='continuous mixture')
    max_probability = math.exp(model.log_likelihood(float('inf'), z_expected))
    ax.scatter([z_max], [max_probability], marker='|', s=180, label='p_max')
    ax.set(
        title=f'Beam model at expected range {z_expected:.2f} m',
        xlabel='Measured range (m)',
        ylabel='Likelihood density / mass',
    )
    ax.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def _run_tests():
    # CALIBRATED 2026-07-20 — see measurement_model_test.py for full rationale.
    model = BeamModel(
        z_max=4.0,
        sigma_hit=(0.0017, 0.0078),
        lambda_short=0.5,
        w_hit=0.94,
        w_short=0.01,
        w_max=0.03,
        w_rand=0.02,
    )
    peak = model.log_likelihood(1.2, 1.2)
    far = model.log_likelihood(2.2, 1.2)
    assert math.isfinite(peak) and peak > far
    assert math.isfinite(model.log_likelihood(float('inf'), 1.2))
    assert model.log_likelihood(float('nan'), 1.2) == _NEG_INF

    gaussian = GaussianModel(z_max=4.0, sigma_hit=0.02)
    assert gaussian.log_likelihood(1.0, 1.0) > gaussian.log_likelihood(1.5, 1.0)
    assert math.isfinite(gaussian.log_likelihood(float('inf'), 1.0))

    try:
        BeamModel(4.0, 0.01, 1.0, 0.7, 0.1, 0.1, 0.2)
    except ValueError:
        pass
    else:
        raise AssertionError('invalid mixing weights were accepted')

    from map_model import RectMap
    rect_map = RectMap(-1.0, 1.0, -1.0, 1.0)
    score = model.total_log_likelihood(
        [1.0, 1.0],
        (0.0, 0.0, 0.0),
        rect_map,
        [(-math.pi / 2, (0.0, 0.0)), (math.pi, (0.0, 0.0))],
    )
    assert math.isfinite(score)
    print('measurement_model tests passed')


if __name__ == '__main__':
    _run_tests()