"""Axis-aligned rectangular map and 2D ray casting."""

import math


_EPSILON = 1e-12


class RectMap:
    def __init__(self, x_min, x_max, y_min, y_max):
        """Walls at the four supplied coordinates. Units are meters."""
        bounds = (x_min, x_max, y_min, y_max)
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError('map bounds must be finite')
        if x_min >= x_max or y_min >= y_max:
            raise ValueError('minimum bounds must be smaller than maximum bounds')
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)

    def ray_cast(self, x, y, angle):
        """Return distance from a ray origin to the nearest wall segment."""
        dx = math.cos(angle)
        dy = math.sin(angle)
        candidates = []

        if abs(dx) > _EPSILON:
            for wall_x in (self.x_min, self.x_max):
                distance = (wall_x - x) / dx
                hit_y = y + distance * dy
                if (
                    distance > _EPSILON
                    and self.y_min - _EPSILON <= hit_y <= self.y_max + _EPSILON
                ):
                    candidates.append(distance)

        if abs(dy) > _EPSILON:
            for wall_y in (self.y_min, self.y_max):
                distance = (wall_y - y) / dy
                hit_x = x + distance * dx
                if (
                    distance > _EPSILON
                    and self.x_min - _EPSILON <= hit_x <= self.x_max + _EPSILON
                ):
                    candidates.append(distance)

        return min(candidates) if candidates else float('inf')

    def expected_range(
        self,
        pose,
        sensor_angle_on_robot,
        sensor_offset=(0.0, 0.0),
    ):
        """Transform a robot-frame sensor ray and cast it in the world frame."""
        x, y, theta = pose
        offset_x, offset_y = sensor_offset
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        sensor_x = x + cos_theta * offset_x - sin_theta * offset_y
        sensor_y = y + sin_theta * offset_x + cos_theta * offset_y
        return self.ray_cast(
            sensor_x, sensor_y, theta + sensor_angle_on_robot)


def _run_tests():
    rect_map = RectMap(-1.0, 1.0, -1.0, 1.0)

    assert math.isclose(
        rect_map.expected_range((0.0, 0.0, 0.0), -math.pi / 2), 1.0)
    assert math.isclose(
        rect_map.expected_range((0.0, 0.0, 0.0), math.pi), 1.0)
    assert math.isclose(
        rect_map.expected_range((0.3, 0.0, 0.0), math.pi), 1.3)
    assert math.isclose(
        rect_map.expected_range((0.3, 0.0, math.pi / 2), -math.pi / 2),
        0.7,
    )

    # From a wall, an inward ray reaches the opposite wall; an outward ray
    # has no positive intersection. An outside ray can still hit an entry wall.
    assert math.isclose(rect_map.ray_cast(1.0, 0.0, math.pi), 2.0)
    assert math.isinf(rect_map.ray_cast(1.0, 0.0, 0.0))
    assert math.isclose(rect_map.ray_cast(2.0, 0.0, math.pi), 1.0)
    assert math.isinf(rect_map.ray_cast(2.0, 0.0, 0.0))

    offset_range = rect_map.expected_range(
        (0.0, 0.0, 0.0), -math.pi / 2, (0.0, -0.09))
    assert math.isclose(offset_range, 0.91)

    print('map_model tests passed')


if __name__ == '__main__':
    _run_tests()