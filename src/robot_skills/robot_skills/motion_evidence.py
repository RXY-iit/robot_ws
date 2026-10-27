"""Motion progress in a fixed odometry frame, independent of map corrections."""
import math


def translation_error(start, current, dx, dy):
    distance = math.hypot(dx, dy)
    c, s = math.cos(start["yaw"]), math.sin(start["yaw"])
    wx, wy = current["x"] - start["x"], current["y"] - start["y"]
    bx, by = c * wx + s * wy, -s * wx + c * wy
    along = (bx * dx + by * dy) / distance if distance else 0.0
    cross = abs(bx * dy - by * dx) / distance if distance else 0.0
    return along, cross, math.hypot(bx - dx, by - dy)


class MotionProgress:
    def __init__(self, initial_error, source_now, wall_now, epsilon):
        self.best = initial_error
        self.source_at = source_now
        self.wall_at = wall_now
        self.epsilon = epsilon

    def stalled(self, error, source_now, wall_now):
        if self.best - error >= self.epsilon:
            self.best, self.source_at, self.wall_at = error, source_now, wall_now
        # Source-time stall is distinct from the independent wall-time failsafe.
        return source_now - self.source_at >= 5.0 or wall_now - self.wall_at >= 30.0


def swept_grid_clear(data, width, height, resolution, origin, polygon, delta):
    """Conservative swept AABB in grid coordinates; unknown/outside means blocked."""
    if resolution <= 0 or width <= 0 or height <= 0 or len(data) != width * height or len(polygon) < 3:
        return False
    c, s = math.cos(origin[2]), math.sin(origin[2])
    cells = []
    for x, y in polygon:
        for offset in ((0, 0), delta):
            wx, wy = x + offset[0] - origin[0], y + offset[1] - origin[1]
            cells.append(((c * wx + s * wy) / resolution, (-s * wx + c * wy) / resolution))
    if not all(math.isfinite(v) for p in cells for v in p):
        return False
    x0, x1 = math.floor(min(p[0] for p in cells)), math.floor(max(p[0] for p in cells))
    y0, y1 = math.floor(min(p[1] for p in cells)), math.floor(max(p[1] for p in cells))
    if x0 < 0 or y0 < 0 or x1 >= width or y1 >= height:
        return False
    return all(0 <= data[y * width + x] < 65 for y in range(y0, y1 + 1) for x in range(x0, x1 + 1))
