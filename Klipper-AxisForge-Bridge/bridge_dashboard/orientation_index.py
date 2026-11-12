"""Look up the A/B/C orientation for Klipper's live XYZ position.

Klipper ignores A/B/C, so the bridge replays the uploaded file's modal state
and matches Klipper's reported XYZ to the file's path. A/B/C are interpolated
along each move, mirroring AxisForge. Rotation-only moves produce no Klipper
motion, so they complete as soon as the path moves past them.
"""

import bisect
import math

from .gcode_check import COMMENTS, ROTARY_AXES, WORD


class OrientationIndex:
    def __init__(self, path):
        self.points = [(0.0, 0.0, 0.0)]
        self.angles = [(0.0, 0.0, 0.0)]
        self._build(path)
        self._cursor = 0

    def _build(self, path):
        position = [0.0, 0.0, 0.0]
        angles = [0.0, 0.0, 0.0]
        with open(path, encoding="utf-8", errors="replace") as source:
            for line_no, raw in enumerate(source, 1):
                words = [(l.upper(), float(v)) for l, v in WORD.findall(COMMENTS.sub("", raw))]
                if not words:
                    continue
                head = words[0]
                command = f"{head[0]}{int(head[1])}" if head[0] in "GM" else ""
                if command == "G91":
                    raise ValueError(f"line {line_no}: relative positioning (G91) is not supported")
                if command not in ("", "G0", "G1"):
                    continue
                values = dict(words)
                for index, axis in enumerate("XYZ"):
                    if axis in values:
                        position[index] = values[axis]
                for index, axis in enumerate(ROTARY_AXES):
                    if axis in values:
                        angles[index] = values[axis]
                if (tuple(position), tuple(angles)) != (self.points[-1], self.angles[-1]):
                    self.points.append(tuple(position))
                    self.angles.append(tuple(angles))

    def reset(self):
        self._cursor = 0

    def orientation_at(self, xyz):
        """Return (a, b, c) for a live XYZ, searching forward from the last match."""
        best, best_t, best_distance = self._cursor, 0.0, math.inf
        for index in range(self._cursor, len(self.points) - 1):
            t, distance = self._project(index, xyz)
            if distance < best_distance - 1e-9:
                best, best_t, best_distance = index, t, distance
            elif best_distance < 0.5 and distance > best_distance + 5.0:
                break
        self._cursor = best
        start, end = self.angles[best], self.angles[best + 1] if best + 1 < len(self.angles) else self.angles[best]
        return tuple(a + best_t * (b - a) for a, b in zip(start, end))

    def _project(self, index, xyz):
        if index + 1 >= len(self.points):
            return 0.0, math.dist(self.points[index], xyz)
        a, b = self.points[index], self.points[index + 1]
        delta = [q - p for p, q in zip(a, b)]
        norm = sum(d * d for d in delta)
        t = 1.0 if norm == 0 else max(0.0, min(1.0, sum((x - p) * d for x, p, d in zip(xyz, a, delta)) / norm))
        closest = [p + t * d for p, d in zip(a, delta)]
        return t, math.dist(closest, xyz)
