"""Tests for read-only Klipper motion record encoding and timing bounds."""

import json
import unittest
from types import SimpleNamespace

from klipper_extras.robot_motion_bridge import TrapQStream, encode_move


def move(print_time, duration):
    return SimpleNamespace(
        print_time=print_time,
        move_t=duration,
        start_v=12.0,
        accel=-3.0,
        start_x=1.0,
        start_y=2.0,
        start_z=3.0,
        x_r=0.5,
        y_r=0.0,
        z_r=-0.5,
    )


class MotionRecordTests(unittest.TestCase):
    def test_encode_move_is_json_compatible(self):
        record = encode_move(move(4.0, 0.25))
        json.dumps(record)
        self.assertEqual(record["print_time"], 4.0)
        self.assertEqual(record["duration"], 0.25)
        self.assertEqual(record["start_position"], [1.0, 2.0, 3.0])
        self.assertEqual(record["direction"], [0.5, 0.0, -0.5])

    def test_pull_excludes_segments_at_or_after_safe_boundary(self):
        stream = TrapQStream(None, None)
        stream._extract = lambda _start: [
            move(1.0, 0.2),
            move(2.0, 0.3),
        ]

        records = stream.pull(safe_until=2.3)

        self.assertEqual([record["print_time"] for record in records], [1.0])
        self.assertEqual(stream.last_batch, (1.0, 0.2))


if __name__ == "__main__":
    unittest.main()
