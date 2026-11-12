import tempfile
import unittest
from pathlib import Path

from bridge_dashboard.orientation_index import OrientationIndex


class OrientationIndexTest(unittest.TestCase):
    def index(self, text):
        path = Path(tempfile.mkdtemp()) / "job.gcode"
        path.write_text(text)
        return OrientationIndex(path)

    def test_interpolates_along_move(self):
        index = self.index("G1 X0 Y0 Z0\nG1 X10 A20 B-10 C0\n")
        a, b, c = index.orientation_at((5, 0, 0))
        self.assertAlmostEqual(a, 10)
        self.assertAlmostEqual(b, -5)
        self.assertAlmostEqual(c, 0)

    def test_modal_and_forward_only(self):
        index = self.index("G1 X0 Y0 Z0\nG1 X10 C30\nG1 X20\nG1 X30 C0\n")
        self.assertAlmostEqual(index.orientation_at((15, 0, 0))[2], 30)
        self.assertAlmostEqual(index.orientation_at((25, 0, 0))[2], 15)

    def test_rotation_only_move_completes_before_next_motion(self):
        index = self.index("G1 X0 Y0 Z0\nG1 X10\nG1 C90\nG1 X20\n")
        self.assertAlmostEqual(index.orientation_at((15, 0, 0))[2], 90)

    def test_relative_positioning_rejected(self):
        with self.assertRaises(ValueError):
            self.index("G91\nG1 X1\n")


if __name__ == "__main__":
    unittest.main()
