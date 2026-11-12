import unittest

from print_control.gcode_parser import GCodeParser


class OrientationParsingTest(unittest.TestCase):
    def parse(self, text):
        return list(GCodeParser().parse(text.splitlines()))

    def test_abc_are_modal_absolute(self):
        segments = self.parse('G1 X10 A5 B-10 C15 F600\nG1 X20\nG1 B0')
        self.assertEqual(segments[0].start_orientation_deg, (0.0, 0.0, 0.0))
        self.assertEqual(segments[0].orientation_deg, (5.0, -10.0, 15.0))
        self.assertEqual(segments[1].orientation_deg, (5.0, -10.0, 15.0))
        self.assertEqual(segments[1].start_orientation_deg, (5.0, -10.0, 15.0))
        self.assertEqual(segments[2].orientation_deg, (5.0, 0.0, 15.0))

    def test_rotation_only_move_has_zero_length(self):
        segment = self.parse('G1 C90 F600')[0]
        self.assertEqual(segment.orientation_deg, (0.0, 0.0, 90.0))
        self.assertEqual(segment.start, segment.end)


if __name__ == '__main__':
    unittest.main()
