"""Grid density preserves bed coverage, start corner and serpentine traversal."""
import unittest

import numpy as np

from print_control.bed_probe import grid_points


class ProbeGridTests(unittest.TestCase):
    def test_every_supported_shape_has_even_spacing_and_no_diagonal_transitions(self):
        for columns in range(2, 10):
            for rows in range(2, 10):
                with self.subTest(columns=columns, rows=rows):
                    points = np.asarray(grid_points((235., 200.), 30., columns, rows))
                    self.assertEqual(len(points), columns*rows)
                    self.assertEqual(len(set(map(tuple, points))), columns*rows)
                    np.testing.assert_array_equal(points[0], [30., 30.])
                    np.testing.assert_array_equal(points.min(axis=0), [30., 30.])
                    np.testing.assert_array_equal(points.max(axis=0), [205., 170.])
                    np.testing.assert_allclose(np.diff(np.unique(points[:, 0])), 175./(columns-1))
                    np.testing.assert_allclose(np.diff(np.unique(points[:, 1])), 140./(rows-1))
                    self.assertTrue(np.all(np.count_nonzero(np.diff(points, axis=0), axis=1) == 1))

    def test_invalid_counts_are_rejected_before_building_a_grid(self):
        for count in (1, 10, 3., True, '3', None, float('nan')):
            for columns, rows in ((count, 3), (3, count)):
                with self.subTest(columns=columns, rows=rows), self.assertRaises(ValueError):
                    grid_points((235., 235.), 30., columns, rows)

    def test_default_retains_the_original_nine_points(self):
        self.assertEqual(grid_points((235., 235.), 30.), [
            (30., 30.), (117.5, 30.), (205., 30.),
            (205., 117.5), (117.5, 117.5), (30., 117.5),
            (30., 205.), (117.5, 205.), (205., 205.)])
