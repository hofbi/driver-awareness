"""Geometry Tests"""

import unittest

import pandas as pd
from geometry_msgs.msg import Point

from driver_awareness_msgs.msg import Gaze
from gaze_detector import geometry
from gaze_detector.geometry import Circle, GazeData


class GeometryTest(unittest.TestCase):
    """Geometry Test"""

    RES_X = 20
    RES_Y = 10

    def compare_normalized_and_pixel_points(self, x_norm, y_norm, x_pix, y_pix):
        expected = Point(x=x_pix, y=y_pix)
        actual = geometry.norm_to_pixel(
            Point(x=x_norm, y=y_norm), self.RES_X, self.RES_Y
        )
        self.assertEqual(actual, expected)

    def test_norm_to_pixel__border_cases__correct(self):
        self.compare_normalized_and_pixel_points(0, 0, 0, self.RES_Y)
        self.compare_normalized_and_pixel_points(0, 1, 0, 0)
        self.compare_normalized_and_pixel_points(1, 0, self.RES_X, self.RES_Y)
        self.compare_normalized_and_pixel_points(1, 1, self.RES_X, 0)

    def test_norm_to_pixel__average_case__correct(self):
        self.compare_normalized_and_pixel_points(0.1, 0.4, 2, 6)

    def test_norm_to_pixel__invalid_cases__correct(self):
        with self.assertRaises(ValueError):
            geometry.norm_to_pixel(Point(x=-1, y=0.2), self.RES_X, self.RES_Y)
        with self.assertRaises(ValueError):
            geometry.norm_to_pixel(Point(x=0.1, y=2), self.RES_X, self.RES_Y)


class CircleTest(unittest.TestCase):
    """Circle Test"""

    def setUp(self):
        self.circle = Circle(1, 2, 3)

    def test_constructor__sample_data__correct(self):
        self.assertEqual(1, self.circle.circle_x)
        self.assertEqual(2, self.circle.circle_y)
        self.assertEqual((1, 2), self.circle.position)

    def test_is_gaze_inside__all_cases__correct(self):
        self.assertTrue(self.circle.is_gaze_inside(1))
        self.assertFalse(self.circle.is_gaze_inside(3))
        self.assertFalse(self.circle.is_gaze_inside(5))

    def test_mean_dist__no_gazes_inside__zero(self):
        self.assertFalse(self.circle.has_gazes_inside())
        self.assertEqual(0, self.circle.mean_dist())

    def test_mean_dist__two_gazes_inside__greater_zero(self):
        self.circle.append_gaze_inside_dist(1.0)
        self.circle.append_gaze_inside_dist(2.0)

        self.assertEqual(1.5, self.circle.mean_dist())

    def test_append_gaze_inside_dist__gaze_inside__has_gazes(self):
        self.circle.append_gaze_inside_dist(1)

        self.assertTrue(self.circle.has_gazes_inside())

    def test_append_gaze_inside_dist__gaze_not_inside__has_no_gazes(self):
        self.circle.append_gaze_inside_dist(3)

        self.assertFalse(self.circle.has_gazes_inside())

    def test_from_pd__correct_class_created(self):
        series = pd.Series({"x": 1, "y": 2, "radius": 3})

        unit = Circle.from_pd(series)

        self.assertEqual((1, 2), unit.position)


class GazeDataTest(unittest.TestCase):
    """Gaze Data Test"""

    def setUp(self):
        self.data = GazeData()

    def test_constructor__empty_list(self):
        self.assertFalse(self.data.gaze_x)
        self.assertFalse(self.data.gaze_y)
        self.assertFalse(self.data.on_surface)

    def test_append__two_elements__correct_size(self):
        self.data.append(Gaze())
        self.data.append(Gaze())

        self.assertEqual(2, len(self.data.gaze_x))
        self.assertEqual(2, len(self.data.gaze_y))
        self.assertEqual(2, len(self.data.on_surface))

    def test_to_numpy_array__empty_data__size_correct(self):
        result = self.data.to_numpy_array()

        self.assertEqual((0, 3), result.shape)

    def test_to_numpy_array__two_elements__size_correct(self):
        self.data.append(Gaze())
        self.data.append(Gaze())

        result = self.data.to_numpy_array()

        self.assertEqual((2, 3), result.shape)


class GeometryTestSuite(unittest.TestSuite):
    """Geometry Test Suite"""

    def __init__(self):
        super().__init__()
        self.addTest(GeometryTest())
        self.addTest(CircleTest())
        self.addTest(GazeDataTest())
