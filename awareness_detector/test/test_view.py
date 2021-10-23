"""View Test"""

import unittest

from driver_awareness_msgs.msg import Gaze, ROI
from awareness_detector.sa import SituationElement
from awareness_detector.view import CameraAdjustment, ScreenParameter, Color
from awareness_detector import view
from geometry_msgs.msg import Point


def create_gaze(x_pos, y_pos):
    """Create gaze data with predefined positions"""
    return Gaze(gaze_pixel=Point(x=x_pos, y=y_pos))


def compare_gazes(testcase, expected, actual):
    """Compare the x and y pixel positions of the gazes"""
    testcase.assertEqual(expected.gaze_pixel.x, actual.gaze_pixel.x)
    testcase.assertEqual(expected.gaze_pixel.y, actual.gaze_pixel.y)


class ViewTest(unittest.TestCase):
    """View Test"""

    def test_get_color_from_classification__default__red(self):
        roi = ROI()

        result = view.get_color_from_classification(roi)

        self.assertEqual(Color.RED, result)

    def test_get_color_from_classification__detected__yellow(self):
        roi = ROI(classification=SituationElement.Classification.DETECTED)

        result = view.get_color_from_classification(roi)

        self.assertEqual(Color.YELLOW, result)

    def test_get_color_from_classification__comprehended__green(self):
        roi = ROI(classification=SituationElement.Classification.COMPREHENDED)

        result = view.get_color_from_classification(roi)

        self.assertEqual(Color.GREEN, result)


class CameraAdjustmentTest(unittest.TestCase):
    """Camera Adjustment Test"""

    def setUp(self):
        screen_parameter = ScreenParameter(0, 0, 0, 0, 1920, 0, 1920, 1080, 640, 480)
        self.camera_adjustment = CameraAdjustment(screen_parameter)

    def test_adjust_gaze_data_to_camera_resolution__0_0(self):
        actual_gaze = create_gaze(0, 0)
        self.camera_adjustment.adjust_gaze_data_to_camera_resolution(actual_gaze)

        compare_gazes(self, create_gaze(-240.0 / 2.25, 0), actual_gaze)

    def test_adjust_gaze_data_to_camera_resolution__240_0(self):
        actual_gaze = create_gaze(240, 0)
        self.camera_adjustment.adjust_gaze_data_to_camera_resolution(actual_gaze)

        compare_gazes(self, create_gaze(0, 0), actual_gaze)

    def test_adjust_gaze_data_to_camera_resolution__640_480(self):
        actual_gaze = create_gaze(640, 480)
        self.camera_adjustment.adjust_gaze_data_to_camera_resolution(actual_gaze)

        compare_gazes(self, create_gaze(400 / 2.25, 480 / 2.25), actual_gaze)

    def test_adjust_gaze_data_to_camera_resolution_with_padding_0_0(self):
        actual_gaze = create_gaze(0, 0)
        self.camera_adjustment.adjust_gaze_data_to_camera_resolution(actual_gaze)

        compare_gazes(self, create_gaze(-20 / 2.1875, -20 / 2.1875), actual_gaze)

    def test_adjust_gaze_data_to_camera_resolution_with_padding_240_0(self):
        actual_gaze = create_gaze(20, 20)
        self.camera_adjustment.adjust_gaze_data_to_camera_resolution(actual_gaze)

        compare_gazes(self, create_gaze(0, 0), actual_gaze)

    def test_adjust_gaze_data_to_camera_resolution_with_padding_640_480(self):
        actual_gaze = create_gaze(1420, 1070)
        self.camera_adjustment.adjust_gaze_data_to_camera_resolution(actual_gaze)

        compare_gazes(self, create_gaze(640, 480), actual_gaze)


class ViewTestSuite(unittest.TestSuite):
    """View Test"""

    def __init__(self):
        super(ViewTestSuite, self).__init__()
        self.addTest(ViewTest())
        self.addTest(CameraAdjustmentTest())
