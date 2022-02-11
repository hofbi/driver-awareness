"""Pupil Tests"""

import unittest

from gaze_detector import pupil


class PupilTest(unittest.TestCase):
    """Pupil Test"""

    def test_is_calibration_point_finished__wrong_mode__false(self):
        message = {b"name": "foo"}

        self.assertFalse(pupil.is_calibration_point_finished(message))

    def test_is_calibration_point_finished__not_sampled__false(self):
        message = {b"name": "manual_marker_calibration", b"msg": "blub"}

        self.assertFalse(pupil.is_calibration_point_finished(message))

    def test_is_calibration_point_finished__sampled__true(self):
        message = {b"name": "manual_marker_calibration", b"msg": "Sampled"}

        self.assertTrue(pupil.is_calibration_point_finished(message))


class PupilTestSuite(unittest.TestSuite):
    """Pupil Test Suite"""

    def __init__(self):
        super().__init__()
        self.addTest(PupilTest())
