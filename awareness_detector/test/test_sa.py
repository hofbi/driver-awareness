#!/usr/bin/env python

"""SA Test"""
import math
import time
import itertools
import unittest
from unittest.mock import MagicMock

from awareness_detector.sa import (
    GazeBuffer,
    SituationAwareness,
    SituationAwarenessParameter,
    SituationElement,
    SituationElementTracker,
    DistanceTrackingStrategy,
    IdTrackingStrategy,
    PunishmentModel,
)
from driver_awareness_msgs.msg import Gaze, ROI
from awareness_detector.geometry import Object2D
from geometry_msgs.msg import Point
from opencv_apps.msg import Point2D

COMPREHENSION_ALIVE_TIME = 1
NUM_GAZE_COMPREHENDED = 3
NUM_GAZE_DETECTED = 1
TEST_SA_PARAMETER = SituationAwarenessParameter(
    COMPREHENSION_ALIVE_TIME, NUM_GAZE_COMPREHENDED, NUM_GAZE_DETECTED
)


def create_se(
    se_type=ROI().type,
    classification=ROI().classification,
    x_pos=ROI().roi.center.x,
    y_pos=ROI().roi.center.y,
    radius=ROI().roi.radius,
):
    """Create Situation Element"""
    roi_msg = create_roi(x_pos, y_pos, radius)
    roi_msg.type = se_type
    roi_msg.classification = classification
    return SituationElement(roi_msg, TEST_SA_PARAMETER)


def create_roi(x_pos, y_pos, radius=ROI().roi.radius, roi_id=0):
    """Create ROI with predefined positions"""
    roi = ROI()
    roi.roi.center = Point2D(x_pos, y_pos)
    roi.roi.radius = radius
    roi.id = roi_id
    return roi


def create_gaze(x_pos, y_pos):
    """Create gaze data with predefined positions"""
    return Gaze(gaze_pixel=Point(x=x_pos, y=y_pos))


class SituationAwarenessTest(unittest.TestCase):
    """Situation Awareness Test"""

    def setUp(self):
        self.mock = MagicMock()
        self.mock.punishment.return_value = 0.0
        self.situation_awareness = SituationAwareness(self.mock)

    def assert_sa(self, optimal, actual):
        self.assertEqual(optimal, self.situation_awareness.sa_msg.optimal_sa)
        self.assertEqual(actual, self.situation_awareness.sa_msg.actual_sa)

    def test_constructor__sa_msg_all_zero(self):
        self.assert_sa(0, 0)

    def test_calculate_sa__empty_lists__sa_msg_all_zero(self):
        self.situation_awareness.calculate_sa([], [])

        self.assert_sa(0, 0)

    def test_calculate_sa__one_required_undetected__sa_opt_2_sa_act_0(self):
        roi = create_se(ROI.REQUIRED)

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(2, 0)

    def test_calculate_sa__one_desired_undetected__sa_opt_1_sa_act_0(self):
        roi = create_se(ROI.DESIRED)

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(1, 0)

    def test_calculate_sa__one_required_detected__sa_opt_2_sa_act_1(self):
        roi = create_se(ROI.REQUIRED, SituationElement.Classification.DETECTED)

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(2, 1)

    def test_calculate_sa__one_desired_detected__sa_opt_1_sa_act_0p5(self):
        roi = create_se(ROI.DESIRED, SituationElement.Classification.DETECTED)

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(1, 0.5)

    def test_calculate_sa__one_required_comprehended__sa_opt_2_sa_act_2(self):
        roi = create_se(ROI.REQUIRED, SituationElement.Classification.COMPREHENDED)

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(2, 2)

    def test_calculate_sa__one_desired_comprehended__sa_opt_1_sa_act_1(self):
        roi = create_se(ROI.DESIRED, SituationElement.Classification.COMPREHENDED)

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(1, 1)

    def test_calculate_sa__one_desired_comprehended_and_punishment__sa_opt_1_sa_act_0(
        self,
    ):
        roi = create_se(ROI.DESIRED, SituationElement.Classification.COMPREHENDED)
        self.mock.punishment.return_value = 1.0

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(1, 0)

    def test_calculate_sa__one_required_comprehended_and_2_punishments__sa_opt_2_sa_act_0(
        self,
    ):
        roi = create_se(ROI.REQUIRED, SituationElement.Classification.COMPREHENDED)
        self.mock.punishment.return_value = 2.0

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(2, 0)

    def test_calculate_sa__one_desired_comprehended_and_2_punishments__sa_opt_1_sa_act_0(
        self,
    ):
        roi = create_se(ROI.DESIRED, SituationElement.Classification.COMPREHENDED)
        self.mock.punishment.return_value = 2.0

        self.situation_awareness.calculate_sa([roi], [])

        self.assert_sa(1, 0)


class PunishmentModelTest(unittest.TestCase):
    """Punishment Model Test"""

    def setUp(self):
        sample_coordinates = [[1, 2], [2, 2], [2, 3], [8, 7], [8, 8], [25, 80]]
        self.sample_gazes = [
            create_gaze(coord[0], coord[1]) for coord in sample_coordinates
        ]
        self.punishment_model = PunishmentModel(3, 2, 1)

    def test_punishment__empty_input_list__zero(self):
        self.assertEqual(0.0, self.punishment_model.punishment([]))

    def test_punishment__dbscan_doc_sample_input__two(self):
        self.assertEqual(2.0, self.punishment_model.punishment(self.sample_gazes))

    def test_punishment__dbscan_doc_sample_input_and_scale_factor_2__four(self):
        punishment_model = PunishmentModel(3, 2, 2)

        self.assertEqual(4.0, punishment_model.punishment(self.sample_gazes))


class SituationElementTest(unittest.TestCase):
    """Situation Element Test"""

    def setUp(self):
        self.situation_element = create_se()
        self.emulated_time = itertools.count(0, 1)
        # Register the time.time function and patch it then
        self.time_method = time.time

    def set_emulated_time(self, time_delta):
        self.emulated_time = itertools.count(next(self.emulated_time), time_delta)

        time.time = lambda: next(self.emulated_time)

    def unset_emulated_time(self):
        time.time = self.time_method

    def test_constructor__default_values(self):
        roi_msg = ROI()

        self.assertEqual(roi_msg, self.situation_element.roi_msg)
        self.assertFalse(self.situation_element.roi_msg.classification)
        self.assertTrue(self.situation_element.is_alive)
        self.assertTrue(self.situation_element.has_changed)

    def test_update_msg__values_updated(self):
        new_roi_msg = ROI()
        new_roi_msg.type = ROI.DESIRED
        self.assertNotEqual(new_roi_msg, self.situation_element.roi_msg)

        self.situation_element.update_msg(new_roi_msg)

        self.assertTrue(self.situation_element.has_changed)
        self.assertEqual(new_roi_msg, self.situation_element.roi_msg)
        self.assertTrue(self.situation_element.is_alive)

    def test_update__updated_once__is_alive_true(self):
        self.situation_element.update(0)

        self.assertFalse(self.situation_element.has_changed)
        self.assertTrue(self.situation_element.is_alive)
        self.assertFalse(self.situation_element.roi_msg.classification)

    def test_update__update_twice__is_alive_false(self):
        self.situation_element.update(0)
        self.situation_element.update(0)

        self.assertFalse(self.situation_element.has_changed)
        self.assertFalse(self.situation_element.is_alive)

    def test_update__1_gazes_inside__classification_detected(self):
        self.situation_element.update(NUM_GAZE_DETECTED)

        self.assertEqual(
            SituationElement.Classification.DETECTED,
            self.situation_element.roi_msg.classification,
        )

    def test_update__many_gazes_inside__classification_comprehended(self):
        self.situation_element.update(NUM_GAZE_COMPREHENDED + 1)

        self.assertEqual(
            SituationElement.Classification.COMPREHENDED,
            self.situation_element.roi_msg.classification,
        )

    def test_update__comprehension_expired__classification_detected(self):
        self.situation_element.update(NUM_GAZE_COMPREHENDED + 1)

        self.set_emulated_time(2 * COMPREHENSION_ALIVE_TIME)
        self.situation_element.update(0)
        self.unset_emulated_time()

        self.assertEqual(
            SituationElement.Classification.DETECTED,
            self.situation_element.roi_msg.classification,
        )

    def test_is_comprehension_expired__below_expiring_time__false(self):
        self.set_emulated_time(0.5 * COMPREHENSION_ALIVE_TIME)
        self.assertFalse(self.situation_element.is_comprehension_expired())
        self.unset_emulated_time()

    def test_is_comprehension_expired__above_expiring_time_not_comprehended__false(
        self,
    ):
        self.set_emulated_time(2 * COMPREHENSION_ALIVE_TIME)
        self.assertNotEqual(
            SituationElement.Classification.COMPREHENDED,
            self.situation_element.roi_msg.classification,
        )
        self.assertFalse(self.situation_element.is_comprehension_expired())
        self.unset_emulated_time()

    def test_is_comprehension_expired__above_expiring_time_comprehended__true(self):
        self.set_emulated_time(0.05 * COMPREHENSION_ALIVE_TIME)
        self.situation_element.update(NUM_GAZE_COMPREHENDED + 1)
        self.set_emulated_time(2 * COMPREHENSION_ALIVE_TIME)
        time.time()

        self.assertEqual(
            SituationElement.Classification.COMPREHENDED,
            self.situation_element.roi_msg.classification,
        )
        self.assertTrue(self.situation_element.is_comprehension_expired())
        self.unset_emulated_time()

    def test_update__msg_update_updated_once__is_alive_true(self):
        self.situation_element.update_msg(ROI())

        self.situation_element.update(0)

        self.assertFalse(self.situation_element.has_changed)
        self.assertTrue(self.situation_element.is_alive)

    def test_update__msg_update_updated_twice__is_alive_false(self):
        self.situation_element.update_msg(ROI())

        self.situation_element.update(0)
        self.situation_element.update(0)

        self.assertFalse(self.situation_element.has_changed)
        self.assertFalse(self.situation_element.is_alive)

    def test_type_weight__desired__type_weight_1(self):
        unit = create_se(ROI.DESIRED)

        self.assertEqual(SituationElement.TypeWeight.DESIRED, unit.type_weight)

    def test_type_weight__required__type_weight_2(self):
        unit = create_se(ROI.REQUIRED)

        self.assertEqual(SituationElement.TypeWeight.REQUIRED, unit.type_weight)

    def test_classification_value__all_cases__correct_weights(self):
        self.assertEqual(
            SituationElement.ClassificationWeight.COMPREHENDED,
            create_se(
                ROI.REQUIRED, SituationElement.Classification.COMPREHENDED
            ).classification_value,
        )
        self.assertEqual(
            SituationElement.ClassificationWeight.DETECTED,
            create_se(
                ROI.REQUIRED, SituationElement.Classification.DETECTED
            ).classification_value,
        )
        self.assertEqual(
            SituationElement.ClassificationWeight.UNDETECTED,
            create_se(ROI.REQUIRED).classification_value,
        )

    def test_distance_to__same_position__distance_0(self):
        result = self.situation_element.distance_to(self.situation_element.roi_msg)

        self.assertEqual(0, result)

    def test_distance_to__center_is_1_and_1__distance_sqrt_2(self):
        roi = create_roi(1, 1)

        result = self.situation_element.distance_to(roi)

        self.assertEqual(math.sqrt(2), result)

    def test_is_gaze_inside__gaze_outside__false(self):
        element = create_se(radius=1)
        gaze = create_gaze(1, 1)

        self.assertFalse(element.is_gaze_inside(gaze))

    def test_is_gaze_inside__gaze_inside__true(self):
        element = create_se(radius=1)
        gaze = create_gaze(0, 0)

        self.assertTrue(element.is_gaze_inside(gaze))

    def test_is_gaze_inside__gaze_on_border__true(self):
        element = create_se(radius=1)
        gaze = create_gaze(1, 0)

        self.assertTrue(element.is_gaze_inside(gaze))


class SituationElementTrackerTest(unittest.TestCase):
    """Situation Element Tracker Test"""

    def setUp(self):
        self.se_list = [
            SituationElement(ROI(), TEST_SA_PARAMETER),
            SituationElement(create_roi(10, 5), TEST_SA_PARAMETER),
        ]
        self.test_gazes = [create_gaze(0, 0), create_gaze(0, 1), create_gaze(1, 0)]
        self.mock = MagicMock()
        self.mock.find_best_candidates.return_value = []

    def test_constructor__empty_list(self):
        tracker = SituationElementTracker(None, 1)

        self.assertFalse(tracker.se_list)
        self.assertFalse(tracker.non_roi_gazes)

    def test_add_or_update_ses__add_2_without_best_candidates__len_is_2(self):
        tracker = SituationElementTracker(self.mock, 1)

        tracker.add_or_update_ses([ROI(), ROI()])

        self.assertEqual(2, len(tracker.se_list))
        self.assertFalse(tracker.non_roi_gazes)

    def test_add_or_update_ses__add_2_with_best_candidates__len_is_2(self):
        tracker = SituationElementTracker(self.mock, 1)
        tracker.add_or_update_ses([ROI(), ROI()])
        self.mock.find_best_candidates.return_value = [
            SituationElement(ROI(), TEST_SA_PARAMETER)
        ]

        tracker.add_or_update_ses([ROI(), ROI()])

        self.assertEqual(2, len(tracker.se_list))
        self.assertFalse(tracker.non_roi_gazes)

    def test_update_elements__2_elements_update_once__len_is_2(self):
        tracker = SituationElementTracker(self.mock, TEST_SA_PARAMETER)
        tracker.add_or_update_ses([ROI(), ROI()])

        tracker.update_elements([])

        self.assertEqual(2, len(tracker.se_list))
        self.assertFalse(tracker.non_roi_gazes)

    def test_update_elements__2_elements_update_twice__empty(self):
        tracker = SituationElementTracker(self.mock, TEST_SA_PARAMETER)
        tracker.add_or_update_ses([ROI(), ROI()])

        tracker.update_elements([])
        tracker.update_elements([])

        self.assertFalse(tracker.se_list)
        self.assertFalse(tracker.non_roi_gazes)

    def test_update_elements__all_gazes_inside__empty_non_roi_gazes(self):
        tracker = SituationElementTracker(self.mock, TEST_SA_PARAMETER)
        tracker.add_or_update_ses([create_roi(0, 0, radius=1)])

        tracker.update_elements([create_gaze(0, 0)])

        self.assertEqual(1, len(tracker.se_list))
        self.assertFalse(tracker.non_roi_gazes)

    def test_update_elements__no_gazes_inside__has_non_roi_gazes(self):
        tracker = SituationElementTracker(self.mock, TEST_SA_PARAMETER)
        tracker.add_or_update_ses([create_roi(0, 0, radius=1)])

        tracker.update_elements([create_gaze(1, 1)])

        self.assertEqual(1, len(tracker.se_list))
        self.assertEqual(1, len(tracker.non_roi_gazes))

    def test_update_elements__one_of_two_gazes_inside__one_non_roi_gaze(self):
        tracker = SituationElementTracker(self.mock, TEST_SA_PARAMETER)
        tracker.add_or_update_ses([create_roi(0, 0, radius=1)])
        tracker.add_or_update_ses([create_roi(2, 0, radius=1)])

        tracker.update_elements([create_gaze(0, 0), create_gaze(1, 1)])

        self.assertEqual(2, len(tracker.se_list))
        self.assertEqual(1, len(tracker.non_roi_gazes))

    def test_update_elements__gazes_and_no_ses__two_non_roi_gaze(self):
        tracker = SituationElementTracker(self.mock, TEST_SA_PARAMETER)

        tracker.update_elements([create_gaze(0, 0), create_gaze(1, 1)])

        self.assertFalse(tracker.se_list)
        self.assertEqual(2, len(tracker.non_roi_gazes))

    def test_update_best_candidate__no_candidates__nothing_changed(self):
        SituationElementTracker.update_best_candidate([], ROI())

        self.assertTrue(True)

    def test_update_best_candidate__one_candidate__candidate_updated(self):
        element = SituationElement(self.se_list[1].roi_msg, TEST_SA_PARAMETER)
        element.update(0)
        roi = create_roi(0.5, 0.5)

        SituationElementTracker.update_best_candidate([element], roi)

        self.assertEqual(element.roi_msg, roi)

    def test_update_best_candidate__one_candidate_changed__no_candidate_updated(
        self,
    ):
        element = SituationElement(self.se_list[1].roi_msg, TEST_SA_PARAMETER)
        roi = create_roi(0.5, 0.5)

        SituationElementTracker.update_best_candidate([element], roi)

        self.assertEqual(element.roi_msg, self.se_list[1].roi_msg)


class DistanceTrackingStrategyTest(unittest.TestCase):
    """Distance Tracking Strategy Test"""

    def setUp(self):
        self.se_list = [
            SituationElement(ROI(), TEST_SA_PARAMETER),
            SituationElement(create_roi(10, 5), TEST_SA_PARAMETER),
        ]
        self.unit = DistanceTrackingStrategy(1)

    def test_find_best_candidate__empty_se_list__empty_result_list(self):
        result = self.unit.find_best_candidates([], ROI())

        self.assertFalse(result)

    def test_find_best_candidate__se_list_perfect_match__one_result(self):
        roi = ROI()

        result = self.unit.find_best_candidates(self.se_list, roi)

        self.assertEqual(1, len(result))
        self.assertEqual(result[0].roi_msg, roi)

    def test_find_best_candidate__se_list_close_enough__one_result(self):
        roi = create_roi(0.5, 0.5)

        result = self.unit.find_best_candidates(self.se_list, roi)

        self.assertEqual(1, len(result))
        self.assertEqual(result[0].roi_msg, ROI())

    def test_find_best_candidate__se_list_too_far_away__empty_list(self):
        roi = create_roi(1.5, 1.5)

        result = self.unit.find_best_candidates(self.se_list, roi)

        self.assertFalse(result)

    def test_find_best_candidate__both_close_enough__two_results(self):
        roi = create_roi(5, 5)
        unit = DistanceTrackingStrategy(10)

        result = unit.find_best_candidates(self.se_list, roi)

        self.assertEqual(2, len(result))


class GazeBufferTest(unittest.TestCase):
    """Gaze Buffer Test"""

    def setUp(self):
        self.gaze_buffer = GazeBuffer()
        self.test_gaze = create_gaze(10, 20)

    def assert_gaze_equal(self):
        self.assertEqual(self.test_gaze.gaze_pixel.x, self.gaze_buffer.current_gaze[0])
        self.assertEqual(self.test_gaze.gaze_pixel.y, self.gaze_buffer.current_gaze[1])

    def test_constructor__empty_buffer_and_correct_gaze(self):
        self.assertFalse(self.gaze_buffer.data)
        self.assertEqual((0, 0), self.gaze_buffer.current_gaze)

    def test_push_array__two_elements__len_is_2_and_correct_gaze(self):
        self.gaze_buffer.push_array([Gaze(), self.test_gaze])

        self.assertEqual(2, len(self.gaze_buffer.data))
        self.assert_gaze_equal()

    def test_clear__empty_buffer_and_correct_gaze(self):
        self.gaze_buffer.push_array([Gaze(), self.test_gaze])

        self.gaze_buffer.clear()

        self.assertFalse(self.gaze_buffer.data)
        self.assert_gaze_equal()


class IdTrackingStrategyTest(unittest.TestCase):
    """Id Tracking Strategy Test"""

    def setUp(self):
        self.unit = IdTrackingStrategy()
        self.se_list = [
            SituationElement(ROI(), TEST_SA_PARAMETER),
            SituationElement(create_roi(10, 5, roi_id=1), TEST_SA_PARAMETER),
        ]

    def test_find_best_candidate__empty_se_list__empty_result_list(self):
        result = self.unit.find_best_candidates([], Object2D(3, None, None))

        self.assertFalse(result)

    def test_find_best_candidate__se_list_no_match__empty_result_list(self):
        result = self.unit.find_best_candidates(self.se_list, Object2D(3, None, None))

        self.assertFalse(result)

    def test_find_best_candidate__se_list_one_match__result_list_size_one(self):
        result = self.unit.find_best_candidates(self.se_list, Object2D(1, None, None))

        self.assertEqual(1, len(result))


class SATestSuite(unittest.TestSuite):
    """SA Test"""

    def __init__(self):
        super(SATestSuite, self).__init__()
        self.addTest(GazeBufferTest())
        self.addTest(SituationAwarenessTest())
        self.addTest(SituationElementTest())
        self.addTest(SituationElementTrackerTest())
        self.addTest(DistanceTrackingStrategyTest())
        self.addTest(IdTrackingStrategyTest())
        self.addTest(PunishmentModelTest())
