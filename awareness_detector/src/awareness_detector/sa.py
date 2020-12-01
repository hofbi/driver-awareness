"""Situation Awareness Module"""

import operator
import time

import enum
import numpy as np
from sklearn.cluster import DBSCAN
from driver_awareness_msgs.msg import SA, ROI


class SituationAwareness:
    """Class to calculate the situation awareness"""

    def __init__(self, punishment_model):
        self.__optimal_sa = 0
        self.__actual_sa = 0
        self.__punishment = punishment_model

    @property
    def sa_msg(self):
        sa_msg = SA()
        sa_msg.optimal_sa = self.__optimal_sa
        sa_msg.actual_sa = self.__actual_sa
        return sa_msg

    def calculate_sa(self, se_array, non_roi_gazes):
        type_weights = [situation_element.type_weight for situation_element in se_array]
        classifications = [
            situation_element.classification_value for situation_element in se_array
        ]
        self.__optimal_sa = sum(type_weights)
        actual_sa = sum(list(map(operator.mul, type_weights, classifications)))
        self.__actual_sa = actual_sa - self.__punishment.punishment(non_roi_gazes)
        self.__actual_sa = max(0.0, self.__actual_sa)


class PunishmentModel:
    """Class to calculate the punishment score"""

    def __init__(
        self, dbscan_max_distance_px, dbscan_min_samples, punishment_factor=0.25
    ):
        self.__dbscan = DBSCAN(
            eps=float(dbscan_max_distance_px),
            min_samples=dbscan_min_samples,
        )
        self.__punishment_factor = punishment_factor

    def punishment(self, non_roi_gazes):
        if not non_roi_gazes:
            return 0.0

        gaze_coordinates = [
            (gaze.gaze_pixel.x, gaze.gaze_pixel.y) for gaze in non_roi_gazes
        ]
        labels = self.__dbscan.fit_predict(gaze_coordinates)
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        return self.__punishment_factor * num_clusters


class GazeBuffer:
    """Buffer for gaze data"""

    def __init__(self):
        self.__data = []
        self.__current_gaze = (0, 0)

    @property
    def data(self):
        return self.__data

    @property
    def current_gaze(self):
        return self.__current_gaze

    def push_array(self, gaze_array):
        self.__data.extend(gaze_array)
        self.__current_gaze = (gaze_array[-1].gaze_pixel.x, gaze_array[-1].gaze_pixel.y)

    def clear(self):
        self.__data = []


class SituationAwarenessParameter:
    """Collection of Parameter for detection/comprehension decisions"""

    def __init__(
        self, comprehension_alive_time, num_gaze_comprehended, num_gaze_detected
    ):
        self.__comprehension_alive_time = comprehension_alive_time
        self.__num_gaze_comprehended = num_gaze_comprehended
        self.__num_gaze_detected = num_gaze_detected

    @property
    def num_gaze_detected(self):
        return self.__num_gaze_detected

    @property
    def num_gaze_comprehended(self):
        return self.__num_gaze_comprehended

    @property
    def comprehension_alive_time(self):
        return self.__comprehension_alive_time


class SituationElement:
    """Situation Element"""

    class TypeWeight:
        """Defines the type weights for situation elements"""

        REQUIRED = 2.0
        DESIRED = 1.0

    class ClassificationWeight:
        """Defines the classification weights for situation elements"""

        COMPREHENDED = 1.0
        DETECTED = 0.5
        UNDETECTED = 0.0

    class Classification(enum.Enum):
        """Defines classification types for situation elements"""

        COMPREHENDED = "comprehended"
        DETECTED = "detected"

    SE_ALIVE_COUNTER = 1

    def __init__(self, roi_msg, sa_parameter):
        self.__msg = roi_msg
        self.__sa_parameter = sa_parameter
        self.__changed = True
        self.__alive_counter = self.SE_ALIVE_COUNTER
        self.__time_last_looked = time.time()
        self.__num_gazes_inside = 0

    @property
    def roi_msg(self):
        return self.__msg

    @property
    def is_alive(self):
        return self.__alive_counter > 0

    @property
    def has_changed(self):
        return self.__changed

    def is_gaze_inside(self, gaze_data):
        return (
            pow(gaze_data.gaze_pixel.x - self.roi_msg.roi.center.x, 2)
            + pow(gaze_data.gaze_pixel.y - self.roi_msg.roi.center.y, 2)
        ) <= pow(self.roi_msg.roi.radius, 2)

    def is_comprehension_expired(self):
        return (
            time.time() - self.__time_last_looked
            > self.__sa_parameter.comprehension_alive_time
            and self.roi_msg.classification
            == SituationElement.Classification.COMPREHENDED
        )

    def update_msg(self, roi_msg):
        self.__msg.roi = roi_msg.roi
        self.__msg.type = roi_msg.type
        self.__changed = True
        self.__alive_counter = self.SE_ALIVE_COUNTER

    def update(self, num_gazes_inside):
        self.__time_last_looked = time.time()
        self.__num_gazes_inside += num_gazes_inside
        if self.__num_gazes_inside > self.__sa_parameter.num_gaze_comprehended:
            self.__msg.classification = SituationElement.Classification.COMPREHENDED
        elif self.__num_gazes_inside >= self.__sa_parameter.num_gaze_detected:
            self.__msg.classification = SituationElement.Classification.DETECTED
        if self.is_comprehension_expired():
            self.__num_gazes_inside = self.__sa_parameter.num_gaze_detected
            self.__msg.classification = SituationElement.Classification.DETECTED
        if self.__changed:
            self.__changed = False
            return
        self.__alive_counter -= 1

    @property
    def classification_value(self):
        if self.__msg.classification == self.Classification.COMPREHENDED:
            return self.ClassificationWeight.COMPREHENDED
        if self.__msg.classification == self.Classification.DETECTED:
            return self.ClassificationWeight.DETECTED
        return self.ClassificationWeight.UNDETECTED

    @property
    def type_weight(self):
        return (
            self.TypeWeight.DESIRED
            if self.__msg.type == ROI.DESIRED
            else self.TypeWeight.REQUIRED
        )

    def distance_to(self, roi_msg):
        roi_center = np.array([roi_msg.roi.center.x, roi_msg.roi.center.y])
        center = np.array([self.roi_msg.roi.center.x, self.roi_msg.roi.center.y])
        return np.linalg.norm(center - roi_center)


class SituationElementTracker:
    """Keeps track of the current SEs in the FOV"""

    def __init__(self, tracking_strategy, sa_parameter):
        self.__se_list = []
        self.__non_roi_gazes = []
        self.__tracking_strategy = tracking_strategy
        self.__sa_parameter = sa_parameter

    @property
    def se_list(self):
        return self.__se_list

    @property
    def non_roi_gazes(self):
        return self.__non_roi_gazes

    @staticmethod
    def update_best_candidate(candidates, roi_msg):
        for candidate in candidates:
            if not candidate.has_changed:
                candidate.update_msg(roi_msg)
                break

    def update_elements(self, gaze_points):
        """Update all SEs and remove if not alive"""
        gazes_inside_list = [[False] * len(gaze_points)]
        for tracking_object in self.__se_list:
            # TODO test for update called with ...
            gazes_inside = [
                tracking_object.is_gaze_inside(gaze) for gaze in gaze_points
            ]
            tracking_object.update(sum(gazes_inside))
            gazes_inside_list.append(gazes_inside)
        gazes_in_any_roi = np.any(np.array(gazes_inside_list), axis=0)
        self.__non_roi_gazes = list(np.array(gaze_points)[~gazes_in_any_roi])
        dead_elements = [
            tracking_object
            for tracking_object in self.__se_list
            if not tracking_object.is_alive
        ]
        for dead_element in dead_elements:
            self.__se_list.remove(dead_element)

    def add_or_update_ses(self, roi_array):
        """Add new SEs to the list of tracked elements or update if they already exist"""
        for roi_msg in roi_array:
            best_candidates = self.__tracking_strategy.find_best_candidates(
                self.__se_list, roi_msg
            )
            if best_candidates:
                self.update_best_candidate(best_candidates, roi_msg)
            else:
                self.__se_list.append(SituationElement(roi_msg, self.__sa_parameter))


class DistanceTrackingStrategy:
    """Distance based SE tracking strategy"""

    def __init__(self, max_tracking_distance):
        self.__max_tracking_distance = max_tracking_distance

    def find_best_candidates(self, se_list, roi_msg):
        return [
            current_se
            for current_se in se_list
            if current_se.distance_to(roi_msg) < self.__max_tracking_distance
        ]


class IdTrackingStrategy:
    """Id based SE tracking strategy"""

    @staticmethod
    def find_best_candidates(se_list, roi_msg):
        return [
            current_se for current_se in se_list if current_se.roi_msg.id == roi_msg.id
        ]
