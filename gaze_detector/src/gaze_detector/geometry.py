"""Geometry Module"""

import numpy as np
from geometry_msgs.msg import Point
from scipy import spatial


def norm_to_pixel(normalized_point, res_x, res_y):
    """Convert normalized point to point in pixel coordinates"""
    if (
        normalized_point.x > 1
        or normalized_point.x < 0
        or normalized_point.y > 1
        or normalized_point.y < 0
    ):
        raise ValueError("Invalid value for normalized coordinates")

    return Point(x=normalized_point.x * res_x, y=res_y - (normalized_point.y * res_y))


def find_closest_circle_distances(circles, gazes):
    """Calculate the mean distance for gaze points inside a circle"""
    kd_tree = spatial.KDTree([circle.position for circle in circles])

    return [
        kd_tree.query([gaze_x, gaze_y])
        for gaze_x, gaze_y in zip(gazes.gaze_x, gazes.gaze_x)
    ]


class PygameColor:
    """Pygame Color Definitions"""

    WHITE = (255, 255, 255)
    RED = (255, 0, 0)


class Circle:
    """Defines a circle for data accuracy measurement"""

    def __init__(self, x_pos, y_pos, radius):
        self.__circle_x = x_pos
        self.__circle_y = y_pos
        self.__circle_r = radius
        self.__dist_gaze_inside = []

    @staticmethod
    def from_pd(series):
        return Circle(series["x"], series["y"], series["radius"])

    @property
    def position(self):
        return self.circle_x, self.circle_y

    @property
    def circle_x(self):
        return self.__circle_x

    @property
    def circle_y(self):
        return self.__circle_y

    def append_gaze_inside_dist(self, dist):
        if self.is_gaze_inside(dist):
            self.__dist_gaze_inside.append(dist)

    def is_gaze_inside(self, gaze_distance):
        return gaze_distance < self.__circle_r

    def mean_dist(self):
        return (
            sum(self.__dist_gaze_inside) / len(self.__dist_gaze_inside)
            if self.has_gazes_inside()
            else 0
        )

    def has_gazes_inside(self):
        return bool(self.__dist_gaze_inside)


class GazeData:
    """Defines a gaze data list"""

    def __init__(self):
        self.__gaze_x = []
        self.__gaze_y = []
        self.__on_surface = []

    def append(self, data):
        self.__gaze_x.append(data.gaze_pixel.x)
        self.__gaze_y.append(data.gaze_pixel.y)
        self.__on_surface.append(data.on_surface)

    @property
    def gaze_x(self):
        return self.__gaze_x

    @property
    def gaze_y(self):
        return self.__gaze_y

    @property
    def on_surface(self):
        return self.__on_surface

    def to_numpy_array(self):
        return np.array([self.gaze_x, self.gaze_y, self.on_surface]).transpose()
