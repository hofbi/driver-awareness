#!/usr/bin/env python

"""Geometry Test"""

import unittest
from unittest.mock import MagicMock

from derived_object_msgs.msg import Object
from geometry_msgs.msg import Point
from opencv_apps.msg import Point2D, Rect

from awareness_detector.geometry import (
    BoundingBoxConverter,
    FieldOfView,
    Object2D,
    ObjectFilter,
)
from driver_awareness_msgs.msg import ROI


def create_object(
    obj_id=0, classification=Object.CLASSIFICATION_UNKNOWN, x_pos=0, y_pos=0, z_pos=0
):
    """Create Object with predefined parameters"""
    object_msg = Object()
    object_msg.id = obj_id
    object_msg.pose.position = Point(x_pos, y_pos, z_pos)
    object_msg.classification = classification
    object_msg.shape.dimensions = [2, 2, 2]
    return object_msg


def create_roi(x_pos, y_pos, radius):
    """Create ROI from coordinates"""
    roi = ROI()
    roi.roi.center = Point2D(x_pos, y_pos)
    roi.roi.radius = radius
    return roi


class FieldOfViewTest(unittest.TestCase):
    """FoV Test"""

    def setUp(self):
        self.__cam_model = MagicMock()
        self.__cam_model.width = 640
        self.__cam_model.height = 480
        self.__unit = FieldOfView(90, self.__cam_model)

    def test_constructor__angle_zero__zero(self):
        unit = FieldOfView(0, self.__cam_model)

        self.assertEqual(0, unit.horizontal)
        self.assertEqual(0, unit.vertical)

    def test_constructor__angle_90__correct(self):
        self.assertEqual(90, self.__unit.horizontal)
        self.assertEqual(73, self.__unit.vertical)

    def test_is_point_in_fov__negative_z__false(self):
        self.assertFalse(self.__unit.is_point_in_fov(-1, 0, 90))

    def test_is_point_in_fov__not_inside__false(self):
        self.assertFalse(self.__unit.is_point_in_fov(1, 10, 90))

    def test_is_point_in_fov__inside__true(self):
        self.assertTrue(self.__unit.is_point_in_fov(1, 1, 90))

    def test_is_point_inside__not_inside__false(self):
        self.assertFalse(self.__unit.is_point_inside(Point(10, 10, 2)))
        self.assertFalse(self.__unit.is_point_inside(Point(10, 1, 2)))
        self.assertFalse(self.__unit.is_point_inside(Point(1, 10, 2)))

    def test_is_point_inside__inside__true(self):
        self.assertTrue(self.__unit.is_point_inside(Point(1, 1, 2)))


class ObjectFilterTest(unittest.TestCase):
    """Object Filter Test"""

    def setUp(self):
        self.__fov_mock = MagicMock()
        self.__fov_mock.is_point_inside.return_value = True
        self.__filter = ObjectFilter(
            5,
            self.__fov_mock,
            1,
            {
                Object.CLASSIFICATION_PEDESTRIAN: "/walker/",
                Object.CLASSIFICATION_CAR: "/vehicle/",
            },
        )

    def test_get_topic_for_object__car__vehicle(self):
        object_msg = create_object(classification=Object.CLASSIFICATION_CAR)

        result = self.__filter.get_topic_for_object(object_msg)

        self.assertEqual("/vehicle/0", result)

    def test_get_topic_for_object__pedestrian__walker(self):
        object_msg = create_object(classification=Object.CLASSIFICATION_PEDESTRIAN)

        result = self.__filter.get_topic_for_object(object_msg)

        self.assertEqual("/walker/0", result)

    def test_get_topic_for_object__unknown__key_error(self):
        with self.assertRaises(KeyError):
            self.__filter.get_topic_for_object(create_object())

    def test_type_filter__empty_list__result_empty(self):
        result = self.__filter.type_filter([])

        self.assertFalse(result)

    def test_type_filter__vehicle_and_pedestrian__list_unchanged(self):
        object_list = [
            create_object(2, Object.CLASSIFICATION_CAR),
            create_object(3, Object.CLASSIFICATION_PEDESTRIAN),
        ]

        result = self.__filter.type_filter(object_list)

        self.assertEqual(result, object_list)

    def test_type_filter__ego_vehicle_and_pedestrian__pedestrian_left(self):
        pedestrian = create_object(3, Object.CLASSIFICATION_PEDESTRIAN)
        object_list = [create_object(1, Object.CLASSIFICATION_CAR), pedestrian]

        result = self.__filter.type_filter(object_list)

        self.assertEqual(1, len(result))
        self.assertEqual(pedestrian, result[0])

    def test_type_filter__vehicle_and_unknown__vehicle_left(self):
        vehicle = create_object(2, Object.CLASSIFICATION_CAR)
        object_list = [vehicle, create_object(3, Object.CLASSIFICATION_UNKNOWN)]

        result = self.__filter.type_filter(object_list)

        self.assertEqual(1, len(result))
        self.assertEqual(vehicle, result[0])

    def test_geometry_filter__empty_list__result_empty(self):
        result = self.__filter.geometry_filter([])

        self.assertFalse(result)

    def test_type_filter__two_in_range__list_unchanged(self):
        object_list = [
            create_object(x_pos=1),
            create_object(x_pos=2),
        ]

        result = self.__filter.geometry_filter(object_list)

        self.assertEqual(result, object_list)

    def test_type_filter__one_in_and_one_out_of_range__one_left(self):
        in_range = create_object(x_pos=2)
        object_list = [
            create_object(x_pos=6),
            in_range,
        ]

        result = self.__filter.geometry_filter(object_list)

        self.assertEqual(1, len(result))
        self.assertEqual(in_range, result[0])

    def test_type_filter__not_in_fov__empty(self):
        self.__fov_mock.is_point_inside.return_value = False

        result = self.__filter.geometry_filter([create_object()])

        self.assertFalse(result)


class Object2DTest(unittest.TestCase):
    """Object 2D Test"""

    def test_to_roi_msg__square__correct_conversion(self):
        rect = Rect(0, 0, 1, 1)
        unit = Object2D(1, rect, ROI.DESIRED)

        result = unit.to_roi_msg()

        self.assertEqual(1, result.id)
        self.assertEqual(ROI.DESIRED, result.type)
        self.assertEqual(0.5, result.roi.radius)
        self.assertEqual(rect.x, result.roi.center.x)
        self.assertEqual(rect.y, result.roi.center.y)

    def test_to_roi_msg__rectangle__correct_conversion(self):
        rect = Rect(2, 3, 2, 4)
        unit = Object2D(10, rect, ROI.REQUIRED)

        result = unit.to_roi_msg()

        self.assertEqual(10, result.id)
        self.assertEqual(ROI.REQUIRED, result.type)
        self.assertEqual(2, result.roi.radius)
        self.assertEqual(rect.x, result.roi.center.x)
        self.assertEqual(rect.y, result.roi.center.y)

    def test_from_2d_point_list__larger_than_min(self):
        object_2d = Object2D.from_2d_point_list(
            1, [[0, 0], [5, 3], [10, 8]], ROI.REQUIRED, [1, 1]
        )

        result = object_2d.to_roi_msg()
        self.assertEqual(1, result.id)
        self.assertEqual(ROI.REQUIRED, result.type)
        self.assertEqual(5, result.roi.radius)
        self.assertEqual(5, result.roi.center.x)
        self.assertEqual(4, result.roi.center.y)
        self.assertEqual(Rect(5, 4, 10, 8), object_2d.bb_2d)

    def test_from_2d_point_list__smaller_than_min(self):
        object_2d = Object2D.from_2d_point_list(
            1, [[0, 0], [5, 3], [10, 8]], ROI.REQUIRED, [12, 12]
        )

        result = object_2d.to_roi_msg()
        self.assertEqual(1, result.id)
        self.assertEqual(ROI.REQUIRED, result.type)
        self.assertEqual(6, result.roi.radius)
        self.assertEqual(5, result.roi.center.x)
        self.assertEqual(4, result.roi.center.y)
        self.assertEqual(Rect(5, 4, 12, 12), object_2d.bb_2d)


class BoundingBoxConverterTest(unittest.TestCase):
    """Bounding Box Converter Test"""

    def setUp(self):
        cam_model = MagicMock()
        cam_model.width = 640
        cam_model.height = 480
        cam_model.project3dToPixel.side_effect = lambda x: (20, 40)
        tf_listener = MagicMock()
        tf_listener.transformPoint.side_effect = lambda _, x: x
        self.__unit = BoundingBoxConverter(tf_listener, cam_model, 90, 3, 2, 30)

    def test_convert__all_points_in_fov_and_close(self):
        object_msg = create_object(1, z_pos=2)

        result = self.__unit.convert(object_msg, "")

        self.assertEqual(result.id, object_msg.id)
        self.assertEqual(result.to_roi_msg().type, ROI.REQUIRED)
        self.assertEqual(result.to_roi_msg().roi, create_roi(20, 40, 1.5).roi)

    def test_convert__all_points_in_fov_and_far(self):
        object_msg = create_object(1, z_pos=40)

        result = self.__unit.convert(object_msg, "")

        self.assertEqual(result.id, object_msg.id)
        self.assertEqual(result.to_roi_msg().type, ROI.DESIRED)
        self.assertEqual(result.to_roi_msg().roi, create_roi(20, 40, 1.5).roi)

    def test_get_3d_corners_from_shape_dimensions__vehicle(self):
        dimensions = [2, 2, 2]

        result = self.__unit.get_3d_corners_from_shape_dimensions(
            dimensions, Object.CLASSIFICATION_CAR
        )

        self.assertEqual(8, len(result))
        self.assertEqual(Point(1.0, 1.0, 2.0), result[0])
        self.assertEqual(Point(-1.0, -1.0, 0.0), result[-1])

    def test_get_3d_corners_from_shape_dimensions__pedestrian(self):
        dimensions = [2, 2, 2]

        result = self.__unit.get_3d_corners_from_shape_dimensions(
            dimensions, Object.CLASSIFICATION_PEDESTRIAN
        )

        self.assertEqual(8, len(result))
        self.assertEqual(Point(1.0, 1.0, 1.0), result[0])
        self.assertEqual(Point(-1.0, -1.0, -1.0), result[-1])

    def test_project_point_on_fov__multiple_cases(self):
        self.assertAlmostEqual(0, self.__unit.project_point_on_fov(0, 90))
        self.assertAlmostEqual(180, self.__unit.project_point_on_fov(180, 90))
        self.assertAlmostEqual(10, self.__unit.project_point_on_fov(10, 90))
        self.assertAlmostEqual(20, self.__unit.project_point_on_fov(20, 90))


class GeometryTestSuite(unittest.TestSuite):
    """Geometry Test"""

    def __init__(self):
        super().__init__()
        self.addTest(FieldOfViewTest())
        self.addTest(ObjectFilterTest())
        self.addTest(Object2DTest())
        self.addTest(BoundingBoxConverter())
