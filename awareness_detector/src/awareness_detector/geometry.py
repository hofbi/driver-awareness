#!/usr/bin/env python
""" Converts 3D bounding boxes from carla objects to 2d bounding boxes in camera frame """
from __future__ import division
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, Point
from driver_awareness_msgs.msg import ROI
from opencv_apps.msg import Rect, Point2D
from derived_object_msgs.msg import Object
from std_msgs.msg import Header


class ObjectFilter:
    """Filter objects based on several requirements"""

    def __init__(self, max_distance, fov, ego_id, classifications):
        self.__distance = max_distance
        self.__fov = fov
        self.__ego_id = ego_id
        self.__classifications = classifications

    def type_filter(self, object_msgs):
        return [
            object_msg
            for object_msg in object_msgs
            if self.__ego_id != object_msg.id
            and object_msg.classification in self.__classifications.keys()
        ]

    def geometry_filter(self, object_msgs):
        return [
            object_msg
            for object_msg in object_msgs
            if np.linalg.norm(
                [
                    object_msg.pose.position.x,
                    object_msg.pose.position.y,
                    object_msg.pose.position.z,
                ]
            )
            < self.__distance
            and self.__fov.is_point_inside(object_msg.pose.position)
        ]

    def get_topic_for_object(self, object_msg):
        return self.__classifications[object_msg.classification] + str(object_msg.id)


class FieldOfView:
    """Field of view definition"""

    def __init__(self, horizontal_degree, cam_model):
        self.__horizontal = horizontal_degree
        self.__vertical = int(
            np.degrees(
                2
                * np.arctan(
                    np.tan(np.radians(self.__horizontal / 2.0))
                    * (cam_model.height / cam_model.width)
                )
            )
        )

    @property
    def horizontal(self):
        return self.__horizontal

    @property
    def vertical(self):
        return self.__vertical

    def is_point_inside(self, point):
        return self.is_point_in_fov(
            point.z, point.x, self.__horizontal
        ) and self.is_point_in_fov(point.z, point.y, self.__vertical)

    @staticmethod
    def is_point_in_fov(point_z, axis, fov):
        if point_z < 0:
            return False
        distance_to_object = np.linalg.norm([axis, point_z])
        return (
            np.arccos(np.fabs(axis) / distance_to_object)
            > (np.pi - np.radians(fov)) / 2.0
        )


class Object2D:
    """2D Object class"""

    def __init__(self, obj_id, rect, sa_type):
        self.__id = obj_id
        self.__rect = rect
        self.__type = sa_type

    @property
    def id(self):
        return self.__id

    @property
    def bb_2d(self):
        return self.__rect

    def to_roi_msg(self):
        roi = ROI(id=self.__id, type=self.__type)
        roi.roi.radius = max(self.__rect.width / 2.0, self.__rect.height / 2.0)
        roi.roi.center = Point2D(x=self.__rect.x, y=self.__rect.y)
        return roi

    @staticmethod
    def from_2d_point_list(obj_id, point_list, sa_type, min_size):
        top_left_x = int(min([x[0] for x in point_list]))
        top_left_y = int(min([x[1] for x in point_list]))
        low_right_x = int(max([x[0] for x in point_list]))
        low_right_y = int(max([x[1] for x in point_list]))

        width = low_right_x - top_left_x
        height = low_right_y - top_left_y
        center_x = top_left_x + width / 2
        center_y = top_left_y + height / 2
        width = max(width, min_size[0])
        height = max(height, min_size[1])

        rect = Rect(center_x, center_y, width, height)
        return Object2D(obj_id, rect, sa_type)


class BoundingBoxConverter:
    """ Converts 3D bounding boxes from carla objects to 2d bounding boxes in camera frame """

    def __init__(
        self, tf_listener, cam_model, cam_fov_deg, min_x, min_y, type_required_distance
    ):
        self.__tf_listener = tf_listener
        self.__cam_model = cam_model
        self.__fov = FieldOfView(cam_fov_deg, cam_model)
        self.__min_size = (min_x, min_y)
        self.__type_required_distance = type_required_distance

    def convert(self, object_msg, object_topic):
        bb_3d = self.get_3d_corners_from_shape_dimensions(
            object_msg.shape.dimensions, object_msg.classification
        )
        bb_2d = self.__transform_object_to_image_plane(bb_3d, object_topic)
        sa_type = self.__get_sa_type_for_object(object_msg.pose.position)
        return Object2D.from_2d_point_list(
            object_msg.id, bb_2d, sa_type, self.__min_size
        )

    def __get_sa_type_for_object(self, object_position):
        return (
            ROI.REQUIRED
            if (
                np.linalg.norm(
                    [object_position.x, object_position.y, object_position.z]
                )
                < self.__type_required_distance
            )
            else ROI.DESIRED
        )

    @staticmethod
    def get_3d_corners_from_shape_dimensions(obj_shape_dimensions, classification):
        """Create 3d bounding box points in object frame"""
        x_dim = 0.5 * obj_shape_dimensions[0]
        y_dim = 0.5 * obj_shape_dimensions[1]
        z_dim = 0.5 * obj_shape_dimensions[2]
        z_bottom = -z_dim if classification == Object.CLASSIFICATION_PEDESTRIAN else 0
        z_top = z_bottom + 2 * z_dim

        return [
            Point(x_dim, y_dim, z_top),
            Point(x_dim, -y_dim, z_top),
            Point(-x_dim, y_dim, z_top),
            Point(-x_dim, -y_dim, z_top),
            Point(x_dim, y_dim, z_bottom),
            Point(x_dim, -y_dim, z_bottom),
            Point(-x_dim, y_dim, z_bottom),
            Point(-x_dim, -y_dim, z_bottom),
        ]

    @staticmethod
    def project_point_on_fov(axis, fov):
        """ Projects a point lying outside the fov onto the fov border """
        beta = (np.pi - np.radians(fov)) / 2.0
        point_distance_fov = axis / np.cos(beta)
        return np.sin(beta) * point_distance_fov

    def __transform_object_to_image_plane(self, points_3d, object_topic):
        """' Transforms 3d points in object frame to 2d points in the image plane """
        header = Header(frame_id=object_topic, stamp=rospy.Time(0))
        stamped_points = [
            PointStamped(header=header, point=point) for point in points_3d
        ]
        points_in_cam_frame = [
            self.__tf_listener.transformPoint(f"/{self.__cam_model.tf_frame}", point)
            for point in stamped_points
        ]

        for point in points_in_cam_frame:
            # Check if point is in both fov, horizontal and vertical
            if not self.__fov.is_point_inside(point.point):
                # project z on both FOVs and take bigger projection value
                fov_z1 = self.project_point_on_fov(point.point.x, self.__fov.horizontal)
                fov_z2 = self.project_point_on_fov(point.point.y, self.__fov.vertical)
                point.point.z = np.max([np.fabs(fov_z1), np.fabs(fov_z2)])

        return [
            self.__cam_model.project3dToPixel(
                [
                    point.point.x,
                    point.point.y,
                    point.point.z,
                ]
            )
            for point in points_in_cam_frame
        ]
