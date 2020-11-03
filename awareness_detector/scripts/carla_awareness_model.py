#!/usr/bin/env python
""" Detects which objects were recognized by the operator """

import image_geometry
import rospy
import tf
from awareness_detector.geometry import ObjectFilter, BoundingBoxConverter, FieldOfView
from carla_msgs.msg import CarlaEgoVehicleInfo
from cv_bridge import CvBridgeError
from derived_object_msgs.msg import ObjectArray, Object
from sensor_msgs.msg import CameraInfo, Image
from driver_awareness_msgs.msg import ROIArray, SA, GazeArray
from awareness_detector.sa import (
    SituationElement,
    SituationAwareness,
    GazeBuffer,
    PunishmentModel,
    SituationElementTracker,
    SituationAwarenessParameter,
    IdTrackingStrategy,
)
from awareness_detector.view import ImageVisualization


class ObjectAwarenessModel:
    """ Detects which objects were recognized by the operator """

    def __init__(self):
        rospy.loginfo("Starting CARLA Awareness Model...")

        gaze_topic = rospy.get_param("~gaze_topic", "/gaze_publisher/gaze")
        ego_vehicle_name = rospy.get_param("~ego_vehicle_name", "ego_vehicle")
        max_object_distance = rospy.get_param("~objects/max_distance_m", 100)
        roi_required_object_distance = rospy.get_param(
            "~objects/roi_required_distance_m", 30
        )
        self.__offset_topbar_px = rospy.get_param("~offset_topbar_px", 30)
        camera_fov_deg = rospy.get_param("~camera_fov_deg", 90)
        min_2d_boundingbox_x = rospy.get_param("~min_2d_boundingbox/x_px", 90)
        min_2d_boundingbox_y = rospy.get_param("~min_2d_boundingbox/y_px", 90)
        time_last_looked_detected_s = rospy.get_param(
            "~awareness_detection/time_last_looked_detected_s", 5
        )
        num_gaze_comprehended = rospy.get_param(
            "~awareness_detection/num_gaze_comprehended_s", 35
        )
        dbscan_max_distance_px = rospy.get_param(
            "~awareness_detection/dbscan_max_distance_px", 15
        )
        dbscan_min_samples = rospy.get_param(
            "~awareness_detection/dbscan_min_samples", 3
        )
        punishment_factor = rospy.get_param(
            "~awareness_detection/punishment_factor", 0.25
        )

        rospy.loginfo(
            "Waiting for vehicle_info and camera_info topics from carla ros bridge"
        )
        ego_vehicle_info = rospy.wait_for_message(
            f"/carla/{ego_vehicle_name}/vehicle_info", CarlaEgoVehicleInfo
        )
        camera_info = rospy.wait_for_message(
            f"/carla/{ego_vehicle_name}/camera/rgb/front/camera_info", CameraInfo
        )
        self.__camera_tf_name = f"/{ego_vehicle_name}/camera/rgb/front"
        rospy.loginfo("Carla ros bridge is online.")

        # Subscribers
        self.__sub_gaze = rospy.Subscriber(gaze_topic, GazeArray, self.gaze_callback)
        self.__sub_objects = rospy.Subscriber(
            "/carla/objects", ObjectArray, self.objects_callback
        )

        # Create camera model from camera info
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(camera_info)

        self.__listener = tf.TransformListener()
        self.__bounding_box_converter = BoundingBoxConverter(
            self.__listener,
            cam_model,
            camera_fov_deg,
            min_2d_boundingbox_x,
            min_2d_boundingbox_y,
            roi_required_object_distance,
        )

        # Data variables
        self.__gaze_buffer = GazeBuffer()
        punishment_model = PunishmentModel(
            dbscan_max_distance_px, dbscan_min_samples, punishment_factor
        )
        self.__situation_awareness = SituationAwareness(punishment_model)
        self.__roi_tracker = SituationElementTracker(
            IdTrackingStrategy(),
            SituationAwarenessParameter(
                time_last_looked_detected_s, num_gaze_comprehended, 1
            ),
        )
        self.__object_filter = ObjectFilter(
            max_object_distance,
            FieldOfView(camera_fov_deg, cam_model),
            ego_vehicle_info.id,
            {
                Object.CLASSIFICATION_PEDESTRIAN: "/walker/",
                Object.CLASSIFICATION_BIKE: "/vehicle/",
                Object.CLASSIFICATION_CAR: "/vehicle/",
                Object.CLASSIFICATION_TRUCK: "/vehicle/",
                Object.CLASSIFICATION_MOTORCYCLE: "/vehicle/",
                Object.CLASSIFICATION_OTHER_VEHICLE: "/vehicle/",
            },
        )
        # Publishers
        self.__roi_pub = rospy.Publisher("~roi", ROIArray, queue_size=10)
        self.__sa_pub = rospy.Publisher("~sa", SA, queue_size=10)

        # Visualization
        if rospy.get_param("~visualize", True):
            self.__image_visualization = ImageVisualization(
                None,
                False,
                True,
            )
            rospy.loginfo("Visualization enabled.")
            self.__image_pub = rospy.Publisher("~image", Image, queue_size=10)
            self.__sub_camera = rospy.Subscriber(
                f"/carla/{ego_vehicle_name}/camera/rgb/front/image_color/compressed",
                Image,
                self.image_callback,
            )

        rospy.loginfo("CARLA Awareness Model started.")

    def gaze_callback(self, gaze_data):
        # Subtract top bar because simulator is not fullscreen
        for gaze in gaze_data.gazes:
            gaze.gaze_pixel.y -= self.__offset_topbar_px
        self.__gaze_buffer.push_array(gaze_data.gazes)

    def transform_objects(self, object_msgs):
        """Transform objects to camera coordinate system"""
        transformed_objects = []
        for object_msg in object_msgs:
            object_topic = self.__object_filter.get_topic_for_object(object_msg)
            if self.__listener.canTransform(
                self.__camera_tf_name, object_topic, rospy.Time(0)
            ):
                try:
                    (
                        [
                            object_msg.pose.position.x,
                            object_msg.pose.position.y,
                            object_msg.pose.position.z,
                        ],
                        [
                            object_msg.pose.orientation.x,
                            object_msg.pose.orientation.y,
                            object_msg.pose.orientation.z,
                            object_msg.pose.orientation.w,
                        ],
                    ) = self.__listener.lookupTransform(
                        self.__camera_tf_name, object_topic, rospy.Time(0)
                    )
                    transformed_objects.append(object_msg)
                except ():
                    pass
        return transformed_objects

    def objects_callback(self, object_array):
        """ Callback for carla objects, position of objects in image plane is calculated here """
        filtered_objects = self.__object_filter.type_filter(object_array.objects)
        filtered_objects = self.transform_objects(filtered_objects)
        filtered_objects = self.__object_filter.geometry_filter(filtered_objects)
        objects_2d = [
            self.__bounding_box_converter.convert(
                obj, self.__object_filter.get_topic_for_object(obj)
            )
            for obj in filtered_objects
        ]

        self.__roi_tracker.add_or_update_ses(objects_2d)
        self.__roi_tracker.update_elements(self.__gaze_buffer.data)
        self.__situation_awareness.calculate_sa(
            self.__roi_tracker.se_list, self.__roi_tracker.non_roi_gazes
        )

        self.__gaze_buffer.clear()
        self.__sa_pub.publish(self.__situation_awareness.sa_msg)

        roi_array = ROIArray()
        roi_array.rois = [element.roi_msg for element in self.__roi_tracker.se_list]
        self.__roi_pub.publish(roi_array)

    def image_callback(self, image):
        """ Callback for camera image from carla, objects and their recognition is visualized here """
        try:
            img = self.__image_visualization.get_img_from_msg(image)
        except CvBridgeError as error:
            rospy.logerr(error)
            return

        self.__image_visualization.display_debug_visualization(
            img, self.__gaze_buffer.current_gaze, self.__roi_tracker.se_list
        )

        self.__image_pub.publish(self.__image_visualization.image_to_msg(img))


def main():
    """Main"""
    rospy.init_node("carla_awareness_model", anonymous=True)
    rospy.Rate(20)

    ObjectAwarenessModel()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
