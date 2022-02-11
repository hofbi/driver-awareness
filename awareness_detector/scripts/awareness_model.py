#!/usr/bin/env python
""" Detects which objects were recognized by the operator based on a free awareness model"""
import cv2
import rospy
from cv_bridge import CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

from awareness_detector.sa import (
    DistanceTrackingStrategy,
    GazeBuffer,
    PunishmentModel,
    SituationAwareness,
    SituationAwarenessParameter,
    SituationElementTracker,
)
from awareness_detector.view import (
    CameraAdjustment,
    ImageVisualization,
    ScreenParameter,
)
from driver_awareness_msgs.msg import SA, GazeArray, ROIArray


class AwarenessModel:
    """Detects which ROIs were recognized by the operator using a free awareness model"""

    def __init__(self):
        rospy.loginfo("Starting Awareness model...")

        gaze_topic = rospy.get_param("~gaze_topic", "/gaze_publisher/gaze")
        roi_topic = rospy.get_param(
            "~roi_topic", "/carla/ego_vehicle/camera/rgb/front/rois"
        )

        self.__screen_parameter = ScreenParameter(
            rospy.get_param("~screen/padding/top_px", 0),
            rospy.get_param("~screen/padding/left_px", 0),
            rospy.get_param("~screen/padding/right_px", 0),
            rospy.get_param("~screen/padding/bottom_px", 0),
            rospy.get_param("~screen/window_pos/x_px", 1920),
            rospy.get_param("~screen/window_pos/y_px", 0),
            rospy.get_param("~screen/monitor_resolution/x_px", 1920),
            rospy.get_param("~screen/monitor_resolution/y_px", 1080),
            rospy.get_param("~screen/camera_resolution/x_px", 640),
            rospy.get_param("~screen/camera_resolution/y_px", 480),
        )
        time_last_looked_detected_s = rospy.get_param(
            "~awareness_detection/time_last_looked_detected_s", 5
        )
        tracker_roi_max_distance_px = rospy.get_param(
            "~awareness_detection/tracker_roi_max_distance_px", 50
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

        # Data variables
        self.__camera_adjustment = CameraAdjustment(self.__screen_parameter)
        self.__gaze_buffer = GazeBuffer()
        punishment_model = PunishmentModel(
            dbscan_max_distance_px, dbscan_min_samples, punishment_factor
        )
        self.__situation_awareness = SituationAwareness(punishment_model)
        self.__roi_tracker = SituationElementTracker(
            DistanceTrackingStrategy(tracker_roi_max_distance_px),
            SituationAwarenessParameter(
                time_last_looked_detected_s, num_gaze_comprehended, 1
            ),
        )

        # Subscribers
        self.__sub_gaze = rospy.Subscriber(gaze_topic, GazeArray, self.gaze_callback)
        self.__sub_rois = rospy.Subscriber(roi_topic, ROIArray, self.roi_callback)

        # Publishers
        self.__pub_sa = rospy.Publisher("~sa", SA, queue_size=10)

        # Visualization
        if rospy.get_param("~visualize", True):
            compress = rospy.get_param("~compress", False)
            self.__image_visualization = ImageVisualization(
                self.__screen_parameter,
                compress,
                rospy.get_param("~visualize_debug", False),
            )
            rospy.loginfo("Visualization enabled.")
            self.__image_pub = rospy.Publisher("~image", Image, queue_size=10)
            topic = rospy.get_param(
                "~image_topic", "/carla/ego_vehicle/camera/rgb/front/image_color"
            )
            msg_type = CompressedImage if compress else Image
            if compress:
                topic += "/compressed"
            self.__sub_camera = rospy.Subscriber(topic, msg_type, self.image_callback)

        rospy.loginfo("Awareness model started.")

    def gaze_callback(self, gaze_data):
        """Callback for gaze data points"""
        for gaze in gaze_data.gazes:
            self.__camera_adjustment.adjust_gaze_data_to_camera_resolution(gaze)
        self.__gaze_buffer.push_array(gaze_data.gazes)

    def roi_callback(self, roi_array):
        """Callback for roi array messages"""
        self.__roi_tracker.add_or_update_ses(roi_array.rois)
        self.__roi_tracker.update_elements(self.__gaze_buffer.data)
        self.__situation_awareness.calculate_sa(
            self.__roi_tracker.se_list, self.__roi_tracker.non_roi_gazes
        )

        self.__gaze_buffer.clear()
        self.__pub_sa.publish(self.__situation_awareness.sa_msg)

    def image_callback(self, image):
        """Callback for camera image from carla, objects and their recognition is visualized here"""
        try:
            img = self.__image_visualization.get_image_from_msg(image)
        except CvBridgeError as error:
            rospy.logerr(error)
            return

        self.__image_visualization.display_debug_visualization(
            img, self.__gaze_buffer.current_gaze, self.__roi_tracker.se_list
        )
        self.__image_visualization.display_image(img)

        cv2.waitKey(3)
        self.__image_pub.publish(self.__image_visualization.image_to_msg(img))


def main():
    """Main"""
    rospy.init_node("awareness_model", anonymous=True)
    rospy.Rate(20)

    AwarenessModel()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
