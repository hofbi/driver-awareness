"""View model responsible for visualization"""

import cv2
from cv_bridge import CvBridge
import numpy as np

from driver_awareness_msgs.msg import ROI
from awareness_detector.sa import SituationElement


def draw_current_gaze(img, current_gaze):
    """Draws the current gaze as circle into an open cv image"""
    cv2.circle(
        img,
        (int(current_gaze[0]), int(current_gaze[1])),
        2,
        Color.BLUE,
        thickness=7,
        lineType=8,
        shift=0,
    )


def get_color_from_classification(roi_msg):
    """Get the color from the ROI classification state"""
    if roi_msg.classification == SituationElement.Classification.COMPREHENDED:
        return Color.GREEN
    if roi_msg.classification == SituationElement.Classification.DETECTED:
        return Color.YELLOW

    return Color.RED


def draw_roi(img, roi_msg):
    """Draws roi from roi_msg as circle into open cv image, color is based on classification state"""
    cv2.circle(
        img,
        (int(roi_msg.roi.center.x), int(roi_msg.roi.center.y)),
        int(roi_msg.roi.radius),
        get_color_from_classification(roi_msg),
        thickness=2,
        lineType=8,
        shift=0,
    )
    draw_roi_text(img, roi_msg)


def draw_roi_text(img, roi_msg):
    """Draw roi text"""
    roi_label = "desired" if roi_msg.type == ROI.DESIRED else "required"
    cv2.putText(
        img,
        roi_label,
        (
            int(roi_msg.roi.center.x),
            int(roi_msg.roi.center.y),
        ),
        cv2.FONT_HERSHEY_COMPLEX,
        0.5,
        get_color_from_classification(roi_msg),
    )


class Color:
    """Defines opencv colors in bgr format"""

    GREEN = (0, 255, 0)
    RED = (0, 0, 255)
    YELLOW = (10, 195, 252)
    BLUE = (255, 0, 0)
    BLACK = (0, 0, 0)


class ScreenParameter:
    """Groups all screen related parameter"""

    def __init__(
        self,
        offset_topbar_px,
        window_pos_x_px,
        window_pos_y_px,
        monitor_resolution_x_px,
        monitor_resolution_y_px,
        camera_resolution_x_px,
        camera_resolution_y_px,
    ):
        self.__offset_topbar_px = offset_topbar_px
        self.__window_pos_x_px = window_pos_x_px
        self.__window_pos_y_px = window_pos_y_px
        self.__monitor_resolution_x_px = monitor_resolution_x_px
        self.__monitor_resolution_y_px = monitor_resolution_y_px
        self.__camera_resolution_x_px = camera_resolution_x_px
        self.__camera_resolution_y_px = camera_resolution_y_px

    @property
    def offset_topbar_px(self):
        return self.__offset_topbar_px

    @property
    def window_pos_x_px(self):
        return self.__window_pos_x_px

    @property
    def window_pos_y_px(self):
        return self.__window_pos_y_px

    @property
    def monitor_resolution_x_px(self):
        return self.__monitor_resolution_x_px

    @property
    def monitor_resolution_y_px(self):
        return self.__monitor_resolution_y_px

    @property
    def camera_resolution_x_px(self):
        return self.__camera_resolution_x_px

    @property
    def camera_resolution_y_px(self):
        return self.__camera_resolution_y_px


class CameraAdjustment:
    """The camera resolution is not the same as the monitor resolution and is additionally scaled. This class
    compensates for this."""

    def __init__(self, screen_parameter):
        self.__offset_topbar_px = screen_parameter.offset_topbar_px
        self.__image_scale_factor_y = (
            screen_parameter.monitor_resolution_y_px - self.__offset_topbar_px
        ) / float(screen_parameter.camera_resolution_y_px)
        image_width = (
            screen_parameter.camera_resolution_x_px * self.__image_scale_factor_y
        )
        self.__image_offset = (
            screen_parameter.monitor_resolution_x_px - image_width
        ) / 2.0

    def adjust_gaze_data_to_camera_resolution(self, gaze_data):
        gaze_data.gaze_pixel.x = (
            gaze_data.gaze_pixel.x - self.__image_offset
        ) / self.__image_scale_factor_y
        gaze_data.gaze_pixel.y = (
            gaze_data.gaze_pixel.y - self.__offset_topbar_px
        ) / self.__image_scale_factor_y


class ImageVisualization:
    """Visualize camera images"""

    def __init__(self, screen_parameter, compress, visualize_debug):
        self.__screen_parameter = screen_parameter
        self.__compress = compress
        self.__visualize_debug = visualize_debug
        self.__bridge = CvBridge()

    def get_image_from_msg(self, msg):
        if self.__compress:
            np_arr = np.fromstring(msg.data, np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            return self.__bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_to_msg(self, image):
        return self.__bridge.cv2_to_imgmsg(image, "bgr8")

    def display_debug_visualization(self, img, current_gaze, se_list):
        if not self.__visualize_debug:
            return

        draw_current_gaze(img, current_gaze)
        for roi in se_list:
            draw_roi(img, roi.roi_msg)

    def display_image(self, img):
        cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow(
            "window",
            self.__screen_parameter.window_pos_x_px,
            self.__screen_parameter.window_pos_y_px,
        )
        cv2.imshow("window", img)
