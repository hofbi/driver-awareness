#!/usr/bin/env python
"""
This module is used to perform remote calibration of the Pupil Core by connecting to the pupil capture
software and displaying calibration markers on the screen using pygame
"""
import rospy
import pygame
from gaze_detector import pupil
from gaze_detector.pupil import PupilRemoteCommands
from gaze_detector.geometry import PygameColor
import msgpack
import time


class RemoteCalibration:
    """Executes remote calibration of the Pupil Core by connecting to the pupil capture software and displaying
    calibration markers"""

    def __init__(self):
        self.__calibration_point_counter = 0
        # ROS Parameters
        pupil_ip = rospy.get_param("~pupil_ip", "localhost")
        port = rospy.get_param("~pupil_port", 50020)
        x_px = rospy.get_param("~screen_resolution/x_px", 1920)
        y_px = rospy.get_param("~screen_resolution/y_px", 1080)
        offset_topbar_px = rospy.get_param("~offset_topbar_px", 30)
        markersize_px = rospy.get_param("~marker/size_px", 160)
        border_offset_px = rospy.get_param("~marker/border_offset_px", 10)

        # Remove topbar from screen height
        y_px -= offset_topbar_px

        # Load marker image
        image = pygame.image.load(rospy.get_param("~marker_path"))
        self.__image = pygame.transform.scale(image, (markersize_px, markersize_px))

        # Connect to pupil app
        self.__pupil_remote, self.__pupil_subscriber = pupil.connect_pupil(
            pupil_ip, port
        )
        self.__pupil_subscriber.subscribe("logging.debug")

        # Init Pygame window
        pygame.init()
        self.__display_surface = pygame.display.set_mode(
            (x_px, y_px), pygame.FULLSCREEN
        )
        pygame.display.set_caption("Pupil Capture Remote Calibration")

        # Substract length of marker, makes it easier to place the marker on the screen
        x2_px = x_px - markersize_px
        y2_px = y_px - markersize_px

        # Create calibration point positions
        self.__marker_positions_px = [
            ((x2_px / 2.0), y2_px / 2.0),
            (border_offset_px, y2_px - border_offset_px),
            (x2_px - border_offset_px, y2_px - border_offset_px),
            (x2_px - 10, border_offset_px),
            (border_offset_px, border_offset_px),
        ]

    def stop_calibration(self):
        self.__pupil_remote.send_string(PupilRemoteCommands.STOP_CALIBRATION)
        rospy.loginfo(self.__pupil_remote.recv_string())

    def display_marker(self, position):
        self.__display_surface.fill(PygameColor.WHITE)
        self.__display_surface.blit(self.__image, position)
        pygame.display.update()

    def is_calibration_running(self):
        return self.__calibration_point_counter < len(self.__marker_positions_px)

    def update(self):
        topic, payload = self.__pupil_subscriber.recv_multipart()
        message = msgpack.loads(payload)

        if pupil.is_calibration_point_finished(message):
            self.__calibration_point_counter += 1
            time.sleep(0.7)
            rospy.loginfo("Changing calibration image.")
            self.display_marker(
                self.__marker_positions_px[self.__calibration_point_counter]
            )

    def start_calibration(self):
        self.__calibration_point_counter = 0
        self.display_marker(self.__marker_positions_px[0])

        # Wait shortly, so person is ready
        time.sleep(0.5)

        self.__pupil_remote.send_string(PupilRemoteCommands.START_CALIBRATION)
        recv = self.__pupil_remote.recv_string()

        return recv == PupilRemoteCommands.CALIBRATION_STARTED_SUCCESSFULLY


def main():
    """Starts the remote calibration using a RemoteCalibration instance"""
    rospy.init_node("remote_calibration", anonymous=True)

    remote_calibration = RemoteCalibration()
    if remote_calibration.start_calibration():
        while not rospy.is_shutdown() and remote_calibration.is_calibration_running():
            remote_calibration.update()
    else:
        rospy.logerror("Error starting calibration.")

    remote_calibration.stop_calibration()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
