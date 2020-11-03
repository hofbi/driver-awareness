#!/usr/bin/env python
"""Receives gaze data from the eye tracking device and publishes it to ros"""

import rospy
from gaze_detector.pupil import PupilInterface
from driver_awareness_msgs.msg import GazeArray


class GazePublisher:
    """Receive gaze data from the eye tacker and publish them to ros"""

    def __init__(self, eye_tracker):
        self.__pub_gaze = rospy.Publisher("~gaze", GazeArray, queue_size=10)
        self.__pub_fixation = rospy.Publisher("~fixation", GazeArray, queue_size=10)
        self.__eye_tracker = eye_tracker
        rospy.loginfo("Publishing gaze data")

    def update(self):
        self.__eye_tracker.update()

        self.__pub_fixation.publish(self.__eye_tracker.get_fixations())
        self.__pub_gaze.publish(self.__eye_tracker.get_gazes())


def main():
    """Run the gaze publisher"""
    rospy.init_node("gaze_publisher", anonymous=True)

    pupil_ip = rospy.get_param("~pupil_ip", "localhost")
    port = rospy.get_param("~pupil_port", 50020)
    screen_resolution_x = rospy.get_param("~screen_resolution_x", 1920)
    screen_resolution_y = rospy.get_param("~screen_resolution_y", 1080)

    eye_tracker = PupilInterface(
        pupil_ip, port, screen_resolution_x, screen_resolution_y
    )
    gaze_publisher = GazePublisher(eye_tracker)
    while not rospy.is_shutdown():
        gaze_publisher.update()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
