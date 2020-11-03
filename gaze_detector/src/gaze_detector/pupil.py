"""Receive data from the pupil capture software"""

import msgpack
import rospy
import zmq
from driver_awareness_msgs.msg import Gaze, GazeArray
from geometry_msgs.msg import Point
from gaze_detector import geometry


class PupilRemoteCommands:
    """Class contains the start and stop command which can be send to remote control the pupil capture software"""

    START_CALIBRATION = "C"
    STOP_CALIBRATION = "c"
    CALIBRATION_STARTED_SUCCESSFULLY = "OK"


def is_calibration_point_finished(message):
    """Check if calibration for a calibration marker is done"""
    return "manual_marker_calibration" in str(message[b"name"]) and "Sampled" in str(
        message[b"msg"]
    )


def connect_pupil(pupil_ip, port):
    """Connects to pupil capture software via zmq"""
    ctx = zmq.Context()
    # The REQ talks to Pupil remote and receives the session unique IPC SUB PORT
    pupil_remote = ctx.socket(zmq.REQ)
    pupil_remote.setsockopt(zmq.RCVTIMEO, 2000)

    rospy.loginfo(f"Connecting to: tcp://{pupil_ip}:{port}")
    pupil_remote.connect(f"tcp://{pupil_ip}:{port}")

    try:
        # Request 'SUB_PORT' from the pupil remote for reading data
        # (pupel remote plugin has to be running in capture GUI)
        pupil_remote.send_string("SUB_PORT")
        sub_port = pupil_remote.recv_string()
    except ():
        rospy.logerr("Connection to Pupil Remote failed. Check IP and Port.")
        exit()

    # Request 'PUB_PORT' from the pupil remote for writing data (pupil remote plugin has to be running in capture GUI)
    pupil_remote.send_string("PUB_PORT")
    pub_port = pupil_remote.recv_string()
    rospy.loginfo(f"Requesting PUB_PORT: {pub_port}")

    # Assumes `sub_port` to be set to the current subscription port
    pupil_subscriber = ctx.socket(zmq.SUB)
    pupil_subscriber.connect(f"tcp://{pupil_ip}:{sub_port}")
    rospy.loginfo(f"Subscribing to: tcp://{pupil_ip}:{sub_port}")

    return pupil_remote, pupil_subscriber


class PupilInterface:
    """Receives gaze data from pupil capture software"""

    def __init__(self, pupil_ip, port, screen_resolution_x, screen_resolution_y):
        self.__message = None
        self.__surface_name = None
        self.__screen_resolution_x = screen_resolution_x
        self.__screen_resolution_y = screen_resolution_y

        _, self.__subscriber = connect_pupil(pupil_ip, port)

        rospy.loginfo("Connected to ZMQ.")
        self.__subscriber.subscribe("surfaces.")  # receive all surfaces messages

        rospy.loginfo("Connection established.")

    def update(self):
        # Receive packet
        topic = self.__subscriber.recv_multipart()
        # Deserialize packet
        self.__message = msgpack.loads(topic[1])
        # Decode surface name
        self.__surface_name = self.__message[b"name"].decode("utf-8")

    def get_fixations(self):
        return self.create_gaze_pixel_data_from_message(b"fixations_on_surfaces")

    def get_gazes(self):
        return self.create_gaze_pixel_data_from_message(b"gaze_on_surfaces")

    def create_gaze_pixel_data_from_message(self, message_name):
        gaze_array = GazeArray()
        for data_point in self.__message[message_name]:
            msg = Gaze()
            msg.surface_name = self.__surface_name
            msg.on_surface = data_point[b"on_surf"]
            msg.confidence = data_point[b"confidence"]
            # Create Point for message with normalized data
            point_gaze_norm = Point()
            point_gaze_norm.x = data_point[b"norm_pos"][0]
            point_gaze_norm.y = data_point[b"norm_pos"][1]
            msg.gaze_norm = point_gaze_norm
            # Create Point for message with data in pixels
            msg.gaze_pixel = geometry.norm_to_pixel(
                point_gaze_norm, self.__screen_resolution_x, self.__screen_resolution_y
            )
            gaze_array.gazes.append(msg)

        return gaze_array
