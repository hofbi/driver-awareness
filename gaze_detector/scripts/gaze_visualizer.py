#!/usr/bin/env python
"""Visualizes gaze data and performs a accuracy test"""
from pathlib import Path
import rospkg
import matplotlib.pyplot as plt
import numpy as np
import rospy
import pandas as pd
from driver_awareness_msgs.msg import GazeArray
from gaze_detector.geometry import PygameColor, GazeData, Circle
from gaze_detector import files, geometry
import pygame
import time


class GazeVisualizer:
    """Visualizes gaze data and performs a accuracy test"""

    def __init__(self):
        self.__gaze_data = GazeData()
        self.__fixation_data = GazeData()

        gaze_topic = rospy.get_param("~gaze_topic", "/gaze_publisher/gaze")
        fixation_topic = rospy.get_param("~fixation_topic", "/gaze_publisher/fixation")
        x_px = rospy.get_param("~screen_resolution/x_px", 1920)
        y_px = rospy.get_param("~screen_resolution/y_px", 1080)
        self.__window_origin_px = (
            rospy.get_param("~window_pos_x_px", 0),
            rospy.get_param("~window_pos_y_px", 0),
        )

        # Set enviroment variable to set the origin of the pygame window
        os.environ[
            "SDL_VIDEO_WINDOW_POS"
        ] = f"{self.__window_origin_px[0]},{self.__window_origin_px[1]}"

        self.__image = pygame.image.load(rospy.get_param("~calibration_image_path"))

        # Load config file for circles on calibration image
        circle_file = pd.read_csv(rospy.get_param("~calibration_circles_path"))
        self.__circles = [Circle.from_pd(row) for _, row in circle_file.iterrows()]
        self.__circle_counter = 0

        pygame.init()
        self.__display_surface = pygame.display.set_mode(
            (x_px, y_px), pygame.FULLSCREEN
        )
        self.__display_surface.blit(self.__image, self.__window_origin_px)
        pygame.display.update()
        time.sleep(2)
        self.draw_circle()

        self.__sub_gaze = rospy.Subscriber(gaze_topic, GazeArray, self.gaze_callback)
        self.__sub_fixation = rospy.Subscriber(
            fixation_topic, GazeArray, self.fixation_callback
        )
        rospy.loginfo("Reading data from topics")
        rospy.loginfo("Gaze: " + gaze_topic)
        rospy.loginfo("Fixation: " + fixation_topic)

    def draw_circle(self):
        pygame.draw.circle(
            self.__display_surface,
            PygameColor.RED,
            self.__circles[self.__circle_counter].position,
            5,
        )
        pygame.display.update()
        time.sleep(2)

    def stop_visualizer(self):
        pygame.quit()
        rospy.loginfo("Finished recording.")

        self.__sub_gaze.unregister()
        self.__sub_fixation.unregister()

        rospy.loginfo("Save data and calculate mean dist from circles")
        self.save_visualized_data()

    def is_calibration_running(self):
        return self.__circle_counter < len(self.__circles)

    def update(self):
        self.__circle_counter += 1
        self.__display_surface.blit(self.__image, self.__window_origin_px)
        self.draw_circle()

    def gaze_callback(self, data):
        for gaze in data.gazes:
            self.__gaze_data.append(gaze)

    def fixation_callback(self, data):
        for gaze in data.gazes:
            self.__fixation_data.append(gaze)

    def save_visualized_data(self):
        pkg_path = rospkg.RosPack().get_path("gaze_detector")
        data_folder_path = Path(pkg_path).joinpath("data")
        data_folder_path.mkdir(exist_ok=True, parents=True)
        data_path = files.get_rotating_data_path(data_folder_path)
        data_path.mkdir(exists=True)

        np.save(data_path.joinpath("npDataGaze"), self.__gaze_data.to_numpy_array())
        np.save(
            data_path.joinpath("npDataFixation"),
            self.__fixation_data.to_numpy_array(),
        )

        fig_gaze, ax_gaze = files.create_figure_from_gaze_data(
            pkg_path, self.__fixation_data
        )
        rospy.loginfo("Saving gaze data..")
        fig_gaze.savefig(data_path.joinpath("gazeData_img.pdf"), dpi=400)
        plt.close(fig_gaze)

        fig_fixation, ax_fixation = files.create_figure_from_gaze_data(
            pkg_path, self.__fixation_data
        )

        rospy.loginfo(f"Circles: {self.__circles}")
        for (dist, index_closes_circle) in geometry.find_closest_circle_distances(
            self.__circles, self.__fixation_data
        ):
            self.__circles[index_closes_circle].append_gaze_inside_dist(dist)

        for circle in self.__circles:
            if circle.has_gazes_inside():
                ax_fixation.text(
                    circle.circle_x + 70,
                    circle.circle_y,
                    str(round(circle.mean_dist(), 3)) + " p",
                    fontsize=5,
                    color="red",
                )

        overall_mean_dist = np.mean(
            [
                circle.mean_dist()
                for circle in self.__circles
                if circle.has_gazes_inside()
            ]
        )
        rospy.loginfo(f"Overall mean dist: {overall_mean_dist} pixels")
        ax_fixation.text(
            100,
            100,
            f"Overall mean dist pixel error: {round(overall_mean_dist, 3)}p",
            fontsize=7,
            color="red",
        )
        rospy.loginfo("Saving fixation data..")
        plt.title("Fixations on calibration image")
        fig_fixation.savefig(data_path.joinpath("fixationData_img.pdf"), dpi=400)
        plt.close(fig_fixation)


def main():
    """Starts the visualization of a GazeVisualizer instance"""
    rospy.init_node("gaze_visualizer", anonymous=True)

    gaze_visualizer = GazeVisualizer()
    time.sleep(2)
    while not rospy.is_shutdown() and gaze_visualizer.is_calibration_running():
        gaze_visualizer.update()

    gaze_visualizer.stop_visualizer()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
