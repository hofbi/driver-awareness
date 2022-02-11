"""Files Operation Module"""

from pathlib import Path

import matplotlib.pyplot as plt


def get_rotating_data_path(data_folder_path):
    """Get next available path for data folder"""
    data_counter = 0
    while Path(data_folder_path).joinpath(f"data{data_counter}").exists():
        data_counter += 1
    return Path(data_folder_path).joinpath(f"data{data_counter}")


def create_figure_from_gaze_data(package_path, gaze_data):
    """Create figure handle and axis from gaze data"""
    img_path = Path(package_path).joinpath("config").joinpath("calibration_image.png")
    img = plt.imread(str(img_path))

    fig = plt.figure(dpi=400)
    axis = fig.add_subplot(111)
    axis.imshow(img)

    for gaze_x, gaze_y, on_surface in zip(
        gaze_data.gaze_x,
        gaze_data.gaze_y,
        gaze_data.on_surface,
    ):
        data_point_color = "blue" if on_surface else "red"
        axis.scatter(
            gaze_x,
            gaze_y,
            color=data_point_color,
            s=3,
        )
    return fig, axis
