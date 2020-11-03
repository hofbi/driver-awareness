cmake_minimum_required(VERSION 3.0)
project(driver_awareness_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation opencv_apps std_msgs)

add_message_files(FILES ROI.msg ROIArray.msg SA.msg Gaze.msg GazeArray.msg)

generate_messages(DEPENDENCIES opencv_apps geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
