cmake_minimum_required(VERSION 3.0)
project(gaze_detector)

find_package(catkin REQUIRED COMPONENTS roslaunch rospy std_msgs geometry_msgs
                                        driver_awareness_msgs)

catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

catkin_add_nosetests(test)

# Roslaunch check
roslaunch_add_file_check(launch)
