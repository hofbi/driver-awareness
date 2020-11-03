cmake_minimum_required(VERSION 3.0)
project(awareness_detector)

find_package(
  catkin REQUIRED
  COMPONENTS roslaunch
             driver_awareness_msgs
             message_generation
             cv_bridge
             sensor_msgs
             rospy
             std_msgs)

catkin_python_setup()

catkin_package()

catkin_add_nosetests(test)

# Roslaunch check
roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)
