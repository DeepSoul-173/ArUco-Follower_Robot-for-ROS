 Imports

rospy: Used to interact with ROS nodes.
sensor_msgs.msg.Image: ROS message type for camera image data.
geometry_msgs.msg.Twist: ROS message type for robot velocity commands.
cv_bridge.CvBridge: Converts ROS image messages into OpenCV images.
cv2 & cv2.aruco: For image processing and ArUco marker detection.
numpy: For handling camera calibration parameters and numerical operations.


Modified CMakeLists.txt for Tortoise Bot Firmware:

cmake_minimum_required(VERSION 3.0.2)
project(tortoisebot_firmware)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  cv_bridge       # Add cv_bridge to handle image messages
  opencv2         # Add OpenCV package for image processing
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp roslib rospy std_msgs cv_bridge opencv2
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/aruco_detector.py  # Make sure this is the correct Python script location
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries if needed (for future tests)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tortoisebot_firmware.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
