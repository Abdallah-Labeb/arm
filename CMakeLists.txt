cmake_minimum_required(VERSION 3.0.2)
project(color_sorting_arm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  gazebo_ros
  tf
  tf2_ros
  cv_bridge
  image_transport
  urdf
  xacro
  xacro
  message_generation
)

add_message_files(
  FILES
  DetectedObject.msg
  DetectedObjectArray.msg
  Object3D.msg
  Object3DArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    trajectory_msgs
    control_msgs
    gazebo_ros
    tf
    tf2_ros
    cv_bridge
    image_transport
    urdf
    xacro
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/color_detector.py
  scripts/position_estimator.py
  scripts/motion_planner.py
  scripts/gripper_controller.py
  scripts/sorting_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
