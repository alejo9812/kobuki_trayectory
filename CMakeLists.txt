cmake_minimum_required(VERSION 3.0.2)
project(kobuki_trayectory)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  gazebo_ros
  geometry_msgs
  rospy
  kobuki_description
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(

)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})