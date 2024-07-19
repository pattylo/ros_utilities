# ROS Utilities
This is a personal libraries for ROS packages. Support transformation operation, Jacobians calculation, and some other stuff.

## Setup
In ```CMakeLists.txt``` do the following:

Under 
```
include_directories(
  include
  ${catkin_INCLUDE_DIRS}

  #########################################
  # ADD HERE!!!
  ros_utilities
  #########################################
)
```
and under 
```
find_package(catkin COMPONENTS
  nodelet
  roscpp
  std_msgs
  roslaunch
  geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  genmsg
  visualization_msgs
  tf

  #########################################
  # ADD HERE!!!
  ros_utilities
  #########################################
)
```