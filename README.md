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

## Usage
### Example I
Within in you executable cpp, just do the following:
```
#########################################
# USE HERE!!!
#include <ros_utilities/ros_utilities.h> 
// include many header files, check source file for more.
static ros_utilities ros_tools;
#########################################

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_log");
    ros::NodeHandle nh;

    // SUB, PUB, and WHATNOT

    #########################################
    # USE HERE!!!
    geometry_msgs::PoseStamped ugv_pose;
    Sophus::SE3d ugvPoseSE3 = ros_tools.posemsg_to_SE3(ugv_pose.pose);
    #########################################

    ros::spin();

    return 0;
}
```
### Example II
Within in you class header, just do the following:
```
#########################################
# USE HERE!!!
#include <ros_utilities/ros_utilities.h> 
// include many header files, check source file for more.
#########################################

class yourClass : public ros_utilities
{
  public:
    yourClass(ros::NodeHandle& nh){};
    yourClass(){};

  private:
  #########################################
  // now you can use all the function within this!!!
  #########################################
}