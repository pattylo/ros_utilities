# ROS Utilities
This is a personal libraries for ROS packages. Support transformation operation, Jacobians calculation, and some other stuff.

## Setup
In ```CMakeLists.txt``` do the following:

Under ```CMakeLists.txt```

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

and
```
include_directories(
    include

    #########################################
    # ADD HERE!!!
    ${catkin_INCLUDE_DIRS}
    #########################################
)
```

and
```
add_executable(
    lala 
    src/main.cpp
)
target_link_libraries(lala
    #########################################
    # ADD HERE!!!
    ${catkin_LIBRARIES}
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
// many header files included, check source file for more.
static std::shared_ptr<ros_utilities> ros_tools_ptr;
#########################################

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_util");
    ros::NodeHandle nh;

    ros_tools_ptr = std::make_shared<ros_utilities>();

    // SUB, PUB, and WHATNOT

    #########################################
    # USE HERE!!!
    geometry_msgs::PoseStamped ugv_pose;
    Sophus::SE3d ugvPoseSE3 = ros_tools_ptr->posemsg_to_SE3(ugv_pose.pose);
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
```
### Maintainer
[Patrick Lo](https://github.com/pattylo) @ AIRo-Lab, RCUAS, PolyU: patty.lo@connect.polyu.hk <br/> 
