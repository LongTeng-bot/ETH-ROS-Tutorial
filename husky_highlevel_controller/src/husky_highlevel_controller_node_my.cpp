#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"
#include <algorithm>

void scanCallback(const sensor_msgs::LaserScan& msg){
    float minval=100;
    for (int i=0;i<msg.ranges.size();i++){
      minval=std::min(minval,msg.ranges[i]);
    }
    ROS_INFO("max distance: [%s]", minval);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);

  ros::Subscriber sub=nodeHandle.subscribe("scan",10,scanCallback);


  ros::spin();
  return 0;
}
