#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include <string>
#include <vector>
//#include "float.h"
#include "math.h"

class Pose{
public:
  float x = 0;
  float y = 0;
  float z = 0;
  float angle=0;
  float dist=0;
public:
  void callback(const sensor_msgs::LaserScan& msg);
  void print_pose(){
    /*ROS_INFO("pos x is [%f]",x);
    ROS_INFO("pos y is [%f]",y);*/
    std::cout<<"x is: "<<x<<" y is: "<<y<<std::endl;
  };
};

void Pose::callback(const sensor_msgs::LaserScan& msg){
// Exercise 2 stuff
/*    float minval=100;
    for (int i=0;i<msg.ranges.size();i++){
      minval=std::min(minval,msg.ranges[i]);
    }
    ROS_INFO("min distance: [%f]", minval);*/

    //ROS_INFO("get min dis");
    //std::cout<<"min distance: "<<minval<<std::endl;

// Exercise 3: Get pillow position
    float right_angle,right_dist;
    for (int i=0;i<msg.ranges.size();i++){
      if (msg.ranges[i]<FLT_MAX){
          right_dist=msg.ranges[i];
          right_angle=msg.angle_min+msg.angle_increment*i;
          break;
      }
    }
    float left_angle,left_dist;
    for (int i=msg.ranges.size()-1;i>=0;i--){
      if (msg.ranges[i]<FLT_MAX){
        left_dist=msg.ranges[i];
        left_angle=msg.angle_max-msg.angle_increment*(msg.ranges.size()-i);
      }
    }

    float mid_dist=(left_dist+right_dist)/2.0, mid_angle=(left_angle+right_angle)/2.0;
    x=cos(mid_angle)*mid_dist;
    y=sin(mid_angle)*mid_dist;
    z=0;
    dist=mid_dist;
    angle=mid_angle;
    //ROS_INFO("x is [%f]",x);
    //ROS_INFO("y is [%f]",y);



}

void pubMarker(int x, int y, int z, ros::Publisher& pubPoint){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
//marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  pubPoint.publish( marker );

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle;
  std::string topic;
  int queueSize;
  nodeHandle.getParam("/husky_highlevel_controller_node/topic_name",topic);
  ROS_INFO("show something 1");
  nodeHandle.getParam("/husky_highlevel_controller_node/queue_size",queueSize);
  float K_dist,K_angle;
  nodeHandle.getParam("/husky_highlevel_controller_node/rotation_gain",K_angle);
  nodeHandle.getParam("/husky_highlevel_controller_node/distance_gain",K_dist);

  Pose pose;
  ros::Subscriber sub=nodeHandle.subscribe(topic,queueSize,&Pose::callback,&pose);
  ros::Publisher pub=nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel",1);
  ros::Publisher pubPoint=nodeHandle.advertise<visualization_msgs::Marker>("pillar_location",1);
  /*ROS_INFO("pos x is [%f]",pose.x);
  ROS_INFO("pos y is [%f]",pose.y);*/
  //ROS_INFO("show something 2");
  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle);
  //ROS_INFO("show something 3");
  ros::Rate loop_rate(10);
  while (ros::ok()){
    //ROS_INFO("pos x is [%f]",pose.x);
    //ROS_INFO("pos y is [%f]",pose.y);

    geometry_msgs::Twist command;
    command.angular.x=0;
    command.angular.y=0;
    command.angular.z=-K_angle*pose.angle;
    command.linear.x=K_dist*pose.dist;
    command.linear.y=0;
    command.linear.z=0;

    pubMarker(pose.x,pose.y,pose.z,pubPoint);
    pub.publish(command);



    ros::spinOnce();
    loop_rate.sleep();
  }
  //pose.print_pose();
  //std::cout<<"work or not?"<<std::endl;
  //ros::spin();

  return 0;
}
