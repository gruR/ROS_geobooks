
// Walker class declaration
// publishes navigation commands to turtlebot based on laser scan data to emulate robot behavior
 
#ifndef INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_
#define INCLUDE_TURTLEBOT_ROOMBA_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Walker {
 private:
  ros::NodeHandle n_;           
  ros::Subscriber laserSub_;    
  ros::Publisher velocityPub_;
  
  //change its course while navigating
  bool changeCourse_;  

  //publish linear x, angular z velocities
  geometry_msgs::Twist msg;  

  // laser scan data
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);

  // publish navigation commands to turtlebot
  void navigate(bool changeCourse);


 public:
  explicit Walker(ros::NodeHandle &n);

  ~Walker();

};

#endif 
