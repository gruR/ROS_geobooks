#include <cmath>
#include <limits>
#include <vector>
#include <ros/console.h>

#include "object_avoidance.hpp"

Walker::Walker(ros::NodeHandle &n) : n_(n) 
{
  //ROS_INFO_STREAM("Lidar Node Initialized!\n");

  // Setup the velocity publisher topic with master
  velocityPub_ =
      n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

  // subscribe to the laserscan topic
  laserSub_ = n_.subscribe("scan", 50, &Walker::processLaserScan, this);
}

Walker::~Walker() {
}


void Walker::processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan) 
{

  auto minRange = scan->range_min;
  auto maxRange = scan->range_max;

  // Get angle resolution
  auto angleRes = scan->angle_increment;

  // Extract scan range values
  std::vector<float> rangeValues = scan->ranges;
  size_t size = rangeValues.size();
  auto mid = size / 2;

  // Check for obstacle within a range of 20 degrees from center i.e 10 degrees
  // on left and 10 degrees on the right of center
  float checkRange = 10 * M_PI / 180;
  size_t checkRangeIncrement = checkRange / angleRes;

  // Get minimum range value in that region of interest
  float minDist = std::numeric_limits<float>::max();

  // 
  for (size_t it = mid - checkRangeIncrement; it < mid + checkRangeIncrement; it++) 
  {
    if (rangeValues[it] < minDist) {
      minDist = rangeValues[it];
    }
  }


  // If the scan value is invalid, then keep going straight.
  if (std::isnan(minDist) || minDist > maxRange || minDist < minRange)
  {
    changeCourse_ = false;
    navigate(changeCourse_);

    return;
  }

  // direction change based on minimum distance
  if (minDist < 1.0) {
    ROS_WARN_STREAM("Obstacle detected ahead! Changing course\n");
    changeCourse_ = true;
  } else {
    changeCourse_ = false;
  }

  // Publish navigation command to turtlebot
  navigate(changeCourse_);
}


void Walker::navigate(bool changeCourse) 
{
  // changeCourse=true -> change direction
  if (changeCourse) {
    msg.linear.x = 0.0;
    msg.angular.z = -0.5;
  } else {
    msg.linear.x = 0.1;
    msg.angular.z = 0.0;
  }

  // Publish the desired velocity commands
  velocityPub_.publish(msg);

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Lidar_node");
  ros::NodeHandle nh;

  // Change logger level to DEBUG.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Construct the walker class for navigation of turtlebot
  Walker walker(nh);

  ros::spin();

  return 0;

}

