#include <ros/console.h>
#include <cmath>
#include <limits>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Subscriber sub;
ros::Publisher pub;
bool changeCourse_;
geometry_msgs::Twist msg;

void navigate(bool changeCourse) {
  if (changeCourse) {
    msg.linear.x = 0.0;
    msg.angular.z = -0.5;
  } else {
    msg.linear.x = 0.5;
    msg.angular.z = 0.0;
  }
  pub.publish(msg);
}

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
  auto minRange = scan->range_min;
  auto maxRange = scan->range_max;
  auto angleRes = scan->angle_increment;
  std::vector<float> rangeValues = scan->ranges;
  size_t size = rangeValues.size();
  auto mid = size / 2;
  float checkRange = 10 * M_PI / 180;
  size_t checkRangeIncrement = checkRange / angleRes;
  float minDist = std::numeric_limits<float>::max();
	
	printf("%f\n", mid);
  for (size_t it = mid - checkRangeIncrement; it < mid + checkRangeIncrement; it++) {
    if (rangeValues[it] < minDist)
      minDist = rangeValues[it];
  }

  if (std::isnan(minDist) || minDist > maxRange || minDist < minRange) {
    changeCourse_ = false;
    navigate(changeCourse_);
    return;
  }

  if (minDist < 0.7) {
    ROS_WARN_STREAM("detect obstacle");
    changeCourse_ = true;
  } else
    changeCourse_ = false;

  navigate(changeCourse_);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "avoid_collision");
  ros::NodeHandle np, ns;
  pub = np.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);
  sub = ns.subscribe("scan", 50, &processLaserScan);

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ros::spin();

  return 0;
}
