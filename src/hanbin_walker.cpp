#include <ros/console.h>
#include <cmath>
#include <limits>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>

geometry_msgs::Twist msg;
boost::mutex mutex;
sensor_msgs::LaserScan g_scan;
int changeCourse_ = 0;  ///< bool variable to indicate whether the robot has to

void processLaserScan(ros::Publisher &pubTeleop);
void navigate(ros::Publisher &pubTeleop, int changeCourse);

template<typename T>
inline bool isnan(T value)
{
	return value != value;
}


void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	mutex.lock(); {
		g_scan = msg;
	} mutex.unlock();
}


void processLaserScan(ros::Publisher &pubTeleop) {
	sensor_msgs::LaserScan scan;

	  mutex.lock(); {
		scan = g_scan;
	  } mutex.unlock();
	
	int nRangeSize = (int)scan.ranges.size();
	int count = 0;
	float sum = 0;
	float average = 9999;
	
	

	for (int i = 0; i < nRangeSize; i++) {
		double dRange = scan.ranges[i];

		if (!isnan(dRange) && dRange <= 0.9) {
			count++;
			sum += dRange;
		}
		if (i == 40) i += 300;
	}

	if (count == 0) {
		printf("-40~40도 사이의 전면장애물까지의 평균거리 : %s\n", "물체 없음.");
		return;
	}
	else {
		average = sum / count;
	}
	printf("-40~40도 사이의 전면장애물까지의 평균거리 : %f\n", average);
	//충돌가능성 탐지
	if (average <= 0.40) {
		changeCourse_ = 1;
		// return;
	}

	navigate(pubTeleop, changeCourse_);
}

void navigate(ros::Publisher &pubTeleop, int changeCourse) {
  // the default behavior is to keep going straight. If change in course is
  // requested, the turtlebot is commanded to rotate
  if (changeCourse) {
    msg.linear.x = 0.0;
    msg.angular.z = -0.5;
  } else {
    msg.linear.x = 0.5;
    msg.angular.z = 0.0;
  }

  // Publish the desired velocity commands
  pubTeleop.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hanbin_walker");

  ROS_INFO_STREAM("Walker Node Initialized!");
  ros::NodeHandle np, ns;
  ros::Publisher velocityPub_ = np.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Subscriber laserSub_ = ns.subscribe("scan", 50, &scanMsgCallback);

  // Change logger level to DEBUG.
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  

  processLaserScan(velocityPub_);
  

  ros::spin();

  return 0;
}
