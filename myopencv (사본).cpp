#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>



#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
using namespace cv;
using namespace std;


float my_linear_x;
float my_angular_z;
/*
void publish(ros::Publisher &pubTeleop, float linear_x, float angular_z)
{
	geometry_msgs::Twist baseCmd;
	ros::Rate loopRate(100.0);
	baseCmd.linear.x=linear_x;
	baseCmd.angular.z=angular_z;
	pubTeleop.publish(baseCmd);
	loopRate.sleep();
}
*/
void poseMessageReceivedRGB(const sensor_msgs::ImageConstPtr& msg) 
{
//	ros::NodeHandle nhp;
//	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
//	geometry_msgs::Twist baseCmd;

	Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
	Mat img_origin = img.clone();
	vector< vector< Point> > squares;


	imshow("img_origin",img_origin);

	GaussianBlur(img, img, Size(5,5), 1 ,1);
	Mat gray0;
	Mat img_hsv;
	Mat img_mask;
	Mat img_w = img.clone();
	int width, height;
	int err;
	int cx, cy;
	float h,w,d;
	float search_top, search_bot;


	cvtColor(img, gray0, CV_BGR2GRAY);
	cvtColor(img, img_hsv, CV_BGR2HSV);
	Mat gray(img.size(), CV_8U);

	Scalar lower_yellow(48,40,100);
	Scalar upper_yellow(60,255,100);

//	Scalar lower_yellow(20,100,100,0);
//	Scalar upper_yellow(30,255,255,0);

//	Scalar lower_yellow(10,10,10);
//	Scalar upper_yellow(255,255,250);
//	inRange(img_hsv, lower_yellow, upper_yellow,img_mask);
	inRange(img_hsv, lower_yellow, upper_yellow,img_mask);

		h=img.rows;
		w=img.cols;
		
		search_top=3*h/4;
		search_bot=3*h/4;+20;
		printf("h : %f w: %f st:%f sb:%f\n",h,w,search_top,search_bot);

/*
		for(int i=0; i<search_top; i++)
		{
			for(int j=0; j<w; j++)
			{
				img_mask.at<int>(i,j)=0;
			}
		}
		for(int i=search_bot; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				img_mask.at<int>(i,j)=0;
			}
		}
		
	
//		mask[0:search_top, 0:w]=0
//		mask[search_bot:h, 0:w]=0
	Moments m=moments(img_mask,true);
	if(m.m00>0)
	{
		printf("sssssssssssssssssssssssssssssssssssssssssss");		
		cx=int(m.m10/m.m00);
		cy=int(m.m01/m.m00);
		Point p(cx,cy);			
		circle(img_w, p, 20, Scalar(0,0,255), -1);
		err=cx-w/2;
		
//		baseCmd.linear.x=0.2;
//		baseCmd.angular.z=-float(err)/100;
//		pub.publish(baseCmd);
		my_linear_x=0.2;
		my_angular_z=-float(err)/100;	


		

		

	}
*/	
		imshow("new_img", img_hsv);
		waitKey(30);


	

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "myopencv");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);
//	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &poseMessageReceivedRGB,
	ros::VoidPtr(), image_transport::TransportHints("compressed"));

	

	ros::spin();
	return 0;

}


