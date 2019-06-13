#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

#include <gsl/gsl_fit.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))
using namespace cv;
using namespace std;


float trap_bottom_width = 0.85;  // width of bottom edge of trapezoid, expressed as percentage of image width
float trap_top_width = 0.07;     // ditto for top edge of trapezoid
float trap_height = 0.4; 
float max_length=0;
Vec4i tempv;
float robot_theta=0;

void draw_line(Mat &img_line, vector<Vec4i> lines)
{
	if (lines.size() == 0) return;


	// In case of error, don't draw the line(s)
	bool draw_right = true;
	bool draw_left = true;
	int width = img_line.cols;
	int height = img_line.rows;
	float length=0;

	//Find slopes of all lines
	//But only care about lines where abs(slope) > slope_threshold
	float slope_threshold = 0.5;
	vector<float> slopes;
	vector<Vec4i> new_lines;
	float theta=0;
	for (int i = 0; i < lines.size(); i++)
	{
		 Vec4i l = lines[i];

		if(l[3]>400 &&l[0]<200)
			{
			line(img_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 1, CV_AA);
			length=sqrt(pow((l[0]-l[2]),2.0)+pow((l[3]-l[1]),2.0));
			if(i==0)
			{
				max_length=length;
				tempv[0]=l[0];
				tempv[1]=l[1];
				tempv[2]=l[2];
				tempv[3]=l[3];
			}
			if(max_length<length)
			{
				max_length=length;
				tempv[0]=l[0];
				tempv[1]=l[1];
				tempv[2]=l[2];
				tempv[3]=l[3];
			}
	







			}
		else
		{
        	line(img_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
		}	
	}
		line(img_line, Point(tempv[0],tempv[1]), Point(tempv[2], tempv[3]), Scalar(255,0,0),1 ,CV_AA);
		printf("max length :%f\n", max_length);
		theta=atan2((tempv[3]-tempv[1]),(tempv[0]-tempv[2]));
		theta=toDegree(theta);
		if(theta>0)
		{
			robot_theta=90-theta;
		}
		if(theta<0)
		{
			robot_theta=90-(-theta);		
		}
		printf("robot_theta : %f theta:%f\n",robot_theta, theta);
		





}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "myopencv");
	ros::NodeHandle nh;

	float rho = 2; // distance resolution in pixels of the Hough grid
	float theta = 1 * CV_PI / 180; // angular resolution in radians of the Hough grid
	float hough_threshold = 15;	 // minimum number of votes(intersections in Hough grid cell)
	float minLineLength = 10; //minimum number of pixels making up a line
	float maxLineGap = 20;	//maximum gap in pixels between connectable line segments
	
	Mat img=imread("/home/hyejin/robotP/image.png",IMREAD_GRAYSCALE);
	Mat img_rsz;
	img_rsz=img.clone();
	printf("row: %d cols: %d\n",img.rows, img.cols);
	resize(img_rsz,img_rsz, Size(555,555),0,0,CV_INTER_LINEAR);
	printf("row: %d cols: %d\n",img_rsz.rows, img_rsz.cols);

	
	GaussianBlur(img_rsz, img_rsz, Size(5,5), 1 ,1);
//	cvtColor(img_rsz, img_rsz, CV_BGR2GRAY);

 	Canny(img_rsz, img_rsz, 125, 250);
	threshold(img_rsz, img_rsz, 170,255,THRESH_BINARY);

	vector<Vec4i> lines;
	HoughLinesP(img_rsz, lines, 1, theta, 20);
	Mat img_line = Mat::zeros(img_rsz.rows, img_rsz.cols, CV_8UC3);

	draw_line(img_line, lines);


	namedWindow("Original", WINDOW_AUTOSIZE);
	imshow("Original", img_line);	

	while(cvWaitKey(0) <0);

	return 0;

}

	
