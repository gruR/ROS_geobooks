#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>



#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))
using namespace cv;
using namespace std;

ros::Publisher pub ;
float pre_dAngleTurned;
boost::mutex mutex;
nav_msgs::Odometry g_odom;
float my_linear_x;
float my_angular_z;

float max_length=0;
Vec4i tempv;
//float robot_theta=0;

void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // receive a '/odom' message with the mutex
    mutex.lock(); {
        g_odom = msg;
    } mutex.unlock();
}


tf::Transform
getCurrentTransformation(void)
{
    // transformation \B9\F6\C6\DB
    tf::Transform transformation;

    // odom \B9\F6\C6\DB
    nav_msgs::Odometry odom;

    // copy a global '/odom' message with the mutex
    mutex.lock(); {
        odom = g_odom;
    } mutex.unlock();

    // \C0\A7ġ \C0\FA\C0\E5
    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    // ȸ\C0\FC \C0\FA\C0\E5
    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    // \B8\AE\C5\CF
    return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// \B7κ\BF\C0\CC \B8\D8\C3\E7\C0ִ\C2 \BB\F3\C5\C2(ó\C0\BD \BB\F3\C5\C2)\C0\C7 \C0\A7ġ\B8\A6 \C0\FA\C0\E5!
tf::Transform
getInitialTransformation(void)
{
    // tf \BA\AFȯ\C7\E0\B7\C4
    tf::Transform transformation;

    // ó\C0\BD\C0\A7ġ\BF\A1 \B4\EB\C7\D1 odometry \B8޽\C3\C1\F6 \B9ޱ\E2
    ros::Rate loopRate(100.0);

    while(ros::ok()) {
        // \C0ϴ\DC callback \B8޽\C3\C1\F6\B8\A6 \B9ް\ED!
        ros::spinOnce();

        // get current transformationreturn;
        transformation = getCurrentTransformation();

        // \B8޽\C3\C1\F6\B8\A6 \B9޾\D2\C0\B8\B8\E9 break!
        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }

    // \B8\AE\C5\CF
    return transformation;
}



bool
doRotation(ros::Publisher &pubTeleop,  tf::Transform &initialTransformation,double dRotation, double dRotationSpeed)
{

    //the command will be to turn at 'rotationSpeed' rad/s

    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.5;
    baseCmd.linear.y = 0.0;
	int option=0;
    if(dRotation < 0.) {
        baseCmd.angular.z = -dRotationSpeed;
    } else {
        baseCmd.angular.z = dRotationSpeed;
    }
	printf("Ss\n");
    // \C0̵\BF\C7ϸ鼭 \C7\F6\C0\E7\C0\A7ġ\BF\A1 \B4\EB\C7\D1 odometry \B8޽\C3\C1\F6 \B9ޱ\E2
    bool bDone = false;
    ros::Rate loopRate(100.0);

	

    while(ros::ok() && !bDone) {
        // \C0ϴ\DC callback \B8޽\C3\C1\F6\B8\A6 \B9ް\ED!
	
        ros::spinOnce();

        // get current transformation
        tf::Transform currentTransformation = getCurrentTransformation();

        //see how far we've traveled
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        tf::Quaternion rotationQuat = relativeTransformation.getRotation();


        
         double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));

  // \C1\BE\B7\E1\C1\B6\B0\C7 üũ

    if( fabs(dAngleTurned) > fabs(dRotation) || (abs(pre_dAngleTurned - dRotation) <  abs(dAngleTurned - dRotation)) || (dRotation == 0)) 
	{

		printf("111111111\n");
	    bDone = true;
            break;
        } else {
	    pre_dAngleTurned = dAngleTurned;
            //send the drive command
            pubTeleop.publish(baseCmd);
		printf("222222222222222\n");
//		

            // sleep!
            loopRate.sleep();
        }
    }


    // ÃÊ±âÈ­
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}




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
	ros::NodeHandle  nhp;
	pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);	
	geometry_msgs::Twist baseCmd;



	// In case of error, don't draw the line(s)
	bool draw_right = true;
	bool draw_left = true;

	float length=0;

	float robot_theta=0;
	float robot_theta_r=0;
	//Find slopes of all lines
	//But only care about lines where abs(slope) > slope_threshold
	float slope_threshold = 0.5;
	vector<float> slopes;
	vector<Vec4i> new_lines;
	float thetas=0;

	float rho = 2; // distance resolution in pixels of the Hough grid
	float theta = 1 * CV_PI / 180; // angular resolution in radians of the Hough grid
	float hough_threshold = 15;	 // minimum number of votes(intersections in Hough grid cell)
	float minLineLength = 10; //minimum number of pixels making up a line
	float maxLineGap = 20;	//maximum gap in pixels between connectable line segments


	

	printf("hhhhhhhh\n");
	Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
	Mat gray0;
	cvtColor(img, gray0, CV_BGR2GRAY);
	Mat img_origin = gray0.clone();
	vector< vector< Point> > squares;
	Mat img_rsz;
	img_rsz=img_origin.clone();

//	printf("row: %d cols: %d\n",img.rows, img.cols);
//	resize(img_rsz,img_rsz, Size(555,555),0,0,CV_INTER_LINEAR);
//	printf("row: %d cols: %d\n",img_rsz.rows, img_rsz.cols);
	GaussianBlur(img_rsz, img_rsz, Size(5,5), 1 ,1);
 	Canny(img_rsz, img_rsz, 125, 250); //////////캐니엣지 변환
	threshold(img_rsz, img_rsz, 170,255,THRESH_BINARY); ///이진
	Mat img_line = Mat::zeros(img_rsz.rows, img_rsz.cols, CV_8UC3);
	int width = img_line.cols;
	int height = img_line.rows;


vector<Vec4i> newlines;
	vector<Vec4i> lines;
	HoughLinesP(img_rsz, lines, 1, theta, 20);/////////////////하프 라인변환(직선검출)



for(int i=0; i<lines.size(); i++)
{
printf("yyyyyyyyyy%d %d %d %d\n", lines[i][0], lines[i][1], lines[i][2],lines[i][3]);
newlines=lines;
}

//printf("%d %d %d %d\n", newlines[0][0], newlines[0][1], newlines[0][2],newlines[0][3]);

if(newlines.size()==0)
return;


//	draw_line(img_line, lines);
//	printf("%d %d %d %d\n",lines[0][0], lines[0][1], lines[0][2], lines[0][3]);

for (int i = 0; i < newlines.size(); i++)
	{
		 Vec4i l = newlines[i];
//printf("hererererere%d %d %d %d\n", newlines[0][0], newlines[0][1], newlines[0][2],newlines[0][3]);
//printf("llllllllllll%d %d %d %d\n", l[0], l[1], l[2],l[3]);

		if(l[3]>100 &&l[0]<40)
		{
			printf("nnnnnnnnnnnnnn\n");
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
		
		////검출한 직선에서 왼쪽 아래직선들 중 길이가 가장 긴 직선 검출(정확성을 위해)
		else
		{
        	line(img_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
		}
		
	}

		printf("tempv %d %d %d %d\n",tempv[0],tempv[1], tempv[2], tempv[3]);
		line(img_line, Point(tempv[0],tempv[1]), Point(tempv[2], tempv[3]), Scalar(255,0,0),1 ,CV_AA);
		printf("max length :%f\n", max_length);

		thetas=atan2((tempv[3]-tempv[1]),(tempv[0]-tempv[2]));
		thetas=toDegree(thetas);
		if(thetas>0)
		{
			robot_theta=90-thetas;
		}
		if(thetas<0)
		{
			robot_theta=90-(-thetas);
			robot_theta_r=toRadian(robot_theta);
			 tf::Transform cTransformation = getInitialTransformation();
			pre_dAngleTurned=0;

		thetas=thetas+50;



			doRotation(pub,cTransformation, robot_theta_r, 0.25);	
			//뽑은 직선의 기울기 계산 후 회전
			
//			baseCmd.angular.z=0.2;
		}
		printf("robot_theta : %f theta:%f\n",robot_theta, thetas);
		pub.publish(baseCmd);
	namedWindow("Original", WINDOW_AUTOSIZE);
	imshow("Original", img_line);

	cvWaitKey(30);



	

}


void RGB_To_HSV(double r, double g, double b, double &h, double &s, double &v)
{
double max,min;
max=MAX(r,g);
max=MAX(max,b);
min=MIN(r,g);
min=MIN(min,b);
v=max;
s=(max !=0.0) ? (max-min)/max : 0.0 ;
if(s==0.0)
	h=0;
else
{
	double delta=max-min;
	if(r==max)
		h=(g-b)/delta;
	else if(g==max)
		h=2.0+(b-r)/delta;
	else if(b==max)
		{
		h=4.0+(r-g)/delta;
		h+=60.0;
	if(h<0.0)
		h+=460.0;
		}
}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "myopencv_realimg");
	ros::NodeHandle nh,nhs, nhp;
 	   ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
	int j=0;
//	const sensor_msgs::ImageConstPtr msg;
	image_transport::ImageTransport it(nh);
	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	
//	ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	image_transport::Subscriber subRGB = it.subscribe("/camera/image", 1, &poseMessageReceivedRGB,
	ros::VoidPtr(), image_transport::TransportHints("compressed"));
	printf("j : %d\n",j);
	j++;

//	double a,b,c;
//	RGB_To_HSV(235,237,114,a,b,c);
//	printf("h: %f, s:%f, v:%f\n",a,b,c);	

	ros::spin();
	return 0;

}

