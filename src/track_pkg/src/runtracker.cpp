#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include "kcftracker.hpp"

using namespace std;
using namespace cv;
static const std::string RGB_WINDOW = "RGB Image window";
//static const std::string DEPTH_WINDOW = "DEPTH Image window";

#define Max_linear_speed 0.6
#define Min_linear_speed 0.4
#define Min_distance 1.5
#define Max_distance 5.0
#define Max_rotation_speed 0.75

float linear_speed = 0;
float rotation_speed = 0;

float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

float k_rotation_speed = 0.004;
float h_rotation_speed_left = 1.2;
float h_rotation_speed_right = 1.36;
 
int ERROR_OFFSET_X_left1 = 100;
int ERROR_OFFSET_X_left2 = 300;
int ERROR_OFFSET_X_right1 = 340;
int ERROR_OFFSET_X_right2 = 540;
bool has_dectect_people=false;
int roi_height = 100;
int roi_width = 100;

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;


bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

float dist_val[5] ;

void detectimg(Mat rgbimage)
{	
	  if(has_dectect_people)
		return;
	  HOGDescriptor hog;   //HOG特征检测器 
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());//设置SVM分类器为默认参数  
    vector<Rect> found, found_filtered;
    hog.detectMultiScale(rgbimage, found, 0, Size(16, 16), Size(32, 32), 1.05, 2);  //对图像进行多尺度检测，检测窗口移动步长为(8,8)  
     //找出所有没有嵌套的矩形框r,并放入found_filtered中,如果有嵌套的话,则取外面最大的那个矩形框放入found_filtered中  
    size_t i,j;
    for ( i = 0; i < found.size(); i++)
		{
			Rect r = found[i];
			for ( j=0; j < found.size(); j++)
				if (j != i && (r & found[j]) == r)
					break;
				if (j == found.size())
					found_filtered.push_back(r);
		}
	//画矩形框
	Rect r;
	for ( i = 0; i<found_filtered.size(); i++)
		{
			Rect r = found_filtered[i];
			r.x += cvRound(r.width*0.1);
			r.width = cvRound(r.width*0.8);
			r.y += cvRound(r.height*0.07);
			r.height = cvRound(r.height*0.8);
			rectangle(rgbimage, r.tl(), r.br(), Scalar(0, 255, 0), 3);
      printf("r.x==%d,y==%d,width==%d,height==%d\n",r.x,r.y,r.width,r.height);
		
		
	if(r.width>80&&r.height>150)  //设置检测阈值
	{		has_dectect_people=true;	
			bRenewROI = true;
      selectRect.x = r.x;
      selectRect.y = r.y;
      selectRect.width = r.width;
      selectRect.height = r.height;
      //selectRect &= cv::Rect(0,0,rgbimage.cols,rgbimage.rows);
      printf("selectRect.x==%d,y==%d,width==%d,height==%d\n",selectRect.x,selectRect.y,selectRect.width,selectRect.height);
	}	
  else
  {
    has_dectect_people=false;
  }}
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  
public:
  ros::Publisher pub;

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
      &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image", 1, 
      &ImageConverter::depthCb, this);
    pub = nh_.advertise<geometry_msgs::Twist>("tracker/cmd_vel", 1000);
	
    cv::namedWindow(RGB_WINDOW);
    //cv::namedWindow(DEPTH_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    //cv::destroyWindow(DEPTH_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image.copyTo(rgbimage);

    detectimg(rgbimage);

    if(bRenewROI)
    {
       
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    if(bBeginKCF)
    {
        result = tracker.update(rgbimage);
        cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
        enable_get_depth = false;
    }
    else
        cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

    cv::imshow(RGB_WINDOW, rgbimage);
    cv::waitKey(1);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
  	cv_bridge::CvImagePtr cv_ptr;
  	try
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
  		cv_ptr->image.copyTo(depthimage);
  	}
  	catch (cv_bridge::Exception& e)
  	{
  		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
  	}

    if(enable_get_depth)
    {
      dist_val[0] = depthimage.at<float>(result.y+result.height/3 , result.x+result.width/3) ;
      dist_val[1] = depthimage.at<float>(result.y+result.height/3 , result.x+2*result.width/3) ;
      dist_val[2] = depthimage.at<float>(result.y+2*result.height/3 , result.x+result.width/3) ;
      dist_val[3] = depthimage.at<float>(result.y+2*result.height/3 , result.x+2*result.width/3) ;
      dist_val[4] = depthimage.at<float>(result.y+result.height/2 , result.x+result.width/2) ;

      float distance = 0;
      int num_depth_points = 5;
      for(int i = 0; i < 5; i++)
      {
        if(dist_val[i] > 0.4 && dist_val[i] < 10.0)
          distance += dist_val[i];
        else
          num_depth_points--;
      }
      distance /= num_depth_points;

      //calculate linear speed
      if(distance > Min_distance)
        linear_speed = distance * k_linear_speed + h_linear_speed;
      else
        linear_speed = 0;

      if(linear_speed > Max_linear_speed)
        linear_speed = Max_linear_speed;

      //calculate rotation speed
      int center_x = result.x + result.width/2;
      if(center_x < ERROR_OFFSET_X_left1) 
        rotation_speed =  Max_rotation_speed;
      else if(center_x > ERROR_OFFSET_X_left1 && center_x < ERROR_OFFSET_X_left2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left;
      else if(center_x > ERROR_OFFSET_X_right1 && center_x < ERROR_OFFSET_X_right2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right;
      else if(center_x > ERROR_OFFSET_X_right2)
        rotation_speed = -Max_rotation_speed;
      else 
        rotation_speed = 0;

      std::cout <<  "linear_speed = " << linear_speed << "  rotation_speed = " << rotation_speed << std::endl;

      // std::cout <<  dist_val[0]  << " / " <<  dist_val[1] << " / " << dist_val[2] << " / " << dist_val[3] <<  " / " << dist_val[4] << std::endl;
      // std::cout <<  "distance = " << distance << std::endl;
    }

  	//cv::imshow(DEPTH_WINDOW, depthimage);
  	cv::waitKey(1);
  }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kcf_tracker");
	ImageConverter ic;
  
	while(ros::ok())
	{
		ros::spinOnce();

    geometry_msgs::Twist twist;
    twist.linear.x = linear_speed; 
    twist.linear.y = 0; 
    twist.linear.z = 0;
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = rotation_speed;
    ic.pub.publish(twist);

		if (cvWaitKey(33) == 'q')
      break;
	}

	return 0;
}

