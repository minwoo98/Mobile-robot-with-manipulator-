#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sstream>

using namespace std;
using namespace cv;


void ImgCallback(const sensor_msgs::ImageConstPtr &img)
{
  cv_bridge::CvImagePtr cv_ptr;

  Mat img_gray;
  Mat thresh;
  Mat img_result;

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  vector<Point2f> approx;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("Error to convert!");
    return;
  }

  cvtColor( cv_ptr->image, img_gray ,CV_BGR2GRAY);
  threshold(img_gray, thresh, 150, 255, CV_THRESH_BINARY);

  findContours(thresh, contours,RETR_TREE, CHAIN_APPROX_NONE);

  img_result = cv_ptr->image.clone();

  for( size_t i=0; i<contours.size();i++)
  {
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
  }
  //findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
  //cv::imshow("img show", cv_ptr->image);
  //drawContours(thresh, contours, -1, Scalar(0,255,0), 2);
 
  imshow("Binary_img", thresh);
  waitKey(1);
  
}

 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;
  
  Mat img_test;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("/usb_cam/image_raw", 1, ImgCallback);
 
  
  ROS_INFO("main");
 

  ros::spin();

  return 0;
}
