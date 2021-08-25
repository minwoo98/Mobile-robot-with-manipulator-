#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sstream>

using namespace std;
using namespace cv;

bool Success = false;

bool takeImg_Snapshot()
{
  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::ImageConstPtr img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/usb_cam/image_raw");

  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
  }catch(cv_bridge::Exception &e)
  {
    ROS_ERROR("Error to convert!");
    return false;
  }

  imwrite("test.jpg", cv_ptr->image);
  waitKey(1);

  return true; 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;
  Mat img_origin;


  Success = takeImg_Snapshot();
  /*
  if(Success)
  {
    ROS_INFO("Loading img.. \n");
    img_origin = imread("test.jpg", IMREAD_COLOR);
    imshow("s", img_origin);
  }
  else;
*/

  ros::spin();

  return 0;
}
