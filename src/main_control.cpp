#include <string.h>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
//#include "etri_nav/ServiceStatus.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"
#include "open_manipulator_pick_and_place.h"
#include "etri_nav/main_control.h"
/*
void OpenManipulatorPickandPlace::setModeState(char ch)
{
  if (ch == '1')
    mode_state_ = HOME_POSE;
  else if (ch == '2')
  {
    mode_state_ = DEMO_START;
    demo_count_ = 0;
  }
  else if (ch == '3')
    mode_state_ = DEMO_STOP;
}

OpenManipulatorPickandPlace* a;
*/

void statusCallback(const actionlib_msgs::GoalStatusArray &msg)
{
  int status = 0;
  
  //ROS_INFO("Status: %d \n", status);
  if(!msg.status_list.empty()) status = msg.status_list[0].status;;

  if(status == 3) 
  {
    arrived_msg.arrived = 1;
    arrived_pub.publish(arrived_msg); // manipulator start
    ROS_INFO("robot arrived. \n");

    return;
  }
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "main_control");
  ros::NodeHandle nh;

  arrived_pub = nh.advertise<etri_nav::main_control>("arrived", 1);
  move_base_state_sub = nh.subscribe("move_base/status", 1, statusCallback);

  ros::Rate r(10.0);
  while(nh.ok())
  {
    
    ros::spinOnce();
    r.sleep();
  }


  ros::spin();

  return 0;
}
