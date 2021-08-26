#include <string.h>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"
#include "open_manipulator_pick_and_place.h"
#include "etri_nav/main_control.h"

int Global_count = 0;
int finish_picking = 0;

void statusCallback(const actionlib_msgs::GoalStatusArray &msg)
{
  int status = 0;

  //ROS_INFO("Status: %d \n", status);
  if(!msg.status_list.empty())
  {
    status = msg.status_list[0].status;
  } 

  if(status == 3 && Global_count== 0) 
  {
    Global_count = 1;
    ROS_INFO("robot arrived. \n");
    return;
  }

  if(status == 3 && Global_count == 3)
  {
    
    ROS_INFO("Placing start. \n");
    Global_count = 4;
    return;
  }
  else;
  
}
void finish_pick_Callback(const etri_nav::main_control &msg)
{
  //Global_count++;
  finish_picking = msg.finish_pick;
  if(finish_picking== 1)
  {
    Global_count = 2;
  }  
}

void Main_Callback(const ros::TimerEvent&)
{
  switch (Global_count)
  {
  case 0:
    break;

  case 1:
    arrived_msg.start_pick = 1;
    start_pick_pub.publish(arrived_msg);
    ROS_INFO("arrived-> pick start \n");
    break;

  case 2:
    arrived_msg.go_back = 1;
    for(int i=0; i<6000; i++)  
    {
      ROS_INFO("wait \n");
    }
    go_back_pub.publish(arrived_msg);
    Global_count = 3;
    break;
  case 3:
    break;

  case 4:  
    arrived_msg.start_place = 1;
    start_place_pub.publish(arrived_msg);
    ROS_INFO("placing.. \n");
 
    break;
  }
  
}


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "main_control");
  ros::NodeHandle nh;

  start_pick_pub = nh.advertise<etri_nav::main_control>("start_pick", 1);
  go_back_pub = nh.advertise<etri_nav::main_control>("go_back", 1);
  start_place_pub = nh.advertise<etri_nav::main_control>("start_place", 1);

  move_base_state_sub = nh.subscribe("move_base/status", 1, statusCallback);
  finish_pick_sub = nh.subscribe("/finish_pick", 10, finish_pick_Callback);

  ros::Timer publish_timer = nh.createTimer(ros::Duration(0.200)/*100ms*/, &Main_Callback);

  while(nh.ok())
  {
    ros::spinOnce();
  }

  ros::spin();

  return 0;
}
