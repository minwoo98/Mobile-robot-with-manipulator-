#include "ros/ros.h"
#include "etri_nav/InitTurtlebotPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"
#include <string.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "open_manipulator_pick_and_place.h"

int status;
int is_arrived;
int count = 0;
int nav;
int going_back;


class Navigation
{
public:
  Navigation()
  {
    node_handle_.getParam("init_pose/position", init_pose_position);
    node_handle_.getParam("init_pose/orientation", init_pose_orientation);

    pub_initial_pose = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    pubPoseStamped = node_handle_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    pubServiceStatus = node_handle_.advertise<std_msgs::String>("/log",1); //pub status
    //start_navigation_sub = node_handle_.subscribe("/start_navigation", 1, start_Callback);
    start_navigation_sub = node_handle_.subscribe("/start_navigation", 10, &Navigation::start_Callback, this);

    //place_start_pub = node_handle_.advertise<etri_nav::main_control>("start_pla", 1);
    start_pick_pub = node_handle_.advertise<etri_nav::main_control>("start_pick", 1);

    go_back_sub = node_handle_.subscribe("go_back", 1, &Navigation::go_back_Callback, this);


    //initialization
    is_pose_initialized = fnSetInitialPose();
    is_sending_goal_ready = set_first_goal();
 
  }
  bool fnSetInitialPose()
  {
    geometry_msgs::PoseWithCovarianceStamped pose_initialization;
    
    ///pose_init
    pose_initialization.header.frame_id = "map";
    pose_initialization.header.stamp = ros::Time::now();

    pose_initialization.pose.pose.position.x = init_pose_position[0];
    pose_initialization.pose.pose.position.y = init_pose_position[1];
    pose_initialization.pose.pose.position.z = init_pose_position[2];

    pose_initialization.pose.pose.orientation.x = init_pose_orientation[0];
    pose_initialization.pose.pose.orientation.x = init_pose_orientation[1];
    pose_initialization.pose.pose.orientation.z = init_pose_orientation[2];
    pose_initialization.pose.pose.orientation.w = init_pose_orientation[3];
  
    pose_initialization.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

    ros::Rate poll_rate(100);
    while(pub_initial_pose.getNumSubscribers() == 0)
    { 
	    poll_rate.sleep();
    }
  
    pub_initial_pose.publish(pose_initialization);
   
    return true;
  }

  bool set_first_goal()
  {
    //picking place initialization
    node_handle_.getParam("picking_pose/position", picking_pose_position);
    node_handle_.getParam("picking_pose/orientation", picking_pose_orientation);

    poseStamped_picking.header.frame_id = "map";
    poseStamped_picking.header.stamp = ros::Time::now();

    poseStamped_picking.pose.position.x = picking_pose_position[0];
    poseStamped_picking.pose.position.y = picking_pose_position[1];
    poseStamped_picking.pose.position.z = picking_pose_position[2];

    poseStamped_picking.pose.orientation.x = picking_pose_orientation[0];
    poseStamped_picking.pose.orientation.y = picking_pose_orientation[1];
    poseStamped_picking.pose.orientation.z = picking_pose_orientation[2];
    poseStamped_picking.pose.orientation.w = picking_pose_orientation[3];

    return true;
  }

  void Sending_first_goal()
  {
    pubPoseStamped.publish(poseStamped_picking);
    is_pose_initialized = false;
    is_sending_goal_ready = false;
    //ROS_INFO("IN..");
  } 

  void Sending_second_goal()
  {
    node_handle_.getParam("placing_pose/position", placing_pose_position);
    node_handle_.getParam("placing_pose/orientation", placing_pose_orientation);

    poseStamped_place.header.frame_id = "map";
    poseStamped_place.header.stamp = ros::Time::now();

    poseStamped_place.pose.position.x = placing_pose_position[0];
    poseStamped_place.pose.position.y = placing_pose_position[1];
    poseStamped_place.pose.position.z = placing_pose_position[2];

    poseStamped_place.pose.orientation.x = placing_pose_orientation[0];
    poseStamped_place.pose.orientation.x = placing_pose_orientation[1];
    poseStamped_place.pose.orientation.z = placing_pose_orientation[2];
    poseStamped_place.pose.orientation.w = placing_pose_orientation[3];

    pubPoseStamped.publish(poseStamped_place);
    ROS_INFO("Return Navigation Start! \n");
  } 

  void start_Callback(const etri_nav::main_control &msg)
  {
    int nav = msg.start_navigation;
    if(nav == 1)  Sending_first_goal();
  }
  void go_back_Callback(const etri_nav::main_control &msg)
  {
    int going_back = msg.go_back;
    if(going_back == 1)  Sending_second_goal();
  }

private:
  ros::NodeHandle node_handle_;

  //initialpose
  ros::Publisher pub_initial_pose;
  ros::Publisher pub_twist;
  //send goal
  ros::Publisher pubServiceStatus;
  ros::Publisher pubPoseStamped;
  //
  ros::Subscriber move_base_state_sub;

  //geometry_msgs::PoseStamped pubPoseStamped;
  geometry_msgs::PoseStamped poseStamped_picking;
  geometry_msgs::PoseStamped poseStamped_place;


  std::vector<double> init_pose_position;
  std::vector<double> init_pose_orientation;
  std::vector<double> picking_pose_position;
  std::vector<double> picking_pose_orientation;
  std::vector<double> placing_pose_position;
  std::vector<double> placing_pose_orientation;
 
  bool is_pose_initialized = false;
  bool is_sending_goal_ready = false;
  bool is_sending_goal_ready2 = false;

  int robot_scenario = 0;
};

//
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pose_initialization");
  ros::NodeHandle node_handle_;
  ROS_INFO("Ready...");
  
  //Create an object of class PoseInitialization that will take care of everything
  Navigation navigation;
  
  ros::spin();

  return 0;
}

