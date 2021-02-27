#include <stdio.h>
#include <vector>

#include "robotiq_2f_gripper_control/gripper_control.h"

#include <cstdlib>
#include <vector>
#include <boost/thread.hpp>

using namespace robotiq_2f_gripper;

bool grap ;
bool release ;
bool grap_alc ;
bool grap_scraper ; 

//==========================================roboticq2f_85======================================================
void robotiq_2f_gripper_Client(){
  actionlib::SimpleActionClient<robotiq_2f_gripper_msgs::CommandRobotiqGripperAction> grap_action_client_("/command_robotiq_action");
  grap_action_client_.waitForServer(ros::Duration(0.0));
  // ROS_INFO("Connected to grap_action server");

  // ROS_INFO("Action server started.");
  robotiq_2f_gripper_msgs::CommandRobotiqGripperGoal goal;

  bool False = false;
  goal.emergency_release = False;
  goal.stop = False;
  std::cout<<grap_alc<<std::endl;
  if(grap_alc == true){
      goal.position = 0.065;
      goal.speed = 0.1;
      goal.force = 0.5;
  }
  else if(grap_scraper == true){
      goal.position = 0.04;
      goal.speed = 0.1;
      goal.force = 400;
  }
  else if(release == true){
      goal.position = 1.001;
      goal.speed = 1.1;
      goal.force = 400.0;
  }
  else{
      goal.position = 1.001;
      goal.speed = 1.1;
      goal.force = 400.0;
  }

  grap_action_client_.sendGoalAndWait(goal);
  ROS_INFO("start send goal to Gripper");
  bool finished_before_timeout = grap_action_client_.waitForResult(ros::Duration(0.0));

  if (finished_before_timeout)                             
  {
    actionlib::SimpleClientGoalState state = grap_action_client_.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    return;
  }

  else
  {
    ROS_ERROR("Failed to call service");
    return;
  }
  ROS_INFO("Action did not finish before the time out.");
  grap = false;
  release = false;
  grap_alc = false;
  grap_scraper = false; 
  return;
}
//===============================================================
void ReleasePoseMsgCallback(const std_msgs::String::ConstPtr& msg){

  ROS_INFO("Connected to release_action server");

  ROS_INFO("Action server started.");

  if(msg->data=="Gripper_release")
  {
    release = true;
    grap_alc = false;
    grap_scraper = false; 
    grap = false;
    ROS_INFO("start send Release gripper");
    if(release == true && grap == false){
      // std::cout<<"RRRRRRRRR"<<std::endl;
      robotiq_2f_gripper_Client();
    }
  }
  return;
}
//===============================================================
void GrapAlcoholMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Connected to grap_action server");

  ROS_INFO("Action server started.");
  //robotis_->is_ik = false;

  
  if(msg->data=="Gripper_grap")
  {
    grap = true;
    grap_alc = true;
    grap_scraper = false; 
    release = false;
    ROS_INFO("start send Release gripper");
    if(grap == true && grap_alc == true){
      // std::cout<<"GGGGGGG"<<std::endl;
      robotiq_2f_gripper_Client();
    }

  }
  return;
}

int main(int argc, char **argv){

  ros::init(argc, argv,"gripper_node");
  ros::NodeHandle    gripper_node;
  
  //================================robitq2f_85========================================================================                                                           
  ros::Subscriber grap_pose_msg_sub = gripper_node.subscribe("grap_alcohol_msg", 5, &GrapAlcoholMsgCallback); 
  ros::Subscriber release_pose_msg_sub = gripper_node.subscribe("release_pose_msg", 5, &ReleasePoseMsgCallback);                                                            
  ///=================================================================================================================

  ros::spin();
}