#ifndef ROBOTIQ_2F_GRIPPER_CONTROL_H_
#define ROBOTIQ_2F_GRIPPER_CONTROL_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>


//===================robotiq2f_gripper=================
#include "robotiq_2f_gripper_msgs/CommandRobotiqGripperAction.h"
//====================================================

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>




namespace robotiq_2f_gripper
{



class GripperControl
{
private:

  bool grap ;
  bool release ;
  bool grap_alc ;
  bool grap_scraper ;
  int     init_argc_;
  char**  init_argv_;
  // int     init_argc_;
  // char**  init_argv_; 
  // //int main(int argc, char** argv);
 
public:


  GripperControl(int argc, char **argv);
  ~GripperControl();
  // //=========robotiq2f_gripper=============================================================
  //void ROS_init(int argc, char** argv);
  inline bool RecordGrapAlcohol(){return grap=true, grap_alc=true;}
  // void RecordGrapScrapper(const std_msgs::String::ConstPtr& msg);
  inline bool RecordGrapRelease(){return release=true, grap=false;}
  void robotiq_2f_gripper_Client();
  void init();
  void grapPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void releasePoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void GrapAlcoholMsgCallback(const std_msgs::String::ConstPtr& msg);
  void ReleasePoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  //=======================================================================================
  /* */


};

}

#endif /* ROBOTIQ_2F_GRIPPER_CONTROL_H_ */
