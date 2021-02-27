/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * DemoModule.h
 *
 *  Created on: 2016. 3. 9.
 *      Author: SCH
 */

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

//using namespace ROBOTIS_DEMO;


class GripperControl
{
private:

  // bool grap ;
  // bool release ;
  // bool grap_alc ;
  // bool grap_scraper ;
  // // int     init_argc_;
  // // char**  init_argv_; 
  // //int main(int argc, char** argv);
 
public:


  // GripperControl();
  // virtual ~GripperControl();
  // //=========robotiq2f_gripper=============================================================
  // // void ROS_init(int argc, char** argv);
  // static inline bool RecordGrapAlcohol(){return grap=true, grap_alc=true;}
  // // void RecordGrapScrapper(const std_msgs::String::ConstPtr& msg);
  // static inline bool RecordGrapRelease(){return release=true, grap=false;}
  // void robotiq_2f_gripper_Client();
  // void grapPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  // void releasePoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  // void GrapAlcoholMsgCallback(const std_msgs::String::ConstPtr& msg);
  // void ReleasePoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  //=======================================================================================
  /* */


};

}

#endif /* ROBOTIQ_2F_GRIPPER_CONTROL_H_ */
