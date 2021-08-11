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

/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "manipulator_h_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_h_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc_(argc),
  init_argv_(argv)
{}

QNode::~QNode() {
  std::ofstream fout(pose_log_path.c_str());
  fout << pose_log.c_str();
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc_,init_argv_,"manipulator_h_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  // Add your ros communications here.
  specific_pose_msg_pub_ = n.advertise<std_msgs::String>("specific_pose_msg", 0);
  set_mode_msg_pub_ = n.advertise<std_msgs::String>("set_mode_msg", 0);

  joint_pose_msg_pub_ = n.advertise<manipulator_h_base_module_msgs::JointPose>("joint_pose_msg", 0);
  kinematics_pose_msg_pub_ = n.advertise<manipulator_h_base_module_msgs::KinematicsPose>("kinematics_pose_msg", 0);
  p2p_pose_msg_pub_ = n.advertise<manipulator_h_base_module_msgs::P2PPose>("p2p_pose_msg", 0);
  moveit_pose_msg_pub_ = n.advertise<manipulator_h_base_module_msgs::P2PPose>("moveit_pose_msg", 0);

  //////////////////////////robitq2f_85////////////////////////////////////////////////////////////////////////                                                        
  grap_alcohol_msg_pub_ = n.advertise<std_msgs::String>("grap_alcohol_msg", 0);
  release_pose_msg_pub_ = n.advertise<std_msgs::String>("release_pose_msg", 0);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////

  get_joint_pose_client_ = n.serviceClient<manipulator_h_base_module_msgs::GetJointPose>("get_joint_pose", 0);
  get_kinematics_pose_client_ = n.serviceClient<manipulator_h_base_module_msgs::GetKinematicsPose>("get_kinematics_pose", 0);
  joy_calib_client_ = n.serviceClient<manipulator_h_joystick::JoyCalibration>("/joy_calib", 0);

  status_msg_sub_ = n.subscribe("status", 10, &QNode::statusMsgCallback, this);

  name_ = n_private.param<std::string>("robot_name", "FUCK");
  self_name_ = QString::fromUtf8(name_.c_str());

  pose_log_path = ros::package::getPath("manipulator_h_gui") + "/log/" + name_ + "_pose_log.yaml";
  start();
  return true;
}

void QNode::run() {

  ros::Rate loop_rate(50);

  while ( ros::ok() )
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "] ";

  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendJointPoseMsg( manipulator_h_base_module_msgs::JointPose msg )
{
  joint_pose_msg_pub_.publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

void QNode::sendKinematicsPoseMsg( manipulator_h_base_module_msgs::KinematicsPose msg )
{
  kinematics_pose_msg_pub_.publish( msg );

  log( Info , "Send Kinematics Pose Msg" );
}

void QNode::sendP2PPoseMsg( manipulator_h_base_module_msgs::P2PPose msg )
{
  p2p_pose_msg_pub_.publish( msg );

  log( Info , "Send P2P Pose Msg" );
}

void QNode::sendMoveItPoseMsg( manipulator_h_base_module_msgs::P2PPose msg )
{
  moveit_pose_msg_pub_.publish( msg );

  log( Info , "Send MoveIt Pose Msg" );
}

//========robotiq_2f_gripper=======================
void QNode::sendGrapAlcoholMsg( std_msgs::String msg )
{
  grap_alcohol_msg_pub_.publish( msg );

  log( Info , "Send Grap Alcohol Msg" );
}
void QNode::sendReleasePoseMsg( std_msgs::String msg )
{
  release_pose_msg_pub_.publish( msg );

  log( Info , "Send Release Pose Msg" );
}
//=============================================================
void QNode::sendSpecificPoseMsg( std_msgs::String msg )
{
  specific_pose_msg_pub_.publish ( msg );

  log( Info , "Go to Manipulator Initial Pose" );
}

void QNode::sendSetModeMsg( std_msgs::String msg )
{
  set_mode_msg_pub_.publish ( msg );

  log( Info , "Set BaseModule" );
}

void QNode::getJointPose( std::vector<std::string> joint_name )
{
  log( Info , "Get Current Joint Pose" );

  manipulator_h_base_module_msgs::GetJointPose _get_joint_pose;

  // request
  for ( int _id = 0; _id < joint_name.size(); _id++ )
    _get_joint_pose.request.joint_name.push_back( joint_name[ _id ] );

  // response
  if ( get_joint_pose_client_.call ( _get_joint_pose ) )
  {
    manipulator_h_base_module_msgs::JointPose _joint_pose;

    for ( int _id = 0; _id < _get_joint_pose.response.joint_name.size(); _id++ )
    {
      _joint_pose.name.push_back( _get_joint_pose.response.joint_name[ _id ] );
      _joint_pose.value.push_back( _get_joint_pose.response.joint_value[ _id ]);
    }
    _joint_pose.slide_pos = _get_joint_pose.response.slide_pos;
    
    Q_EMIT updateCurrentJointPose( _joint_pose );
  }
  else
    log(Error, "fail to get joint pose.");
}

void QNode::getKinematicsPose ( std::string group_name )
{
  log( Info , "Get Current Kinematics Pose" );

  manipulator_h_base_module_msgs::GetKinematicsPose _get_kinematics_pose;

  // request
  _get_kinematics_pose.request.group_name = group_name;

  // response
  if ( get_kinematics_pose_client_.call( _get_kinematics_pose ) )
  {
    double pos[3] = {_get_kinematics_pose.response.group_pose.position.x,
                     _get_kinematics_pose.response.group_pose.position.y,
                     _get_kinematics_pose.response.group_pose.position.z};
    double ori[3] = {_get_kinematics_pose.response.euler[0] * RADIAN2DEGREE,
                     _get_kinematics_pose.response.euler[1] * RADIAN2DEGREE,
                     _get_kinematics_pose.response.euler[2] * RADIAN2DEGREE};
    double phi = _get_kinematics_pose.response.phi * RADIAN2DEGREE;
    for(int i=0; i<3; i++)
    {
      pos[i] = std::round(pos[i]/0.001) * 0.001;
      ori[i] = std::round(ori[i]/0.001) * 0.001;
    }
    pose_log << YAML::Flow << YAML::BeginSeq;

    pose_log << YAML::BeginSeq;
    for(int i=0; i<3; i++)
      pose_log << std::round(pos[i]/0.001) * 0.001;
    pose_log << YAML::EndSeq << YAML::BeginSeq;
    for(int i=0; i<3; i++)
      pose_log << std::round(ori[i]/0.001) * 0.001;
    pose_log << YAML::EndSeq;
    pose_log << std::round(phi/0.001) * 0.001;
    pose_log << YAML::EndSeq;
    
    Q_EMIT updateCurrentKinematicsPose( _get_kinematics_pose );
  }
  else
    log(Error, "fail to get kinematcis pose.");
}

void QNode::getCurrPose(double (&data)[7])
{
  manipulator_h_base_module_msgs::GetKinematicsPose _get_kinematics_pose;
 
  // request
  _get_kinematics_pose.request.group_name = "arm";

  // response
  if ( get_kinematics_pose_client_.call( _get_kinematics_pose ) )
  {
    data[0] = _get_kinematics_pose.response.group_pose.position.x;
    data[1] = _get_kinematics_pose.response.group_pose.position.y;
    data[2] = _get_kinematics_pose.response.group_pose.position.z;
    data[3] = _get_kinematics_pose.response.euler[0] * RADIAN2DEGREE;
    data[4] = _get_kinematics_pose.response.euler[1] * RADIAN2DEGREE;
    data[5] = _get_kinematics_pose.response.euler[2] * RADIAN2DEGREE;
    data[6] = _get_kinematics_pose.response.phi * RADIAN2DEGREE;
  } 
  else
    log(Error, "fail to get current pose.");
  return;
    
}

bool QNode::joyCalib(bool cmd)
{
  manipulator_h_joystick::JoyCalibration _calib_cmd;
  _calib_cmd.request.calib_cmd = cmd;
  if( joy_calib_client_.call( _calib_cmd ) )
    return _calib_cmd.response.calib_status;
}
}  // namespace manipulator_h_gui
