#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "strategy_dual_arm/strategy_msg.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace strategy_dual_arm {

/*****************************************************************************
** Implementation
*****************************************************************************/

StrategyMsg::StrategyMsg(int argc, char** argv ) :
  init_argc_(argc),
  init_argv_(argv)
{}

StrategyMsg::~StrategyMsg() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  //wait();
}

bool StrategyMsg::init() {
  ros::init(init_argc_,init_argv_,"Strategy_Msg");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nm;
  ros::NodeHandle n_private("~");

  // Add your ros communications here.
  ini_pose_msg_pub_ = nm.advertise<std_msgs::String>("ini_pose_msg", 0);
  set_mode_msg_pub_ = nm.advertise<std_msgs::String>("set_mode_msg", 0);

  joint_pose_msg_pub_ = nm.advertise<manipulator_h_base_module_msgs::JointPose>("joint_pose_msg", 0);
  kinematics_pose_msg_pub_ = nm.advertise<manipulator_h_base_module_msgs::KinematicsPose>("kinematics_pose_msg", 0);
  p2p_pose_msg_pub_ = nm.advertise<manipulator_h_base_module_msgs::P2PPose>("p2p_pose_msg", 0);
  moveit_pose_msg_pub_ = nm.advertise<manipulator_h_base_module_msgs::P2PPose>("moveit_pose_msg", 0);
  //======================robitq2f_85====================================================================================
  grap_alcohol_msg_pub_ = nm.advertise<std_msgs::String>("grap_alcohol_msg", 0);
  release_pose_msg_pub_ = nm.advertise<std_msgs::String>("release_pose_msg", 0);
  //==============================================================================================================/
  get_joint_pose_client_ = nm.serviceClient<manipulator_h_base_module_msgs::GetJointPose>("get_joint_pose", 0);
  get_kinematics_pose_client_ = nm.serviceClient<manipulator_h_base_module_msgs::GetKinematicsPose>("get_kinematics_pose", 0);

  //status_msg_sub_ = n.subscribe("status", 10, &StrategyeMsg::statusMsgCallback, this);

  name_ = n_private.param<std::string>("robot_name", "FUCK");
  //self_name_ = QString::fromUtf8(name_.c_str());

  //start();
  return true;
}

void StrategyMsg::run() {

  ros::Rate loop_rate(50);

  while ( ros::ok() )
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close stragety." << std::endl;
  //Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


// void StrategyeMsg::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
// {
//   ROS_INFO((LogLevel) msg->type, msg->status_msg, msg->module_name);
// }

// void StrategyeMsg::log( const LogLevel &level, const std::string &msg, std::string sender)
// {
//   logging_model_.insertRows(logging_model_.rowCount(),1);
//   std::stringstream logging_model_msg;

//   std::stringstream _sender;
//   _sender << "[" << sender << "] ";

//   switch ( level ) {
//   case(Debug) : {
//     ROS_DEBUG_STREAM(msg);
//     logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
//     break;
//   }
//   case(Info) : {
//     ROS_INFO_STREAM(msg);
//     logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
//     break;
//   }
//   case(Warn) : {
//     ROS_WARN_STREAM(msg);
//     logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
//     break;
//   }
//   case(Error) : {
//     ROS_ERROR_STREAM(msg);
//     logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
//     break;
//   }
//   case(Fatal) : {
//     ROS_FATAL_STREAM(msg);
//     logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
//     break;
//   }
//   }
//   QVariant new_row(QString(logging_model_msg.str().c_str()));
//   logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
//   Q_EMIT loggingUpdated(); // used to readjust the scrollbar
// }

void StrategyMsg::sendJointPoseMsg( manipulator_h_base_module_msgs::JointPose msg )
{
  joint_pose_msg_pub_.publish( msg );

  ROS_INFO( "Send Joint Pose Msg" );
}

void StrategyMsg::sendKinematicsPoseMsg( manipulator_h_base_module_msgs::KinematicsPose msg )
{
  kinematics_pose_msg_pub_.publish( msg );

  ROS_INFO( "Send Kinematics Pose Msg" );
}

void StrategyMsg::sendP2PPoseMsg( manipulator_h_base_module_msgs::P2PPose msg )
{
  p2p_pose_msg_pub_.publish( msg );

  ROS_INFO( "Send P2P Pose Msg" );
}
void StrategyMsg::sendMoveItPoseMsg( manipulator_h_base_module_msgs::P2PPose msg )
{
  moveit_pose_msg_pub_.publish( msg );

  ROS_INFO( "Send MoveIt Pose Msg" );
}

//========robotiq_2f_gripper=======================
void StrategyMsg::sendGrapAlcoholMsg( std_msgs::String msg )
{
  grap_alcohol_msg_pub_.publish( msg );

  ROS_INFO( "Send Grap Alcohol Msg" );
}
void StrategyMsg::sendReleasePoseMsg( std_msgs::String msg )
{
  release_pose_msg_pub_.publish( msg );

  ROS_INFO( "Send Release Pose Msg" );
}
//=============================================================
void StrategyMsg::sendIniPoseMsg( std_msgs::String msg )
{
  ini_pose_msg_pub_.publish ( msg );

  ROS_INFO( "Go to Manipulator Initial Pose" );
}

void StrategyMsg::sendSetModeMsg( std_msgs::String msg )
{
  set_mode_msg_pub_.publish ( msg );
  ROS_INFO( "Set BaseModule" );
}

void StrategyMsg::getJointPose( std::vector<std::string> joint_name )
{
  ROS_INFO( "Get Current Joint Pose" );

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
    
    //Q_EMIT updateCurrentJointPose( _joint_pose );
  }
  else
    ROS_INFO( "fail to get joint pose.");
}

void StrategyMsg::getKinematicsPose ( std::string group_name )
{
  ROS_INFO( "Get Current Kinematics Pose" );

  manipulator_h_base_module_msgs::GetKinematicsPose _get_kinematics_pose;

  // request
  _get_kinematics_pose.request.group_name = group_name;

  // response
  if ( get_kinematics_pose_client_.call( _get_kinematics_pose ) )
  {
    manipulator_h_base_module_msgs::KinematicsPose _kinematcis_pose;

    _kinematcis_pose.name = _get_kinematics_pose.request.group_name;
    _kinematcis_pose.pose = _get_kinematics_pose.response.group_pose;
    _kinematcis_pose.phi  = _get_kinematics_pose.response.phi;


    //Q_EMIT updateCurrentKinematicsPose( _get_kinematics_pose );
  }
  else
    ROS_INFO( "fail to get kinematcis pose.");
}

void StrategyMsg::getCurrPose(double (&data)[7])
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
    ROS_INFO( "fail to get current pose.");
  return;
    
}
}  
