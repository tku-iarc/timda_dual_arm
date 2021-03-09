#ifndef STRATEGY_MSG_H_
#define STRATEGY_MSG_H_


#include <map>
#include <ros/ros.h>
#include <string>
//#include <QThread>
//#include <QStringListModel>

#include <eigen3/Eigen/Eigen>

#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


#include <std_msgs/String.h>
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include <manipulator_h_base_module_msgs/JointPose.h>
#include <manipulator_h_base_module_msgs/KinematicsPose.h>
#include <manipulator_h_base_module_msgs/P2PPose.h>
#include <manipulator_h_base_module_msgs/GetJointPose.h>
#include <manipulator_h_base_module_msgs/GetKinematicsPose.h>
//#include "manipulator_h_base_module_msgs/SendGoal.h"


#define DEGREE2RADIAN   (M_PI / 180.0)
#define RADIAN2DEGREE   (180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl


namespace strategy_dual_arm
{
class StrategyMsg
{
public:
  StrategyMsg(int argc, char** argv);
  virtual ~StrategyMsg();
  //~StrategyeMsg();

  bool init();
  void run();
    enum LogLevel
    {
        Debug, Info, Warn, Error, Fatal
    };

    // QStringListModel* loggingModel()
    // {
    //     return &logging_model_;
    // }
    //void log(const LogLevel &level, const std::string &msg, std::string sender = "Stragtegy");
    //void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);
    // void statusMsgCallback(robotis_controller_msgs::StatusMsg msg);
    void sendIniPoseMsg(std_msgs::String msg);
    void sendSetModeMsg(std_msgs::String msg);

    void sendJointPoseMsg(manipulator_h_base_module_msgs::JointPose msg);
    void sendKinematicsPoseMsg(manipulator_h_base_module_msgs::KinematicsPose msg);
    void sendP2PPoseMsg( manipulator_h_base_module_msgs::P2PPose msg );
    void sendMoveItPoseMsg( manipulator_h_base_module_msgs::P2PPose msg );
    //========robotiq_2f_gripper=========================================
    void sendGrapAlcoholMsg(std_msgs::String msg);
    void sendReleasePoseMsg(std_msgs::String msg);
    //=====================================================================

    void getCurrPose(double (&data)[7]);
    //inline QString getName(){return self_name_;}


//public Q_SLOTS:
    void getJointPose( std::vector<std::string> joint_name );
    void getKinematicsPose(std::string group_name);

//Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

    void updateCurrentJointPose(manipulator_h_base_module_msgs::JointPose);
    void updateCurrentKinematicsPose(manipulator_h_base_module_msgs::GetKinematicsPose);

private:
    int     init_argc_;
    char**  init_argv_;
    std::string  name_;
    //QString self_name_;

    ros::Publisher      chatter_publisher_;
    //QStringListModel    logging_model_;

    ros::Publisher      ini_pose_msg_pub_;
    ros::Publisher      set_mode_msg_pub_;

    ros::Publisher      joint_pose_msg_pub_;
    ros::Publisher      kinematics_pose_msg_pub_;
    ros::Publisher      p2p_pose_msg_pub_;
    ros::Publisher      moveit_pose_msg_pub_;

    ros::Publisher      grap_alcohol_msg_pub_;
    ros::Publisher      release_pose_msg_pub_;
    
    
    ros::ServiceClient  get_joint_pose_client_;
    ros::ServiceClient  get_kinematics_pose_client_;

    ros::Subscriber     status_msg_sub_;
};
}

#endif /* strategy_dual_arm_STRATEGY_MSG_H_ */
