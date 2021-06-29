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
 * @file /include/manipulator_h_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef manipulator_h_gui_QNODE_HPP_
#define manipulator_h_gui_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <eigen3/Eigen/Eigen>

#include <std_msgs/String.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include <manipulator_h_base_module_msgs/JointPose.h>
#include <manipulator_h_base_module_msgs/KinematicsPose.h>
#include <manipulator_h_base_module_msgs/P2PPose.h>
#include <manipulator_h_base_module_msgs/GetJointPose.h>
#include <manipulator_h_base_module_msgs/GetKinematicsPose.h>
#include <manipulator_h_joystick/JoyCalibration.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

#endif

#define DEGREE2RADIAN   (M_PI / 180.0)
#define RADIAN2DEGREE   (180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace manipulator_h_gui
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode: public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();

    /*********************
   ** Logging
   **********************/
    enum LogLevel
    {
        Debug, Info, Warn, Error, Fatal
    };

    QStringListModel* loggingModel()
    {
        return &logging_model_;
    }
    void log(const LogLevel &level, const std::string &msg, std::string sender = "GUI");
    void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);

    void sendSpecificPoseMsg(std_msgs::String msg);
    void sendSetModeMsg(std_msgs::String msg);

    void sendJointPoseMsg(manipulator_h_base_module_msgs::JointPose msg);
    void sendKinematicsPoseMsg(manipulator_h_base_module_msgs::KinematicsPose msg);
    void sendP2PPoseMsg( manipulator_h_base_module_msgs::P2PPose msg );
    void sendMoveItPoseMsg( manipulator_h_base_module_msgs::P2PPose msg );
    //========robotiq_2f_gripper=========================================
    void sendGrapAlcoholMsg( std_msgs::String msg );
    void sendReleasePoseMsg( std_msgs::String msg );
    //=====================================================================
    void getCurrPose(double (&data)[7]);
    inline QString getName(){return self_name_;}
    bool joyCalib(bool cmd);


public Q_SLOTS:
    void getJointPose( std::vector<std::string> joint_name );
    void getKinematicsPose(std::string group_name);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

    void updateCurrentJointPose(manipulator_h_base_module_msgs::JointPose);
    void updateCurrentKinematicsPose(manipulator_h_base_module_msgs::GetKinematicsPose);

private:
    int     init_argc_;
    char**  init_argv_;
    std::string  name_;
    QString self_name_;

    ros::Publisher      chatter_publisher_;
    QStringListModel    logging_model_;

    ros::Publisher      specific_pose_msg_pub_;
    ros::Publisher      set_mode_msg_pub_;

    ros::Publisher      joint_pose_msg_pub_;
    ros::Publisher      kinematics_pose_msg_pub_;
    ros::Publisher      p2p_pose_msg_pub_;
    ros::Publisher      moveit_pose_msg_pub_;
    //robotiq_2f_gripper===========================
    ros::Publisher      grap_alcohol_msg_pub_;
    ros::Publisher      release_pose_msg_pub_;
    //==============================================
    ros::ServiceClient  get_joint_pose_client_;
    ros::ServiceClient  get_kinematics_pose_client_;
    ros::ServiceClient  joy_calib_client_;

    ros::Subscriber     status_msg_sub_;

    std::string pose_log_path;
    YAML::Emitter pose_log;
};

}  // namespace manipulator_h_gui

#endif /* manipulator_h_gui_QNODE_HPP_ */
